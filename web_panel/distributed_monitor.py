import json
import logging
import os
from datetime import datetime, timedelta
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger(__name__)
DEFAULT_ZENOH_ENDPOINT = "tcp/127.0.0.1:7447"
ZENOH_ENDPOINT_ENV_VAR = "AGI_WALKER_ZENOH_ENDPOINT"
ACTOR_TTL_ENV_VAR = "AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS"
DEFAULT_ACTOR_TTL_SECONDS = 30.0


class DistributedMonitor:
    def __init__(self) -> None:
        self.zenoh_session: Optional[Any] = None
        self.actors: Dict[str, Dict[str, Any]] = {}
        self.endpoint = os.getenv(ZENOH_ENDPOINT_ENV_VAR, DEFAULT_ZENOH_ENDPOINT)
        self.actor_ttl_seconds = self._read_actor_ttl_seconds()
        self.zenoh_available = False
        self.monitor_active = False
        self.last_error: Optional[str] = None
        self.last_pruned_at: Optional[str] = None
        self.last_pruned_count: int = 0

    def _read_actor_ttl_seconds(self) -> float:
        raw_value = os.getenv(ACTOR_TTL_ENV_VAR)
        if raw_value is None:
            return DEFAULT_ACTOR_TTL_SECONDS
        try:
            return max(1.0, float(raw_value))
        except ValueError:
            return DEFAULT_ACTOR_TTL_SECONDS

    def _prune_stale_actors(self) -> int:
        now = datetime.now()
        cutoff = now - timedelta(seconds=self.actor_ttl_seconds)
        stale_actor_ids = []
        for actor_id, actor in self.actors.items():
            last_seen_raw = actor.get("last_seen")
            if not isinstance(last_seen_raw, str):
                continue
            try:
                last_seen = datetime.fromisoformat(last_seen_raw)
            except ValueError:
                continue
            if last_seen < cutoff:
                stale_actor_ids.append(actor_id)

        for actor_id in stale_actor_ids:
            self.actors.pop(actor_id, None)

        self.last_pruned_at = now.isoformat()
        self.last_pruned_count = len(stale_actor_ids)
        return len(stale_actor_ids)

    def snapshot(self) -> Dict[str, Dict[str, Any]]:
        self._prune_stale_actors()
        return self.actors

    def capabilities(self) -> Dict[str, Any]:
        self._prune_stale_actors()
        return {
            "endpoint": self.endpoint,
            "zenoh_available": self.zenoh_available,
            "monitor_active": self.monitor_active,
            "actors_count": len(self.actors),
            "actor_ttl_seconds": self.actor_ttl_seconds,
            "last_pruned_at": self.last_pruned_at,
            "last_pruned_count": self.last_pruned_count,
            "last_error": self.last_error,
        }

    def initialize(self, broadcast: Callable[[Dict[str, Any]], None]) -> None:
        self.endpoint = os.getenv(ZENOH_ENDPOINT_ENV_VAR, DEFAULT_ZENOH_ENDPOINT)
        self.actor_ttl_seconds = self._read_actor_ttl_seconds()
        self.last_error = None
        self.monitor_active = False
        logger.info(
            "[Zenoh] Initializing Monitor Node for endpoint %s...", self.endpoint
        )
        try:
            import zenoh

            self.zenoh_available = True

            z_conf = zenoh.Config()
            z_conf.insert_json5("connect/endpoints", json.dumps([self.endpoint]))
            self.zenoh_session = zenoh.open(z_conf)
            self.monitor_active = True

            def on_obs(sample: Any) -> None:
                try:
                    key_parts = str(sample.key_expr).split("/")
                    if len(key_parts) < 3:
                        return
                    actor_id = key_parts[-2]

                    import zlib

                    raw_bytes = (
                        sample.payload.to_bytes()
                        if hasattr(sample.payload, "to_bytes")
                        else sample.payload
                    )
                    if len(raw_bytes) > 0:
                        header = raw_bytes[0]
                        data_content = raw_bytes[1:]

                        if header == 1:
                            decompressed = zlib.decompress(data_content)
                            payload = json.loads(decompressed.decode("utf-8"))
                        elif header == 0:
                            payload = json.loads(data_content.decode("utf-8"))
                        else:
                            payload = json.loads(raw_bytes.decode("utf-8"))
                    else:
                        payload = {}

                    self.actors[actor_id] = {
                        "id": actor_id,
                        "status": "active",
                        "last_seen": datetime.now().isoformat(),
                        "data": payload,
                    }

                    broadcast(
                        {
                            "type": "distributed_update",
                            "actor_id": actor_id,
                            "data": self.actors[actor_id],
                        }
                    )
                except Exception as e:
                    logger.info(f"[Zenoh] Error processing obs: {e}")

            logger.info("   [OK] Subscribing to ag/*/obs")
            self.zenoh_session.declare_subscriber("ag/*/obs", on_obs)
        except ImportError as e:
            self.zenoh_available = False
            self.last_error = f"Zenoh dependency unavailable: {e}"
            logger.info("[Zenoh] Failed to init: %s", self.last_error)
        except Exception as e:
            self.last_error = str(e)
            logger.info("[Zenoh] Failed to init: %s", e)

    def close(self) -> None:
        if self.zenoh_session:
            self.zenoh_session.close()
            self.zenoh_session = None
        self.monitor_active = False
