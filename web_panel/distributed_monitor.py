import json
import logging
from datetime import datetime
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger(__name__)


class DistributedMonitor:
    def __init__(self) -> None:
        self.zenoh_session: Optional[Any] = None
        self.actors: Dict[str, Dict[str, Any]] = {}

    def snapshot(self) -> Dict[str, Dict[str, Any]]:
        return self.actors

    def initialize(self, broadcast: Callable[[Dict[str, Any]], None]) -> None:
        logger.info("[Zenoh] Initializing Monitor Node...")
        try:
            import zenoh

            z_conf = zenoh.Config()
            z_conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
            self.zenoh_session = zenoh.open(z_conf)

            def on_obs(sample: Any) -> None:
                try:
                    key_parts = str(sample.key_expr).split("/")
                    if len(key_parts) < 3:
                        return
                    actor_id = key_parts[-2]

                    import zlib

                    raw_bytes = sample.payload.to_bytes() if hasattr(sample.payload, "to_bytes") else sample.payload
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
        except Exception as e:
            logger.info(f"[Zenoh] Failed to init: {e}")

    def close(self) -> None:
        if self.zenoh_session:
            self.zenoh_session.close()
            self.zenoh_session = None
