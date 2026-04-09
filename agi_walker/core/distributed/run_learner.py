import argparse
import json
import time
import numpy as np
import zenoh
import logging

# Configure Logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] [Learner] %(message)s"
)
logger = logging.getLogger("learner")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--zenoh-prefix", type=str, default="ag")
    args = parser.parse_args()

    # 1. Setup Zenoh
    logger.info("Initializing Zenoh Learner...")
    conf = zenoh.Config()

    import os

    router = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")

    # Connect to Sidecar or Router
    logger.info(f"Connecting to Zenoh at {router}")
    conf.insert_json5("connect/endpoints", f'["{router}"]')
    session = zenoh.open(conf)

    # We want to subscribe to ALL actors
    key_obs_wildcard = f"{args.zenoh_prefix}/*/obs"
    action_publishers = {}

    logger.info(f"Subscribing to: {key_obs_wildcard}")

    def on_obs(sample):
        try:
            # Parse Key to get ID
            # key: ag/actor/1/obs
            key_parts = str(sample.key_expr).split("/")
            actor_id = key_parts[-2]  # "1" or "actor1"

            # Payload Handling (Compression Support)
            import zlib

            raw_bytes = bytes(sample.payload)

            if len(raw_bytes) > 0:
                header = raw_bytes[0]  # First byte is header
                data_content = raw_bytes[1:]

                if header == 1:  # Zlib
                    decompressed = zlib.decompress(data_content)
                    json.loads(decompressed.decode("utf-8"))
                else:  # Raw (0 or legacy JSON char)
                    # Fallback for backward compatibility (if first char is '{' = 0x7B)
                    if header == 0x7B:
                        json.loads(raw_bytes.decode("utf-8"))
                    else:
                        json.loads(data_content.decode("utf-8"))

            print(
                f"✅ [Learner] Received Obs from {actor_id} -> Computing Action",
                flush=True,
            )
            # logger.info(f"Received Obs from {actor_id}: {list(payload.keys())}")

            # 2. Compute Action (Random Policy)
            # In real scenario, this would batch and infer NN
            action = np.random.uniform(-1.0, 1.0, size=(12,)).tolist()

            # 3. Publish Action
            key_act = f"{args.zenoh_prefix}/{actor_id}/act"
            action_payload = {"action": action}
            if key_act not in action_publishers:
                action_publishers[key_act] = session.declare_publisher(key_act)

            # logger.info(f"Sending Action to {actor_id}")
            action_publishers[key_act].put(json.dumps(action_payload).encode("utf-8"))

        except Exception as e:
            logger.error(f"Error processing obs: {e}")

    sub = session.declare_subscriber(key_obs_wildcard, on_obs)

    logger.info("🧠 Learner Running. Waiting for observations...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping...")
        sub.undeclare()
        session.close()


if __name__ == "__main__":
    main()
