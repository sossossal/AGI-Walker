import argparse
import json
import os
import socket
import struct
import time
import zenoh
import logging

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] [Sidecar] %(message)s'
)
logger = logging.getLogger("sidecar")

class GodotClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                logger.info(f"✅ Connected to Godot at {self.host}:{self.port}")
                return
            except ConnectionRefusedError:
                logger.warning(f"Connection refused at {self.host}:{self.port}, retrying in 2s...")
                time.sleep(2)
            except Exception as e:
                logger.error(f"Connection failed: {e}")
                time.sleep(2)

    def send_action(self, action_dict):
        if not self.sock: return
        try:
            data = json.dumps(action_dict).encode('utf-8')
            length = struct.pack('<I', len(data))
            self.sock.sendall(length + data)
        except Exception as e:
            logger.error(f"Send failed: {e}")
            self.reconnect()

    def receive_obs(self):
        if not self.sock: return None
        try:
            # Read Length
            print("DEBUG: Waiting for 4 bytes length...", flush=True)
            len_bytes = self._recv_all(4)
            if not len_bytes: raise ConnectionError("Closed")
            length = struct.unpack('<I', len_bytes)[0]
            print(f"DEBUG: Got length {length}, waiting for payload...", flush=True)
            
            # Read Payload
            data = self._recv_all(length)
            if not data: raise ConnectionError("Incomplete")
            
            print("DEBUG: Got payload, parsing...", flush=True)
            return json.loads(data.decode('utf-8'))
        except Exception as e:
            logger.error(f"Receive failed: {e}")
            self.reconnect()
            return None

    def _recv_all(self, n):
        data = b''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet: return None
            data += packet
        return data

    def reconnect(self):
        if self.sock: self.sock.close()
        self.connect()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=str, default="actor1")
    parser.add_argument("--godot-host", type=str, default="127.0.0.1", help="Hostname/IP of Godot instance")
    parser.add_argument("--godot-port", type=int, default=9000)
    parser.add_argument("--zenoh-prefix", type=str, default="ag")
    parser.add_argument(
        "--zenoh-router",
        type=str,
        default=os.environ.get("ZENOH_ROUTER"),
        help="Zenoh Router endpoint (e.g. tcp/zenoh-router:7447)",
    )
    args = parser.parse_args()

    # 1. Setup Zenoh
    logger.info("Initializing Zenoh...")
    conf = zenoh.Config()

    if args.zenoh_router:
        logger.info(f"Connecting to Zenoh Router at {args.zenoh_router}")
        conf.insert_json5("connect/endpoints", f'["{args.zenoh_router}"]')
    else:
        # Default: Listen on local TCP (Standalone Mode)
        conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
        
    session = zenoh.open(conf)
    
    key_obs = f"{args.zenoh_prefix}/{args.id}/obs"
    key_act = f"{args.zenoh_prefix}/{args.id}/act"
    pub_obs = session.declare_publisher(key_obs)
    
    logger.info(f"Pub Obs: {key_obs}")
    logger.info(f"Sub Act: {key_act}")

    # 2. Setup Godot
    godot = GodotClient(args.godot_host, args.godot_port)
    godot.connect()

    # 3. Action Listener
    # We use a queue or just update a shared state for the main loop to pick up?
    # Or validly, Zenoh callback can push directly to TCP?
    # Sending to TCP is IO bound, maybe safe in callback if non-blocking or protected.
    # For simplicity, let's use a thread-safe approach or just let callback send.
    
    def on_action(sample):
        try:
            # Zenoh 1.0 payload is ZBytes
            payload_bytes = bytes(sample.payload)
            payload = json.loads(payload_bytes.decode('utf-8'))
            print(f"✅ [Sidecar] Received Action -> Forwarding to Godot", flush=True)
            # Note: Godot expects {"type": "step", "action": [...]}
            # We assume Learner sends exactly that or just the action array.
            # Let's standardize: Learner sends {"action": [...]}, we wrap it.
            
            cmd = {"type": "step", "action": payload.get("action", [])}
            godot.send_action(cmd)
            
        except Exception as e:
            logger.error(f"Error processing action: {e}")

    sub = session.declare_subscriber(key_act, on_action)

    # 4. Observation Loop
    # We trigger the loop by pulling from Godot
    # But Godot waits for "step" command before sending next obs?
    # Initial state: Godot waits. We need to send "reset" first?
    # Yes, usually Env.reset() sends "reset".
    # Here, who initiates? 
    # Let's say Learner initiates "reset" over Zenoh?
    # Or Sidecar auto-resets on start.
    
    logger.info("Sending initial RESET to Godot...")
    godot.send_action({"type": "reset"})
    
    try:
        while True:
            # 1. Read Obs from Godot (Blocking until arrived)
            obs = godot.receive_obs()
            if obs:
                # 2. Publish to Zenoh
                import zlib
                
                payload_str = json.dumps(obs)
                payload_bytes = payload_str.encode('utf-8')
                
                # Compression Strategy: Zlib if > 1KB
                # Header: 0x00 = Raw, 0x01 = Zlib
                if len(payload_bytes) > 1024:
                    compressed = zlib.compress(payload_bytes)
                    final_payload = b'\x01' + compressed
                    # logger.info(f"Compressed {len(payload_bytes)} -> {len(final_payload)} bytes")
                else:
                    final_payload = b'\x00' + payload_bytes
                
                pub_obs.put(final_payload)
            
            # 3. Wait/Yield?
            # Godot is now waiting for next Step.
            # We just wait for Zenoh callback to fire on_action.
            # So this loop is actually driven by the arrival of Obs.
            # Once Obs is published, we blocked on receive_obs() line above.
            # But receive_obs() won't return until Godot sends something.
            # Godot only sends something AFTER we send an action.
            # So we are deadlocked if we don't handle the first step?
            
            # Flow:
            # RESET -> Godot sends Obs0.
            # Sidecar reads Obs0 -> Publishes Obs0.
            # Learner receives Obs0 -> Computes Act0 -> Publishes Act0.
            # Sidecar receives Act0 -> Sends Step(Act0) to Godot.
            # Godot advances -> Sends Obs1.
            # Sidecar reads Obs1 -> ... loop continues.
            
            # So the while loop is correct. receive_obs() will block until cycle completes.
            pass
            
    except KeyboardInterrupt:
        logger.info("Stopping...")
        sub.undeclare()
        session.close()

if __name__ == "__main__":
    main()
