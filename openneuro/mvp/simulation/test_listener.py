"""
Simple Zenoh Listener for MVP Verification
"""
import zenoh
import json
import time

def listener(sample):
    payload_bytes = bytes(sample.payload)
    print(f">> [Received] {sample.key_expr}: {payload_bytes.decode('utf-8')}")

if __name__ == "__main__":
    print("Initializing Listener...")
    session = zenoh.open(zenoh.Config())
    
    print("Declaring subscribers...")
    sub1 = session.declare_subscriber("zone/1/state", listener)
    sub2 = session.declare_subscriber("zone/1/imc22/output", listener)
    
    print("Listener ready. Waiting for data... (Press Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
        session.close()
