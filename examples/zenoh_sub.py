import zenoh, time

conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/127.0.0.1:7447"]')
print("Sub opening session...")
z = zenoh.open(conf)
print("Declared Sub")
sub = z.declare_subscriber(
    "demo/key", lambda s: print(f"RECEIVED: {s.payload.decode()}", flush=True)
)
while True:
    time.sleep(1)
