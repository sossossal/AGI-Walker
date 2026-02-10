import socket
import struct
import json
import threading
import time

class MockGodotServer:
    def __init__(self, host='127.0.0.1', port=9000):
        self.host = host
        self.port = port
        self.running = True
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)
        print(f"🎭 Mock Server listening on {self.host}:{self.port}")
        
    def start(self):
        while self.running:
            try:
                self.server_sock.settimeout(1.0)
                client, addr = self.server_sock.accept()
                print(f"🎭 Client connected: {addr}")
                self._handle_client(client)
            except socket.timeout:
                continue
            except Exception as e:
                # print(f"Server error: {e}")
                pass
                
    def _handle_client(self, client):
        client.settimeout(None)
        while self.running:
            try:
                # Read Length (4 bytes)
                len_bytes = self._recv_all(client, 4)
                if not len_bytes:
                    break
                    
                length = struct.unpack('<I', len_bytes)[0]
                data = self._recv_all(client, length)
                
                cmd = json.loads(data.decode('utf-8'))
                # print(f"🎭 Received: {cmd}")
                
                # Mock Response
                # obs = {"vector": [qpos, qvel...], "reward": 0.0, "done": False}
                obs = {
                    "vector": [0.1] * 24,
                    "reward": 1.0,
                    "done": False
                }
                
                response_json = json.dumps(obs).encode('utf-8')
                response_len = struct.pack('<I', len(response_json))
                client.sendall(response_len + response_json)
                
            except Exception as e:
                print(f"🎭 Client Handler Error: {e}")
                break
        client.close()
        print("🎭 Client disconnected")

    def _recv_all(self, sock, n):
        data = b''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def stop(self):
        self.running = False
        self.server_sock.close()

if __name__ == "__main__":
    server = MockGodotServer()
    server.start()
