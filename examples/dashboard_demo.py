import sys
import socket
import struct
import numpy as np
import cv2
import threading
import time

# 尝试导入 PyQt6，如果失败则回退到 OpenCV 窗口
try:
    from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
    from PyQt6.QtGui import QImage, QPixmap
    from PyQt6.QtCore import pyqtSignal, QThread, Qt
    HAS_QT = True
except ImportError:
    HAS_QT = False
    print("⚠️ 未安装 PyQt6，将使用 OpenCV 窗口显示")
    print("建议安装: pip install PyQt6")

HOST = '127.0.0.1'
PORT = 9998

class VideoReceiver:
    def __init__(self):
        self.running = False
        self.sock = None
        self.latest_frame = None
        self.lock = threading.Lock()

    def connect(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((HOST, PORT))
                self.running = True
                print(f"✅ 已连接到视频流: {HOST}:{PORT}")
                return
            except ConnectionRefusedError:
                print(f"⏳ 等待 Godot 视频服务器启动... ({HOST}:{PORT})")
                time.sleep(2)

    def start_loop(self, callback=None):
        self.connect()
        while self.running:
            try:
                # 1. 读取 4字节长度
                header_data = self._recv_exact(4)
                if not header_data:
                    break
                
                size = struct.unpack('<I', header_data)[0]
                
                # 2. 读取 JPEG 数据
                jpg_data = self._recv_exact(size)
                if not jpg_data:
                    break
                
                # 3. 解码
                frame = cv2.imdecode(np.frombuffer(jpg_data, np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    with self.lock:
                        self.latest_frame = frame
                    
                    if callback:
                        callback(frame)
                        
            except Exception as e:
                print(f"❌ 连接错误: {e}")
                self.sock.close()
                self.connect() # 尝试重连

    def _recv_exact(self, n):
        data = b''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

# ================= PyQt GUI =================

if HAS_QT:
    class VideoThread(QThread):
        frame_received = pyqtSignal(np.ndarray)

        def run(self):
            receiver = VideoReceiver()
            receiver.start_loop(lambda frame: self.frame_received.emit(frame))

    class DashboardWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("AGI-Walker Remote Dashboard")
            self.resize(800, 600)
            
            # UI
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            layout = QVBoxLayout(central_widget)
            
            self.label_status = QLabel("等待视频流...")
            layout.addWidget(self.label_status)
            
            self.label_video = QLabel()
            self.label_video.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.label_video.setStyleSheet("background-color: #222; border: 2px solid #555;")
            self.label_video.setMinimumSize(640, 360)
            layout.addWidget(self.label_video)
            
            # Start Thread
            self.thread = VideoThread()
            self.thread.frame_received.connect(self.update_image)
            self.thread.start()

        def update_image(self, cv_img):
            # 将 CV2 (BGR) 转为 Qt (RGB)
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # 显示
            self.label_video.setPixmap(QPixmap.fromImage(qt_img))
            self.label_status.setText(f"Connected | Resolution: {w}x{h}")

    def run_qt_app():
        app = QApplication(sys.argv)
        window = DashboardWindow()
        window.show()
        sys.exit(app.exec())

# ================= OpenCV Fallback =================

def run_cv_app():
    receiver = VideoReceiver()
    receiver.connect()
    
    print("按 'q' 退出 OpenCV 窗口")
    
    while True:
        # 手动轮询接收 (简单实现)
        # 注意：这里实际上应该在线程里跑，为了简单起见，我们假设 VideoReceiver 的 loop 稍微改一下
        # 但为了复用 VideoReceiver，我们还是开个线程吧
        pass
        
    # 由于 VideoReceiver 是阻塞 loop，我们直接用 callback
    def show_frame(frame):
        cv2.imshow("AGI-Walker Remote Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit(0)
            
    receiver.start_loop(show_frame)

# ================= Main =================

if __name__ == "__main__":
    if HAS_QT:
        run_qt_app()
    else:
        run_cv_app()
