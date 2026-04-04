"""
异步TCP客户端
使用asyncio实现高性能异步通信
"""

import asyncio
import json
import time
from typing import Dict, Optional, Callable
from dataclasses import dataclass

import logging

logger = logging.getLogger(__name__)


@dataclass
class AsyncClientConfig:
    """异步客户端配置"""

    host: str = "127.0.0.1"
    port: int = 9999
    connect_timeout: float = 5.0
    read_buffer_size: int = 4096
    queue_max_size: int = 100
    reconnect_delay: float = 1.0
    max_reconnect_attempts: int = 5


class AsyncGodotClient:
    """
    异步Godot TCP客户端

    特点：
    - 非阻塞异步IO
    - 自动重连
    - 协程安全
    """

    def __init__(self, config: Optional[AsyncClientConfig] = None) -> None:
        self.config = config or AsyncClientConfig()

        # 连接状态
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.connected = False
        self.running = False

        # 数据队列
        self.sensor_queue: asyncio.Queue = asyncio.Queue(
            maxsize=self.config.queue_max_size
        )

        # 缓冲区
        self.buffer = ""

        # 统计
        self.packets_received = 0
        self.packets_sent = 0
        self.reconnect_count = 0
        self.last_receive_time = 0.0

        # 回调
        self.on_connect: Optional[Callable] = None
        self.on_disconnect: Optional[Callable] = None
        self.on_sensor_data: Optional[Callable] = None

    async def connect(self) -> bool:
        """异步连接到Godot服务器"""
        try:
            self.reader, self.writer = await asyncio.wait_for(
                asyncio.open_connection(self.config.host, self.config.port),
                timeout=self.config.connect_timeout,
            )

            self.connected = True
            self.running = True

            logger.info(
                f"✅ 异步连接到Godot仿真器 {self.config.host}:{self.config.port}"
            )

            if self.on_connect:
                await self._call_callback(self.on_connect)

            return True

        except asyncio.TimeoutError:
            logger.info(f"❌ 连接超时: {self.config.host}:{self.config.port}")
            return False
        except ConnectionRefusedError:
            logger.info("❌ 连接被拒绝，请确保Godot仿真器正在运行")
            return False
        except Exception as e:
            logger.info(f"❌ 连接错误: {e}")
            return False

    async def disconnect(self) -> None:
        """断开连接"""
        self.running = False
        self.connected = False

        if self.writer:
            self.writer.close()
            try:
                await self.writer.wait_closed()
            except Exception:
                pass

        self.reader = None
        self.writer = None

        if self.on_disconnect:
            await self._call_callback(self.on_disconnect)

        logger.info("🔌 异步连接已断开")

    async def reconnect(self) -> bool:
        """重连"""
        for attempt in range(self.config.max_reconnect_attempts):
            logger.info(
                f"🔄 重连尝试 {attempt + 1}/{self.config.max_reconnect_attempts}"
            )

            await self.disconnect()
            await asyncio.sleep(self.config.reconnect_delay)

            if await self.connect():
                self.reconnect_count += 1
                return True

        logger.info("❌ 重连失败")
        return False

    async def receive_loop(self) -> None:
        """接收循环"""
        while self.running and self.reader:
            try:
                data = await self.reader.read(self.config.read_buffer_size)

                if not data:
                    logger.info("⚠️ 服务器关闭连接")
                    self.connected = False
                    break

                self.buffer += data.decode("utf-8")

                # 处理完整的JSON行
                while "\n" in self.buffer:
                    line, self.buffer = self.buffer.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue

                    try:
                        sensor_data = json.loads(line)
                        self.packets_received += 1
                        self.last_receive_time = time.time()

                        # 放入队列
                        try:
                            self.sensor_queue.put_nowait(sensor_data)
                        except asyncio.QueueFull:
                            # 队列满时丢弃最旧的
                            self.sensor_queue.get_nowait()
                            self.sensor_queue.put_nowait(sensor_data)

                        # 触发回调
                        if self.on_sensor_data:
                            await self._call_callback(self.on_sensor_data, sensor_data)

                    except json.JSONDecodeError as e:
                        logger.info(f"⚠️ JSON解析错误: {e}")

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.info(f"❌ 接收错误: {e}")
                self.connected = False
                break

    async def receive_sensors(self, timeout: float = 1.0) -> Optional[Dict]:
        """
        异步接收传感器数据

        Args:
            timeout: 超时时间

        Returns:
            传感器数据或None
        """
        try:
            return await asyncio.wait_for(self.sensor_queue.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None

    def get_latest_sensors(self) -> Optional[Dict]:
        """获取最新传感器数据（非阻塞）"""
        try:
            return self.sensor_queue.get_nowait()
        except asyncio.QueueEmpty:
            return None

    async def send_commands(self, commands: Dict) -> bool:
        """
        异步发送控制指令

        Args:
            commands: 控制指令字典

        Returns:
            是否发送成功
        """
        if not self.connected or not self.writer:
            return False

        try:
            msg = json.dumps(commands) + "\n"
            self.writer.write(msg.encode("utf-8"))
            await self.writer.drain()
            self.packets_sent += 1
            return True

        except Exception as e:
            logger.info(f"❌ 发送错误: {e}")
            self.connected = False
            return False

    async def _call_callback(self, callback: Callable, *args) -> None:
        """安全调用回调"""
        try:
            if asyncio.iscoroutinefunction(callback):
                await callback(*args)
            else:
                callback(*args)
        except Exception as e:
            logger.info(f"⚠️ 回调错误: {e}")

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "connected": self.connected,
            "packets_received": self.packets_received,
            "packets_sent": self.packets_sent,
            "queue_size": self.sensor_queue.qsize(),
            "reconnect_count": self.reconnect_count,
            "last_receive_time": self.last_receive_time,
        }


class AsyncControllerBase:
    """
    异步控制器基类

    提供多协程运行框架
    """

    def __init__(self, client: Optional[AsyncGodotClient] = None) -> None:
        self.client = client or AsyncGodotClient()
        self.running = False
        self.tasks: list = []

    async def start(self) -> None:
        """启动控制器"""
        if not await self.client.connect():
            return False

        self.running = True
        return True

    async def stop(self) -> None:
        """停止控制器"""
        self.running = False

        # 取消所有任务
        for task in self.tasks:
            task.cancel()

        await self.client.disconnect()

    async def run(self, duration: float = 120.0) -> None:
        """运行控制循环"""
        if not await self.start():
            return

        try:
            # 创建并发任务
            self.tasks = [
                asyncio.create_task(self._receive_loop()),
                asyncio.create_task(self._control_loop()),
                asyncio.create_task(self._monitor_loop()),
            ]

            # 等待指定时间或直到停止
            await asyncio.sleep(duration)

        except asyncio.CancelledError:
            pass
        finally:
            await self.stop()

    async def _receive_loop(self) -> None:
        """接收协程"""
        await self.client.receive_loop()

    async def _control_loop(self) -> None:
        """控制协程（子类实现）"""
        while self.running:
            await asyncio.sleep(0.033)  # 30Hz

    async def _monitor_loop(self) -> None:
        """监控协程"""
        while self.running:
            self.client.get_stats()
            # 可以添加监控逻辑
            await asyncio.sleep(1.0)


# 测试代码
async def test_async_client() -> None:
    """测试异步客户端"""
    logger.info("异步TCP客户端测试\n")

    client = AsyncGodotClient()

    logger.info("尝试连接到Godot...")
    connected = await client.connect()

    if not connected:
        logger.info("\n⚠️ 无法连接到Godot，请确保仿真器正在运行")
        return

    logger.info("\n开始接收数据...")

    # 启动接收任务
    receive_task = asyncio.create_task(client.receive_loop())

    try:
        for i in range(30):  # 3秒
            sensor_data = await client.receive_sensors(timeout=0.5)

            if sensor_data:
                orient = (
                    sensor_data.get("sensors", {})
                    .get("imu", {})
                    .get("orient", [0, 0, 0])
                )
                height = sensor_data.get("torso_height", 0)
                logger.info(
                    f"[{i}] Roll: {orient[0]:.1f}° Pitch: {orient[1]:.1f}° Height: {height:.2f}m"
                )

                # 发送测试指令
                await client.send_commands(
                    {"motors": {"hip_left": 5.0, "hip_right": -5.0}}
                )

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("\n用户中断")
    finally:
        receive_task.cancel()
        await client.disconnect()

    logger.info("\n统计:")
    logger.info(json.dumps(client.get_stats(), indent=2))


if __name__ == "__main__":
    asyncio.run(test_async_client())
