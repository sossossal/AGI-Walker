"""
AGI-Walker 硬件驱动测试 (Mock 模式)
验证 RealRobotDriver 和 SysID 数据采集在 Mock 模式下的正确性。
"""

import unittest
import os
import csv
from agi_walker.core.drivers.real_robot_driver import RealRobotDriver
from agi_walker.core.drivers.collect_sysid_data import collect_data

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))


class TestRealRobotDriver(unittest.TestCase):
    """测试 Mock 模式下的硬件驱动"""

    def test_connect_disconnect(self):
        driver = RealRobotDriver(mock=True)
        self.assertTrue(driver.connect())
        self.assertTrue(driver.running)
        driver.disconnect()
        self.assertFalse(driver.running)

    def test_send_motor_commands(self):
        driver = RealRobotDriver(mock=True)
        driver.connect()

        cmd = {"motor_1": 1.57, "motor_2": -0.5, "motor_3": 0.0}
        self.assertTrue(driver.send_motor_commands(cmd))

        state = driver.get_state()
        self.assertIn("motor_1", state["motors"])
        self.assertAlmostEqual(state["motors"]["motor_1"]["pos"], 1.57)
        self.assertAlmostEqual(state["motors"]["motor_2"]["pos"], -0.5)

        driver.disconnect()

    def test_get_state_structure(self):
        driver = RealRobotDriver(mock=True)
        driver.connect()
        driver.send_motor_commands({"motor_1": 0.5})

        state = driver.get_state()
        self.assertIn("motors", state)
        self.assertIn("imu", state)
        self.assertIn("rpy", state["imu"])
        self.assertIn("acc", state["imu"])
        self.assertIn("gyro", state["imu"])

        driver.disconnect()


class TestSysIDCollection(unittest.TestCase):
    """测试 Mock 模式下的 SysID 数据采集"""

    def test_collect_short_sweep(self):
        out_file = os.path.join(_THIS_DIR, "_test_sysid_output.csv")
        if os.path.exists(out_file):
            os.remove(out_file)

        # 短时间采集
        collect_data("COM3", duration=0.5, out_file=out_file, mock=True)

        self.assertTrue(os.path.exists(out_file))

        with open(out_file, "r") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            self.assertGreater(len(rows), 5, "应至少采集 5 个样本")
            # 检查所有必需列
            expected_cols = [
                "time",
                "target_pos",
                "actual_pos",
                "actual_vel",
                "actual_torque",
            ]
            for col in expected_cols:
                self.assertIn(col, rows[0], f"缺少列: {col}")

        # 清理
        os.remove(out_file)


if __name__ == "__main__":
    unittest.main(verbosity=2)
