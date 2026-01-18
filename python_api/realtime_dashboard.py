"""
实时监控仪表板
Real-time Dashboard

功能:
- 实时数据可视化
- 性能曲线绘制
- 系统状态监控
- 告警显示
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import numpy as np
from typing import Dict, List
from collections import deque
import time


class RealtimeDashboard:
    """实时监控仪表板"""
    
    def __init__(self, max_points: int = 500):
        """
        初始化仪表板
        
        参数:
            max_points: 最大显示点数
        """
        self.max_points = max_points
        
        # 数据缓冲
        self.time_data = deque(maxlen=max_points)
        self.energy_data = deque(maxlen=max_points)
        self.temp_data = deque(maxlen=max_points)
        self.velocity_data = deque(maxlen=max_points)
        self.safety_level = deque(maxlen=max_points)
        
        # 当前值
        self.current_battery = 100.0
        self.current_temp = 25.0
        self.current_velocity = 0.0
        self.current_safety = 0
        
        # 告警
        self.warnings = []
        self.max_warnings = 10
        
        # 创建图形
        self.setup_plots()
    
    def setup_plots(self):
        """设置绘图"""
        # 创建主窗口
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('AGI-Walker 实时监控仪表板', fontsize=16, fontweight='bold')
        
        # 使用GridSpec布局
        gs = GridSpec(3, 3, figure=self.fig, hspace=0.3, wspace=0.3)
        
        # 1. 电池电量 (左上)
        self.ax_battery = self.fig.add_subplot(gs[0, 0])
        self.ax_battery.set_title('电池电量 (%)', fontweight='bold')
        self.ax_battery.set_ylim(0, 100)
        self.line_battery, = self.ax_battery.plot([], [], 'g-', linewidth=2)
        
        # 2. 温度 (中上)
        self.ax_temp = self.fig.add_subplot(gs[0, 1])
        self.ax_temp.set_title('系统温度 (°C)', fontweight='bold')
        self.ax_temp.set_ylim(20, 100)
        self.line_temp, = self.ax_temp.plot([], [], 'r-', linewidth=2)
        # 添加警告线
        self.ax_temp.axhline(y=70, color='orange', linestyle='--', label='警告')
        self.ax_temp.axhline(y=85, color='red', linestyle='--', label='危险')
        
        # 3. 速度 (右上)
        self.ax_velocity = self.fig.add_subplot(gs[0, 2])
        self.ax_velocity.set_title('速度 (m/s)', fontweight='bold')
        self.ax_velocity.set_ylim(0, 2.5)
        self.line_velocity, = self.ax_velocity.plot([], [], 'b-', linewidth=2)
        
        # 4. 电池指示器 (左中) - 大显示
        self.ax_battery_gauge = self.fig.add_subplot(gs[1, 0])
        self.ax_battery_gauge.set_title('电池状态', fontweight='bold')
        self.ax_battery_gauge.set_xlim(0, 1)
        self.ax_battery_gauge.set_ylim(0, 1)
        self.ax_battery_gauge.axis('off')
        self.battery_text = self.ax_battery_gauge.text(
            0.5, 0.5, '100%', 
            ha='center', va='center', 
            fontsize=40, fontweight='bold'
        )
        
        # 5. 温度指示器 (中中)
        self.ax_temp_gauge = self.fig.add_subplot(gs[1, 1])
        self.ax_temp_gauge.set_title('温度状态', fontweight='bold')
        self.ax_temp_gauge.set_xlim(0, 1)
        self.ax_temp_gauge.set_ylim(0, 1)
        self.ax_temp_gauge.axis('off')
        self.temp_text = self.ax_temp_gauge.text(
            0.5, 0.5, '25°C', 
            ha='center', va='center', 
            fontsize=35, fontweight='bold'
        )
        
        # 6. 安全等级 (右中)
        self.ax_safety = self.fig.add_subplot(gs[1, 2])
        self.ax_safety.set_title('安全等级', fontweight='bold')
        self.ax_safety.set_xlim(0, 1)
        self.ax_safety.set_ylim(0, 1)
        self.ax_safety.axis('off')
        self.safety_text = self.ax_safety.text(
            0.5, 0.5, 'SAFE', 
            ha='center', va='center', 
            fontsize=30, fontweight='bold',
            color='green'
        )
        
        # 7. 告警日志 (底部全宽)
        self.ax_warnings = self.fig.add_subplot(gs[2, :])
        self.ax_warnings.set_title('系统告警', fontweight='bold')
        self.ax_warnings.set_xlim(0, 1)
        self.ax_warnings.set_ylim(0, 1)
        self.ax_warnings.axis('off')
        self.warning_text = self.ax_warnings.text(
            0.05, 0.9, '', 
            ha='left', va='top', 
            fontsize=10, family='monospace'
        )
    
    def update_data(self, timestamp: float, energy: float, temp: float, 
                   velocity: float, safety: int):
        """
        更新数据
        
        参数:
            timestamp: 时间戳
            energy: 电池电量 (%)
            temp: 温度 (°C)
            velocity: 速度 (m/s)
            safety: 安全等级 (0=SAFE, 1=WARNING, 2=DANGER, 3=EMERGENCY)
        """
        self.time_data.append(timestamp)
        self.energy_data.append(energy)
        self.temp_data.append(temp)
        self.velocity_data.append(velocity)
        self.safety_level.append(safety)
        
        self.current_battery = energy
        self.current_temp = temp
        self.current_velocity = velocity
        self.current_safety = safety
        
        # 检查告警
        if energy < 20:
            self.add_warning(f"[{timestamp:.1f}s] 电池电量低: {energy:.1f}%")
        if temp > 70:
            self.add_warning(f"[{timestamp:.1f}s] 温度警告: {temp:.1f}°C")
        if temp > 85:
            self.add_warning(f"[{timestamp:.1f}s] 温度危险: {temp:.1f}°C")
        if safety >= 2:
            levels = ['SAFE', 'WARNING', 'DANGER', 'EMERGENCY']
            self.add_warning(f"[{timestamp:.1f}s] 安全等级: {levels[safety]}")
    
    def add_warning(self, message: str):
        """添加告警"""
        if message not in self.warnings:  # 避免重复
            self.warnings.append(message)
            if len(self.warnings) > self.max_warnings:
                self.warnings.pop(0)
    
    def update_plot(self, frame):
        """更新绘图 (用于动画)"""
        if len(self.time_data) == 0:
            return
        
        time_array = np.array(self.time_data)
        
        # 更新曲线
        self.line_battery.set_data(time_array, np.array(self.energy_data))
        self.ax_battery.relim()
        self.ax_battery.autoscale_view()
        
        self.line_temp.set_data(time_array, np.array(self.temp_data))
        self.ax_temp.relim()
        self.ax_temp.autoscale_view()
        
        self.line_velocity.set_data(time_array, np.array(self.velocity_data))
        self.ax_velocity.relim()
        self.ax_velocity.autoscale_view()
        
        # 更新大数字显示
        # 电池颜色
        if self.current_battery > 50:
            battery_color = 'green'
        elif self.current_battery > 20:
            battery_color = 'orange'
        else:
            battery_color = 'red'
        
        self.battery_text.set_text(f'{self.current_battery:.0f}%')
        self.battery_text.set_color(battery_color)
        
        # 温度颜色
        if self.current_temp < 70:
            temp_color = 'green'
        elif self.current_temp < 85:
            temp_color = 'orange'
        else:
            temp_color = 'red'
        
        self.temp_text.set_text(f'{self.current_temp:.0f}°C')
        self.temp_text.set_color(temp_color)
        
        # 安全等级
        safety_levels = [
            ('SAFE', 'green'),
            ('WARNING', 'orange'),
            ('DANGER', 'red'),
            ('EMERGENCY', 'darkred')
        ]
        level_text, level_color = safety_levels[min(self.current_safety, 3)]
        self.safety_text.set_text(level_text)
        self.safety_text.set_color(level_color)
        
        # 更新告警
        warning_display = '\n'.join(self.warnings[-10:])
        self.warning_text.set_text(warning_display)
        
        return (self.line_battery, self.line_temp, self.line_velocity,
                self.battery_text, self.temp_text, self.safety_text,
                self.warning_text)
    
    def show(self, interval: int = 100):
        """
        显示仪表板 (阻塞)
        
        参数:
            interval: 更新间隔 (ms)
        """
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, 
            interval=interval, blit=True
        )
        plt.show()
    
    def save_snapshot(self, filename: str):
        """保存当前快照"""
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"仪表板快照已保存: {filename}")


class SimpleTextDashboard:
    """简化的文本仪表板 (无GUI)"""
    
    def __init__(self):
        self.start_time = time.time()
        self.update_count = 0
    
    def update(self, energy: float, temp: float, velocity: float, safety: int):
        """更新显示"""
        self.update_count += 1
        elapsed = time.time() - self.start_time
        
        # 清屏 (简化版)
        if self.update_count % 10 == 0:
            print("\n" + "="*70)
            print(f"时间: {elapsed:.1f}s | 更新次数: {self.update_count}")
            print("-"*70)
            
            # 电池
            battery_bar = '█' * int(energy/5) + '░' * (20 - int(energy/5))
            print(f"电池: [{battery_bar}] {energy:.1f}%")
            
            # 温度
            temp_status = "正常" if temp < 70 else ("警告" if temp < 85 else "危险")
            print(f"温度: {temp:.1f}°C ({temp_status})")
            
            # 速度
            print(f"速度: {velocity:.2f} m/s")
            
            # 安全
            safety_names = ['安全', '警告', '危险', '紧急']
            print(f"安全: {safety_names[min(safety, 3)]}")


if __name__ == "__main__":
    print("实时监控仪表板加载完成")
    print("\n模式:")
    print("  1. 图形界面 (需要matplotlib)")
    print("  2. 文本界面")
    
    # 使用简化文本版本示例
    dashboard = SimpleTextDashboard()
    
    print("\n运行5秒模拟...")
    for i in range(50):
        # 模拟数据
        energy = max(100 - i*2, 0)
        temp = 25 + i * 1.2
        velocity = 1.0 + np.random.uniform(-0.2, 0.2)
        safety = 0 if temp < 70 else (1 if temp < 85 else 2)
        
        dashboard.update(energy, temp, velocity, safety)
        time.sleep(0.1)
    
    print("\n" + "="*70)
    print("演示完成")
