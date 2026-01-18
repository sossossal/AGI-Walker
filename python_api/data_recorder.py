"""
数据记录系统
Data Recording System

功能:
- 完整数据记录
- 多格式导出 (CSV, JSON, HDF5)
- 数据回放
- 性能分析
"""

import json
import csv
import pickle
import numpy as np
from typing import Dict, List, Any, Optional
from pathlib import Path
from datetime import datetime


class DataRecorder:
    """数据记录器"""
    
    def __init__(self, session_name: Optional[str] = None):
        """
        初始化数据记录器
        
        参数:
            session_name: 会话名称 (默认使用时间戳)
        """
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.session_name = session_name
        self.start_time = datetime.now()
        
        # 数据存储
        self.data = {
            'metadata': {
                'session_name': session_name,
                'start_time': self.start_time.isoformat(),
                'version': '2.5'
            },
            'timeline': [],
            'energy': [],
            'thermal': [],
            'safety': [],
            'sensors': [],
            'control': [],
            'events': []
        }
        
        # 统计
        self.record_count = 0
        self.last_timestamp = 0
    
    def record_state(self, timestamp: float, state: Dict):
        """
        记录完整状态
        
        参数:
            timestamp: 时间戳
            state: 状态字典
        """
        self.data['timeline'].append({
            'timestamp': timestamp,
            'state': state
        })
        
        self.record_count += 1
        self.last_timestamp = timestamp
    
    def record_energy(self, timestamp: float, battery_soc: float, 
                     power_w: float, current_a: float):
        """记录能量数据"""
        self.data['energy'].append({
            'timestamp': timestamp,
            'battery_soc': battery_soc,
            'power_w': power_w,
            'current_a': current_a
        })
    
    def record_thermal(self, timestamp: float, temperatures: Dict):
        """记录热数据"""
        self.data['thermal'].append({
            'timestamp': timestamp,
            'temperatures': temperatures
        })
    
    def record_safety(self, timestamp: float, level: int, violations: List[str]):
        """记录安全事件"""
        self.data['safety'].append({
            'timestamp': timestamp,
            'level': level,
            'violations': violations
        })
    
    def record_sensor(self, timestamp: float, sensor_name: str, value: Any):
        """记录传感器数据"""
        self.data['sensors'].append({
            'timestamp': timestamp,
            'sensor': sensor_name,
            'value': value
        })
    
    def record_control(self, timestamp: float, action: Any, params: Dict):
        """记录控制命令"""
        self.data['control'].append({
            'timestamp': timestamp,
            'action': action,
            'params': params
        })
    
    def record_event(self, timestamp: float, event_type: str, description: str):
        """记录事件"""
        self.data['events'].append({
            'timestamp': timestamp,
            'type': event_type,
            'description': description
        })
    
    def save_json(self, filepath: str):
        """保存为JSON"""
        # 添加结束信息
        self.data['metadata']['end_time'] = datetime.now().isoformat()
        self.data['metadata']['duration_s'] = self.last_timestamp
        self.data['metadata']['record_count'] = self.record_count
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)
        
        print(f"数据已保存为JSON: {filepath}")
    
    def save_csv(self, output_dir: str):
        """保存为CSV (多个文件)"""
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        
        # 保存能量数据
        if self.data['energy']:
            filepath = Path(output_dir) / f'{self.session_name}_energy.csv'
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=['timestamp', 'battery_soc', 'power_w', 'current_a'])
                writer.writeheader()
                writer.writerows(self.data['energy'])
            print(f"能量数据已保存: {filepath}")
        
        # 保存安全数据
        if self.data['safety']:
            filepath = Path(output_dir) / f'{self.session_name}_safety.csv'
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'level', 'violations'])
                for record in self.data['safety']:
                    writer.writerow([
                        record['timestamp'],
                        record['level'],
                        '; '.join(record['violations'])
                    ])
            print(f"安全数据已保存: {filepath}")
    
    def save_pickle(self, filepath: str):
        """保存为Pickle (完整Python对象)"""
        with open(filepath, 'wb') as f:
            pickle.dump(self.data, f)
        
        print(f"数据已保存为Pickle: {filepath}")
    
    def get_summary(self) -> str:
        """生成数据摘要"""
        summary = []
        summary.append("="*70)
        summary.append("数据记录摘要")
        summary.append("="*70)
        
        summary.append(f"\n会话信息:")
        summary.append(f"  名称: {self.session_name}")
        summary.append(f"  开始时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        summary.append(f"  持续时间: {self.last_timestamp:.1f}s")
        summary.append(f"  总记录数: {self.record_count}")
        
        summary.append(f"\n数据统计:")
        summary.append(f"  状态记录: {len(self.data['timeline'])}")
        summary.append(f"  能量记录: {len(self.data['energy'])}")
        summary.append(f"  热记录: {len(self.data['thermal'])}")
        summary.append(f"  安全记录: {len(self.data['safety'])}")
        summary.append(f"  传感器记录: {len(self.data['sensors'])}")
        summary.append(f"  控制记录: {len(self.data['control'])}")
        summary.append(f"  事件记录: {len(self.data['events'])}")
        
        # 能量统计
        if self.data['energy']:
            socs = [r['battery_soc'] for r in self.data['energy']]
            powers = [r['power_w'] for r in self.data['energy']]
            summary.append(f"\n能量分析:")
            summary.append(f"  初始电量: {socs[0]:.1f}%")
            summary.append(f"  最终电量: {socs[-1]:.1f}%")
            summary.append(f"  平均功耗: {np.mean(powers):.1f}W")
            summary.append(f"  峰值功耗: {max(powers):.1f}W")
        
        # 安全统计
        if self.data['safety']:
            levels = [r['level'] for r in self.data['safety']]
            summary.append(f"\n安全分析:")
            summary.append(f"  最高等级: {max(levels)}")
            summary.append(f"  告警次数: {sum(1 for l in levels if l > 0)}")
        
        return "\n".join(summary)


class DataPlayer:
    """数据回放器"""
    
    def __init__(self, filepath: str):
        """
        加载记录数据
        
        参数:
            filepath: 数据文件路径
        """
        if filepath.endswith('.json'):
            with open(filepath, 'r', encoding='utf-8') as f:
                self.data = json.load(f)
        elif filepath.endswith('.pkl') or filepath.endswith('.pickle'):
            with open(filepath, 'rb') as f:
                self.data = pickle.load(f)
        else:
            raise ValueError("不支持的文件格式")
        
        self.current_index = 0
        self.timeline = self.data['timeline']
    
    def get_metadata(self) -> Dict:
        """获取元数据"""
        return self.data['metadata']
    
    def get_state_at(self, timestamp: float) -> Optional[Dict]:
        """获取指定时间的状态"""
        for record in self.timeline:
            if abs(record['timestamp'] - timestamp) < 0.01:
                return record['state']
        return None
    
    def play(self, callback, speed: float = 1.0):
        """
        回放数据
        
        参数:
            callback: 回调函数 callback(timestamp, state)
            speed: 播放速度倍数
        """
        import time
        
        if not self.timeline:
            return
        
        start_time = time.time()
        last_timestamp = 0
        
        for record in self.timeline:
            timestamp = record['timestamp']
            state = record['state']
            
            # 等待到正确时间
            if last_timestamp > 0:
                wait_time = (timestamp - last_timestamp) / speed
                time.sleep(max(0, wait_time))
            
            # 调用回调
            callback(timestamp, state)
            
            last_timestamp = timestamp
    
    def analyze(self) -> Dict:
        """分析数据"""
        analysis = {
            'duration': self.data['metadata'].get('duration_s', 0),
            'record_count': len(self.timeline)
        }
        
        # 能量分析
        if self.data['energy']:
            socs = [r['battery_soc'] for r in self.data['energy']]
            powers = [r['power_w'] for r in self.data['energy']]
            
            analysis['energy'] = {
                'initial_soc': socs[0],
                'final_soc': socs[-1],
                'consumed_pct': socs[0] - socs[-1],
                'avg_power': np.mean(powers),
                'max_power': max(powers),
                'total_energy_wh': np.mean(powers) * analysis['duration'] / 3600
            }
        
        return analysis


if __name__ == "__main__":
    print("数据记录系统加载完成")
    
    # 示例
    recorder = DataRecorder("test_session")
    
    # 记录一些数据
    for i in range(100):
        timestamp = i * 0.1
        
        # 状态
        recorder.record_state(timestamp, {
            'position': [i*0.01, 0, 0],
            'velocity': [0.1, 0, 0]
        })
        
        # 能量
        recorder.record_energy(timestamp, 100-i*0.5, 200, 5)
        
        # 安全
        if i % 20 == 0:
            recorder.record_safety(timestamp, 0, [])
        
        # 事件
        if i == 50:
            recorder.record_event(timestamp, 'INFO', '达到中点')
    
    # 保存
    recorder.save_json('test_session.json')
    
    # 显示摘要
    print("\n" + recorder.get_summary())
    
    # 回放示例
    print("\n" + "="*70)
    print("数据回放测试")
    print("="*70)
    
    player = DataPlayer('test_session.json')
    metadata = player.get_metadata()
    print(f"会话: {metadata['session_name']}")
    print(f"持续时间: {metadata.get('duration_s', 0):.1f}s")
    
    analysis = player.analyze()
    print(f"\n分析结果:")
    print(f"  记录数: {analysis['record_count']}")
    if 'energy' in analysis:
        print(f"  能量消耗: {analysis['energy']['consumed_pct']:.1f}%")
        print(f"  平均功耗: {analysis['energy']['avg_power']:.1f}W")
