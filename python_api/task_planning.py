"""
任务规划系统
Task Planning System

功能:
- 路径规划 (A*)
- 避障算法
- 任务调度
- 轨迹优化
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import heapq


@dataclass
class Point:
    """二维点"""
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        """计算到另一点的距离"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __hash__(self):
        return hash((round(self.x, 2), round(self.y, 2)))
    
    def __eq__(self, other):
        return abs(self.x - other.x) < 0.01 and abs(self.y - other.y) < 0.01
    
    def __lt__(self, other):
        """用于优先队列比较"""
        return (self.x, self.y) < (other.x, other.y)


class Obstacle:
    """障碍物"""
    
    def __init__(self, center: Point, radius: float):
        self.center = center
        self.radius = radius
    
    def contains(self, point: Point) -> bool:
        """检查点是否在障碍物内"""
        return self.center.distance_to(point) < self.radius


class AStarPlanner:
    """A*路径规划器"""
    
    def __init__(self, start: Point, goal: Point, obstacles: List[Obstacle],
                 grid_resolution: float = 0.1):
        """
        初始化A*规划器
        
        参数:
            start: 起点
            goal: 终点
            obstacles: 障碍物列表
            grid_resolution: 网格分辨率
        """
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.grid_resolution = grid_resolution
        
        # 搜索空间
        self.min_x = min(start.x, goal.x) - 2
        self.max_x = max(start.x, goal.x) + 2
        self.min_y = min(start.y, goal.y) - 2
        self.max_y = max(start.y, goal.y) + 2
    
    def is_collision_free(self, point: Point) -> bool:
        """检查点是否无碰撞"""
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return False
        return True
    
    def get_neighbors(self, point: Point) -> List[Point]:
        """获取邻居节点 (8连通)"""
        neighbors = []
        
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                new_point = Point(
                    point.x + dx * self.grid_resolution,
                    point.y + dy * self.grid_resolution
                )
                
                # 检查边界
                if (self.min_x <= new_point.x <= self.max_x and
                    self.min_y <= new_point.y <= self.max_y):
                    
                    # 检查碰撞
                    if self.is_collision_free(new_point):
                        neighbors.append(new_point)
        
        return neighbors
    
    def heuristic(self, point: Point) -> float:
        """启发函数 (欧几里得距离)"""
        return point.distance_to(self.goal)
    
    def plan(self) -> Optional[List[Point]]:
        """
        执行A*规划
        
        返回:
            路径点列表，如果找不到路径则返回None
        """
        # 开放列表 (优先队列)
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        
        # 记录
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start)}
        
        closed_set = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # 到达目标
            if current.distance_to(self.goal) < self.grid_resolution:
                return self.reconstruct_path(came_from, current)
            
            closed_set.add(current)
            
            # 探索邻居
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + current.distance_to(neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor)
                    
                    # 添加到开放列表
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # 找不到路径
        return None
    
    def reconstruct_path(self, came_from: Dict, current: Point) -> List[Point]:
        """重建路径"""
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path


class TrajectoryOptimizer:
    """轨迹优化器"""
    
    @staticmethod
    def smooth_path(path: List[Point], iterations: int = 10, 
                    alpha: float = 0.5, beta: float = 0.3) -> List[Point]:
        """
        平滑路径
        
        参数:
            path: 原始路径
            iterations: 迭代次数
            alpha: 平滑权重
            beta: 长度权重
        
        返回:
            平滑后的路径
        """
        if len(path) < 3:
            return path
        
        smoothed = [Point(p.x, p.y) for p in path]
        
        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                # 保存原始位置
                prev_x, prev_y = smoothed[i].x, smoothed[i].y
                
                # 朝向平滑方向移动
                smoothed[i].x += alpha * (path[i].x - smoothed[i].x)
                smoothed[i].y += alpha * (path[i].y - smoothed[i].y)
                
                # 朝向邻居的中点移动
                smoothed[i].x += beta * (smoothed[i-1].x + smoothed[i+1].x - 2*smoothed[i].x)
                smoothed[i].y += beta * (smoothed[i-1].y + smoothed[i+1].y - 2*smoothed[i].y)
        
        return smoothed
    
    @staticmethod
    def calculate_path_length(path: List[Point]) -> float:
        """计算路径长度"""
        length = 0
        for i in range(len(path) - 1):
            length += path[i].distance_to(path[i+1])
        return length


class TaskScheduler:
    """任务调度器"""
    
    def __init__(self):
        self.tasks = []
    
    def add_task(self, task: Dict):
        """
        添加任务
        
        任务格式: {
            'name': str,
            'priority': int,
            'duration': float,
            'deadline': float,
            'dependencies': list
        }
        """
        self.tasks.append(task)
    
    def schedule(self) -> List[Dict]:
        """
        调度任务
        
        返回:
            调度后的任务列表
        """
        # 简化的优先级调度 + 依赖处理
        scheduled = []
        completed = set()
        
        # 按优先级和截止时间排序
        remaining = sorted(
            self.tasks,
            key=lambda t: (t.get('priority', 0), t.get('deadline', float('inf')))
        )
        
        current_time = 0
        
        while remaining:
            # 找到可以执行的任务 (依赖已完成)
            available = [
                t for t in remaining
                if all(dep in completed for dep in t.get('dependencies', []))
            ]
            
            if not available:
                break  # 死锁或循环依赖
            
            # 选择最高优先级任务
            task = available[0]
            
            # 调度
            scheduled.append({
                'task': task,
                'start_time': current_time,
                'end_time': current_time + task.get('duration', 1.0)
            })
            
            current_time += task.get('duration', 1.0)
            completed.add(task['name'])
            remaining.remove(task)
        
        return scheduled


if __name__ == "__main__":
    print("任务规划系统加载完成")
    
    # 示例1: A*路径规划
    print("\n" + "="*70)
    print("示例1: A*路径规划")
    print("="*70)
    
    start = Point(0, 0)
    goal = Point(5, 5)
    
    # 创建一些障碍物
    obstacles = [
        Obstacle(Point(2, 2), 0.5),
        Obstacle(Point(3, 3), 0.6),
        Obstacle(Point(4, 1), 0.4)
    ]
    
    planner = AStarPlanner(start, goal, obstacles, grid_resolution=0.2)
    
    print(f"\n规划路径: ({start.x}, {start.y}) -> ({goal.x}, {goal.y})")
    print(f"障碍物数量: {len(obstacles)}")
    
    path = planner.plan()
    
    if path:
        print(f"✓ 找到路径! 路径点数: {len(path)}")
        path_length = TrajectoryOptimizer.calculate_path_length(path)
        print(f"  路径长度: {path_length:.2f} m")
        
        # 平滑路径
        smoothed = TrajectoryOptimizer.smooth_path(path, iterations=20)
        smoothed_length = TrajectoryOptimizer.calculate_path_length(smoothed)
        print(f"  平滑后长度: {smoothed_length:.2f} m")
    else:
        print("✗ 未找到路径")
    
    # 示例2: 任务调度
    print("\n" + "="*70)
    print("示例2: 任务调度")
    print("="*70)
    
    scheduler = TaskScheduler()
    
    # 添加任务
    scheduler.add_task({
        'name': '初始化',
        'priority': 10,
        'duration': 1.0,
        'dependencies': []
    })
    
    scheduler.add_task({
        'name': '移动到A点',
        'priority': 8,
        'duration': 5.0,
        'dependencies': ['初始化']
    })
    
    scheduler.add_task({
        'name': '执行任务A',
        'priority': 7,
        'duration': 3.0,
        'dependencies': ['移动到A点']
    })
    
    scheduler.add_task({
        'name': '返回基地',
        'priority': 5,
        'duration': 5.0,
        'dependencies': ['执行任务A']
    })
    
    schedule = scheduler.schedule()
    
    print(f"\n调度了 {len(schedule)} 个任务:")
    print(f"{'任务':<20} {'开始时间':<12} {'结束时间':<12}")
    print("-"*50)
    
    for item in schedule:
        task = item['task']
        print(f"{task['name']:<20} {item['start_time']:<12.1f} {item['end_time']:<12.1f}")
    
    if schedule:
        total_time = schedule[-1]['end_time']
        print(f"\n总完成时间: {total_time:.1f}s")
