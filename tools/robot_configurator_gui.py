"""
AGI-Walker 机器人可视化配置器

一个完整的图形界面应用，用于：
- 浏览和选择零件
- 拖拽组装机器人
- 调整参数
- 启动仿真
- 查看实时反馈

使用方法:
    python tools/robot_configurator_gui.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import json
import os
import sys
from pathlib import Path
import threading
import time
from typing import Dict, List, Optional

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from parts_library.parts_manager import PartsLibrary
except ImportError:
    print("警告: 无法导入PartsLibrary，将使用模拟数据")
    PartsLibrary = None

try:
    from python_api.godot_client import GodotSimulationClient
except ImportError:
    print("警告: 无法导入GodotSimulationClient，仿真功能将不可用")
    GodotSimulationClient = None


class PartNode:
    """零件节点（画布上的可视化表示）"""
    
    def __init__(self, canvas, part_id: str, part_data: dict, x: int, y: int):
        self.canvas = canvas
        self.part_id = part_id
        self.part_data = part_data
        self.x = x
        self.y = y
        self.width = 80
        self.height = 60
        self.selected = False
        
        # 绘制节点
        self.rect = canvas.create_rectangle(
            x, y, x + self.width, y + self.height,
            fill='lightblue', outline='black', width=2,
            tags=('part', part_id)
        )
        
        # 零件名称
        part_name = part_data.get('model', part_id)[:10]
        self.text = canvas.create_text(
            x + self.width/2, y + self.height/2,
            text=part_name, tags=('part', part_id)
        )
        
    def select(self):
        """选中高亮"""
        self.canvas.itemconfig(self.rect, outline='red', width=3)
        self.selected = True
        
    def deselect(self):
        """取消选中"""
        self.canvas.itemconfig(self.rect, outline='black', width=2)
        self.selected = False
        
    def move(self, dx, dy):
        """移动节点"""
        self.canvas.move(self.rect, dx, dy)
        self.canvas.move(self.text, dx, dy)
        self.x += dx
        self.y += dy


class PartsLibraryPanel(ttk.Frame):
    """零件库面板"""
    
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        
        # 标题
        ttk.Label(self, text="零件库", font=('Arial', 12, 'bold')).pack(pady=5)
        
        # 搜索框
        search_frame = ttk.Frame(self)
        search_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(search_frame, text="搜索:").pack(side=tk.LEFT)
        self.search_var = tk.StringVar()
        self.search_var.trace('w', self.on_search)
        ttk.Entry(search_frame, textvariable=self.search_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # 分类树
        tree_frame = ttk.Frame(self)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=5)
        
        self.tree = ttk.Treeview(tree_frame, selectmode='browse')
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 详情面板
        detail_frame = ttk.LabelFrame(self, text="零件详情")
        detail_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.detail_text = tk.Text(detail_frame, height=8, wrap=tk.WORD, font=('Arial', 9))
        self.detail_text.pack(fill=tk.X, padx=5, pady=5)
        
        # 添加按钮
        add_frame = ttk.Frame(self)
        add_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(add_frame, text="➕ 添加到画布", command=self.add_to_canvas).pack(side=tk.LEFT, padx=5)
        ttk.Button(add_frame, text="🔍 查看详情", command=self.view_details).pack(side=tk.LEFT, padx=5)
        
        # 绑定事件
        self.tree.bind('<<TreeviewSelect>>', self.on_select)
        self.tree.bind('<Double-Button-1>', self.on_double_click)
        
        # 拖拽支持
        self.dragging = False
        self.tree.bind('<ButtonPress-1>', self.on_drag_start)
        self.tree.bind('<B1-Motion>', self.on_drag_motion)
        self.tree.bind('<ButtonRelease-1>', self.on_drag_end)
        
        # 加载零件库
        self.load_parts()
        
    def load_parts(self):
        """加载零件库"""
        # 模拟数据（如果无法导入真实库）
        categories = {
            '电机': [
                {'id': 'motor_1', 'model': 'Dynamixel XL430', 'power': 500},
                {'id': 'motor_2', 'model': 'Dynamixel AX-12', 'power': 300},
            ],
            '传感器': [
                {'id': 'imu_1', 'model': 'MPU6050', 'axes': 6},
                {'id': 'encoder_1', 'model': 'AS5048', 'resolution': 14},
            ],
            '控制器': [
                {'id': 'ctrl_1', 'model': 'Raspberry Pi 4', 'cpu': '1.5GHz'},
            ]
        }
        
        for category, parts in categories.items():
            cat_id = self.tree.insert('', 'end', text=category, open=True)
            for part in parts:
                self.tree.insert(cat_id, 'end', text=part['model'], values=(part['id'],))
                
    def on_select(self, event):
        """选中零件时显示详情"""
        selection = self.tree.selection()
        if selection:
            item = self.tree.item(selection[0])
            if item['values']:  # 是零件节点
                part_id = item['values'][0]
                # 显示详情
                details = f"零件ID: {part_id}\n型号: {item['text']}\n\n点击'添加到画布'来使用此零件"
                self.detail_text.delete(1.0, tk.END)
                self.detail_text.insert(1.0, details)
                
    def on_search(self, *args):
        """搜索功能"""
        # TODO: 实现搜索过滤
        pass
        
    def add_to_canvas(self):
        """添加选中的零件到画布"""
        selection = self.tree.selection()
        if selection:
            item = self.tree.item(selection[0])
            if item['values']:
                part_id = item['values'][0]
                part_data = {'model': item['text'], 'id': part_id}
                self.app.canvas_panel.add_part(part_id, part_data)
                
    def on_double_click(self, event):
        """双击添加到画布"""
        self.add_to_canvas()
        
    def view_details(self):
        """查看详细信息"""
        selection = self.tree.selection()
        if selection:
            item = self.tree.item(selection[0])
            if item['values']:
                part_id = item['values'][0]
                detail_window = tk.Toplevel(self.app.root)
                detail_window.title(f"零件详情 - {item['text']}")
                detail_window.geometry("400x300")
                
                text = tk.Text(detail_window, wrap=tk.WORD, padx=10, pady=10)
                text.pack(fill=tk.BOTH, expand=True)
                
                details = f"""
零件ID: {part_id}
型号: {item['text']}

规格参数:
- 功率: 500W
- 电压: 12V
- 扭矩: 1.4 Nm
- 重量: 57g

适用场景:
- 双足机器人
- 机械臂
- 四足机器人
                """
                text.insert(1.0, details.strip())
                text.config(state=tk.DISABLED)
                
    def on_drag_start(self, event):
        """开始拖拽"""
        item = self.tree.identify_row(event.y)
        if item:
            self.dragging = True
            self.drag_item = item
            
    def on_drag_motion(self, event):
        """拖拽中"""
        if self.dragging:
            # 改变鼠标样式提示
            self.tree.config(cursor='hand2')
            
    def on_drag_end(self, event):
        """结束拖拽"""
        if self.dragging:
            self.tree.config(cursor='')
            # 检查是否拖拽到画布区域
            canvas_widget = self.app.canvas_panel.canvas
            canvas_x = canvas_widget.winfo_rootx()
            canvas_y = canvas_widget.winfo_rooty()
            canvas_w = canvas_widget.winfo_width()
            canvas_h = canvas_widget.winfo_height()
            
            mouse_x = event.x_root
            mouse_y = event.y_root
            
            # 如果鼠标在画布区域内
            if (canvas_x <= mouse_x <= canvas_x + canvas_w and
                canvas_y <= mouse_y <= canvas_y + canvas_h):
                # 添加零件到画布
                item = self.tree.item(self.drag_item)
                if item['values']:
                    part_id = item['values'][0]
                    part_data = {'model': item['text'], 'id': part_id}
                    # 计算画布内的相对位置
                    rel_x = mouse_x - canvas_x
                    rel_y = mouse_y - canvas_y
                    self.app.canvas_panel.add_part_at(part_id, part_data, rel_x, rel_y)
                    
        self.dragging = False


class AssemblyCanvas(ttk.Frame):
    """组装画布面板"""
    
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.nodes = {}
        self.connections = []
        self.selected_node = None
        self.connecting_mode = False
        self.connect_start_node = None
        self.drag_data = {'x': 0, 'y': 0, 'item': None}
        
        # 标题
        ttk.Label(self, text="组装画布", font=('Arial', 12, 'bold')).pack()
        
        # 画布
        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.canvas = tk.Canvas(canvas_frame, bg='#f5f5f5', cursor='cross')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # 添加网格背景
        self.draw_grid()
        
        # 工具栏
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(toolbar, text="🗑️ 删除", command=self.delete_selected).pack(side=tk.LEFT, padx=2)
        self.connect_btn = ttk.Button(toolbar, text="🔗 连接模式", command=self.toggle_connect_mode)
        self.connect_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="🧹 清空", command=self.clear_canvas).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="📐 网格", command=self.toggle_grid).pack(side=tk.LEFT, padx=2)
        
        # 状态栏
        self.status_label = ttk.Label(self, text="就绪", relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(fill=tk.X, side=tk.BOTTOM)
        
        # 绑定事件
        self.canvas.bind('<Button-1>', self.on_click)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.canvas.bind('<Motion>', self.on_motion)
        
    def draw_grid(self):
        """绘制网格背景"""
        w = 800
        h = 600
        spacing = 20
        
        # 绘制网格线
        for i in range(0, w, spacing):
            self.canvas.create_line(i, 0, i, h, fill='#e0e0e0', tags='grid')
        for i in range(0, h, spacing):
            self.canvas.create_line(0, i, w, i, fill='#e0e0e0', tags='grid')
            
    def toggle_grid(self):
        """切换网格显示"""
        current = self.canvas.itemcget('grid', 'state')
        new_state = 'hidden' if current != 'hidden' else 'normal'
        self.canvas.itemconfig('grid', state=new_state)
        
    def add_part(self, part_id: str, part_data: dict):
        """添加零件到画布中心"""
        x = self.canvas.winfo_width() // 2 - 40
        y = len(self.nodes) * 80 + 50
        self.add_part_at(part_id, part_data, x, y)
        
    def add_part_at(self, part_id: str, part_data: dict, x: int, y: int):
        """在指定位置添加零件"""
        # 生成唯一ID
        unique_id = f"{part_id}_{len(self.nodes)}"
        
        node = PartNode(self.canvas, unique_id, part_data, x, y)
        self.nodes[unique_id] = node
        
        self.status_label.config(text=f"已添加: {part_data['model']}")
        
    def on_click(self, event):
        """点击画布"""
        if self.connecting_mode:
            # 连接模式：选择零件进行连接
            clicked = self.canvas.find_withtag(tk.CURRENT)
            if clicked:
                tags = self.canvas.gettags(clicked[0])
                for tag in tags:
                    if tag in self.nodes:
                        if not self.connect_start_node:
                            self.connect_start_node = tag
                            self.nodes[tag].select()
                            self.status_label.config(text=f"已选择起点: {tag}，请选择终点")
                        else:
                            # 创建连接
                            self.create_connection(self.connect_start_node, tag)
                            self.nodes[self.connect_start_node].deselect()
                            self.connect_start_node = None
                            self.status_label.config(text="连接已创建")
                        return
        else:
            # 普通模式：选择或拖拽
            clicked = self.canvas.find_withtag(tk.CURRENT)
            if clicked:
                tags = self.canvas.gettags(clicked[0])
                for tag in tags:
                    if tag in self.nodes:
                        self.select_node(tag)
                        # 准备拖拽
                        self.drag_data['item'] = tag
                        self.drag_data['x'] = event.x
                        self.drag_data['y'] = event.y
                        return
            # 取消选中
            self.deselect_all()
            
    def on_drag(self, event):
        """拖拽零件"""
        if self.drag_data['item'] and not self.connecting_mode:
            dx = event.x - self.drag_data['x']
            dy = event.y - self.drag_data['y']
            
            # 移动节点
            node = self.nodes[self.drag_data['item']]
            node.move(dx, dy)
            
            # 更新连接线
            self.update_connections(self.drag_data['item'])
            
            self.drag_data['x'] = event.x
            self.drag_data['y'] = event.y
            
    def on_release(self, event):
        """释放鼠标"""
        self.drag_data['item'] = None
        
    def on_motion(self, event):
        """鼠标移动（用于显示提示）"""
        x, y = event.x, event.y
        self.status_label.config(text=f"位置: ({x}, {y})")
        
    def toggle_connect_mode(self):
        """切换连接模式"""
        self.connecting_mode = not self.connecting_mode
        if self.connecting_mode:
            self.connect_btn.config(relief=tk.SUNKEN, text="🔗 连接中...")
            self.status_label.config(text="连接模式：选择第一个零件")
            self.canvas.config(cursor='crosshair')
        else:
            self.connect_btn.config(relief=tk.RAISED, text="🔗 连接模式")
            self.status_label.config(text="就绪")
            self.canvas.config(cursor='cross')
            if self.connect_start_node:
                self.nodes[self.connect_start_node].deselect()
                self.connect_start_node = None
                
    def create_connection(self, from_node, to_node):
        """创建两个零件之间的连接"""
        if from_node == to_node:
            messagebox.showwarning("警告", "不能连接到自己")
            return
            
        # 检查是否已存在连接
        for conn in self.connections:
            if (conn['from'] == from_node and conn['to'] == to_node) or \
               (conn['from'] == to_node and conn['to'] == from_node):
                messagebox.showwarning("警告", "连接已存在")
                return
        
        # 绘制连接线
        n1 = self.nodes[from_node]
        n2 = self.nodes[to_node]
        
        line = self.canvas.create_line(
            n1.x + n1.width/2, n1.y + n1.height/2,
            n2.x + n2.width/2, n2.y + n2.height/2,
            fill='blue', width=2, arrow=tk.LAST,
            tags='connection'
        )
        
        self.connections.append({
            'from': from_node,
            'to': to_node,
            'line': line
        })
        
    def update_connections(self, node_id):
        """更新节点相关的所有连接线"""
        node = self.nodes[node_id]
        
        for conn in self.connections:
            if conn['from'] == node_id or conn['to'] == node_id:
                from_node = self.nodes[conn['from']]
                to_node = self.nodes[conn['to']]
                
                self.canvas.coords(
                    conn['line'],
                    from_node.x + from_node.width/2,
                    from_node.y + from_node.height/2,
                    to_node.x + to_node.width/2,
                    to_node.y + to_node.height/2
                )
                    
    def select_node(self, part_id):
        """选中节点"""
        self.deselect_all()
        if part_id in self.nodes:
            self.nodes[part_id].select()
            self.selected_node = part_id
            # 在属性面板显示属性
            self.app.props_panel.show_properties(part_id, self.nodes[part_id].part_data)
            self.status_label.config(text=f"已选中: {part_id}")
            
    def deselect_all(self):
        """取消所有选中"""
        for node in self.nodes.values():
            node.deselect()
        self.selected_node = None
        
    def delete_selected(self):
        """删除选中的节点及相关连接"""
        if self.selected_node and self.selected_node in self.nodes:
            # 删除相关连接
            self.connections = [conn for conn in self.connections 
                              if conn['from'] != self.selected_node and conn['to'] != self.selected_node]
            self.canvas.delete('connection')
            # 重绘所有连接
            for conn in self.connections:
                from_node = self.nodes[conn['from']]
                to_node = self.nodes[conn['to']]
                line = self.canvas.create_line(
                    from_node.x + from_node.width/2,
                    from_node.y + from_node.height/2,
                    to_node.x + to_node.width/2,
                    to_node.y + to_node.height/2,
                    fill='blue', width=2, arrow=tk.LAST,
                    tags='connection'
                )
                conn['line'] = line
            
            # 删除节点
            node = self.nodes[self.selected_node]
            self.canvas.delete(node.rect)
            self.canvas.delete(node.text)
            del self.nodes[self.selected_node]
            self.selected_node = None
            self.status_label.config(text="已删除零件")
            
    def clear_canvas(self):
        """清空画布"""
        if messagebox.askyesno("确认", "确定要清空画布吗？"):
            self.canvas.delete('all')
            self.draw_grid()
            self.nodes.clear()
            self.connections.clear()
            self.selected_node = None
            self.status_label.config(text="画布已清空")


class PropertiesPanel(ttk.Frame):
    """属性面板"""
    
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.current_part = None
        
        # 标题
        ttk.Label(self, text="属性编辑器", font=('Arial', 12, 'bold')).pack(pady=5)
        
        # 内容区域
        self.content_frame = ttk.Frame(self)
        self.content_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 默认提示
        self.placeholder = ttk.Label(
            self.content_frame,
            text="← 从画布选择一个零件\n来编辑其属性",
            justify=tk.CENTER
        )
        self.placeholder.pack(expand=True)
        
    def show_properties(self, part_id, part_data):
        """显示零件属性"""
        # 清空当前内容
        for widget in self.content_frame.winfo_children():
            widget.destroy()
            
        # 显示零件信息
        info_frame = ttk.LabelFrame(self.content_frame, text="基本信息")
        info_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(info_frame, text=f"零件ID: {part_id}").pack(anchor=tk.W, padx=5, pady=2)
        ttk.Label(info_frame, text=f"型号: {part_data.get('model', 'N/A')}").pack(anchor=tk.W, padx=5, pady=2)
        
        # 参数编辑区
        params_frame = ttk.LabelFrame(self.content_frame, text="参数调整")
        params_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 示例参数
        parameters = {
            '功率倍增': (0.5, 2.0, 1.0),
            '刚度': (0.5, 3.0, 1.0),
            '阻尼': (0.1, 1.0, 0.5),
        }
        
        for param_name, (min_val, max_val, default) in parameters.items():
            param_frame = ttk.Frame(params_frame)
            param_frame.pack(fill=tk.X, padx=5, pady=3)
            
            ttk.Label(param_frame, text=param_name, width=10).pack(side=tk.LEFT)
            
            var = tk.DoubleVar(value=default)
            scale = ttk.Scale(
                param_frame,
                from_=min_val, to=max_val,
                variable=var,
                orient=tk.HORIZONTAL
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            entry = ttk.Entry(param_frame, textvariable=var, width=6)
            entry.pack(side=tk.LEFT)
            
        # 应用按钮
        ttk.Button(params_frame, text="✓ 应用更改", command=self.apply_changes).pack(pady=5)
        
    def apply_changes(self):
        """应用参数更改"""
        messagebox.showinfo("成功", "参数已更新")


class FeedbackPanel(ttk.Frame):
    """数据反馈面板"""
    
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        
        # 标题
        ttk.Label(self, text="仿真反馈", font=('Arial', 12, 'bold')).pack()
        
        # 控制按钮
        control_frame = ttk.Frame(self)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.start_btn = ttk.Button(control_frame, text="▶️ 启动仿真", command=self.start_simulation)
        self.start_btn.pack(side=tk.LEFT, padx=2)
        
        self.stop_btn = ttk.Button(control_frame, text="⏹️ 停止", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        
        # 状态显示
        status_frame = ttk.Frame(self)
        status_frame.pack(fill=tk.X, padx=5)
        
        self.status_label = ttk.Label(status_frame, text="状态: 待机", foreground='blue')
        self.status_label.pack(side=tk.LEFT)
        
        # 数据显示
        data_frame = ttk.Frame(self)
        data_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.position_label = ttk.Label(data_frame, text="位置: 0.00m")
        self.position_label.pack(side=tk.LEFT, padx=10)
        
        self.velocity_label = ttk.Label(data_frame, text="速度: 0.00m/s")
        self.velocity_label.pack(side=tk.LEFT, padx=10)
        
        self.battery_label = ttk.Label(data_frame, text="电量: 100%")
        self.battery_label.pack(side=tk.LEFT, padx=10)
        
        # matplotlib 图表
        self.fig = Figure(figsize=(10, 3), dpi=80)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("实时数据")
        self.ax.set_xlabel("时间 (s)")
        self.ax.set_ylabel("位置 (m)")
        
        self.canvas = FigureCanvasTkAgg(self.fig, self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 数据缓冲
        self.time_data = []
        self.position_data = []
        
    def start_simulation(self):
        """启动仿真"""
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_label.config(text="状态: 运行中", foreground='green')
        
        # 模拟数据更新
        self.update_simulation()
        
    def stop_simulation(self):
        """停止仿真"""
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="状态: 已停止", foreground='red')
        
    def update_simulation(self):
        """更新仿真数据（模拟）"""
        if self.stop_btn['state'] == tk.NORMAL:
            import random
            
            # 模拟数据
            t = len(self.time_data) * 0.1
            pos = t * 0.1 + random.uniform(-0.01, 0.01)
            vel = 0.1 + random.uniform(-0.02, 0.02)
            bat = max(0, 100 - t * 0.5)
            
            self.time_data.append(t)
            self.position_data.append(pos)
            
            # 更新标签
            self.position_label.config(text=f"位置: {pos:.2f}m")
            self.velocity_label.config(text=f"速度: {vel:.2f}m/s")
            self.battery_label.config(text=f"电量: {bat:.1f}%")
            
            # 更新图表
            self.ax.clear()
            self.ax.plot(self.time_data, self.position_data, 'b-')
            self.ax.set_title("实时位置")
            self.ax.set_xlabel("时间 (s)")
            self.ax.set_ylabel("位置 (m)")
            self.ax.grid(True)
            self.canvas.draw()
            
            # 继续更新
            self.after(100, self.update_simulation)


class RobotConfiguratorGUI:
    """主应用程序"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("AGI-Walker 机器人配置器 v1.0")
        self.root.geometry("1400x900")
        
        # 创建菜单
        self.create_menu()
        
        # 创建主布局
        self.create_layout()
        
    def create_menu(self):
        """创建菜单栏"""
        menubar = tk.Menu(self.root)
        
        # 文件菜单
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="新建配置", command=self.new_config)
        file_menu.add_command(label="打开配置...", command=self.load_config)
        file_menu.add_command(label="保存配置", command=self.save_config)
        file_menu.add_separator()
        file_menu.add_command(label="退出", command=self.root.quit)
        menubar.add_cascade(label="文件", menu=file_menu)
        
        # 帮助菜单
        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="使用帮助", command=self.show_help)
        help_menu.add_command(label="关于", command=self.show_about)
        menubar.add_cascade(label="帮助", menu=help_menu)
        
        self.root.config(menu=menubar)
        
    def create_layout(self):
        """创建主布局"""
        # 主容器 - 使用PanedWindow实现可调整大小
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True)
        
        # 左侧：零件库（20%宽度）
        self.parts_panel = PartsLibraryPanel(main_paned, self)
        self.parts_panel.config(width=280)
        main_paned.add(self.parts_panel, weight=1)
        
        # 中间+右侧容器
        center_right_paned = ttk.PanedWindow(main_paned, orient=tk.HORIZONTAL)
        main_paned.add(center_right_paned, weight=4)
        
        # 中间：组装画布（50%宽度）
        self.canvas_panel = AssemblyCanvas(center_right_paned, self)
        center_right_paned.add(self.canvas_panel, weight=2)
        
        # 右侧：属性面板（30%宽度）
        self.props_panel = PropertiesPanel(center_right_paned, self)
        self.props_panel.config(width=300)
        center_right_paned.add(self.props_panel, weight=1)
        
        # 底部：反馈面��
        bottom_frame = ttk.Frame(self.root)
        bottom_frame.pack(fill=tk.BOTH, expand=False, side=tk.BOTTOM)
        
        self.feedback_panel = FeedbackPanel(bottom_frame, self)
        self.feedback_panel.pack(fill=tk.BOTH, expand=True)
        
    def new_config(self):
        """新建配置"""
        if messagebox.askyesno("确认", "确定要新建配置吗？当前配置将丢失。"):
            self.canvas_panel.clear_canvas()
            messagebox.showinfo("成功", "已创建新配置")
            
    def load_config(self):
        """加载配置"""
        filename = filedialog.askopenfilename(
            title="打开配置文件",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                # TODO: 加载配置到界面
                messagebox.showinfo("成功", f"已加载配置: {filename}")
            except Exception as e:
                messagebox.showerror("错误", f"加载失败: {e}")
                
    def save_config(self):
        """保存配置"""
        filename = filedialog.asksaveasfilename(
            title="保存配置文件",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                config = {
                    'robot_name': '我的机器人',
                    'parts': [],  # TODO: 从画布收集零件数据
                    'connections': []
                }
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                messagebox.showinfo("成功", f"已保存配置: {filename}")
            except Exception as e:
                messagebox.showerror("错误", f"保存失败: {e}")
                
    def show_help(self):
        """显示帮助"""
        help_text = """
AGI-Walker 机器人配置器 使用帮助

1. 从左侧零件库选择零件
2. 点击"添加到画布"将零件放到中间画布
3. 点击画布上的零件可在右侧编辑属性
4. 调整参数后点击"应用更改"
5. 点击底部的"启动仿真"开始测试
6. 使用"文件"菜单保存/加载配置
        """
        messagebox.showinfo("使用帮助", help_text)
        
    def show_about(self):
        """关于对话框"""
        messagebox.showinfo(
            "关于",
            "AGI-Walker 机器人配置器 v1.0\n\n"
            "一个可视化的机器人设计和仿真工具\n\n"
            "© 2026 AGI-Walker Project"
        )
        
    def run(self):
        """运行应用"""
        self.root.mainloop()


def main():
    """主函数"""
    app = RobotConfiguratorGUI()
    app.run()


if __name__ == "__main__":
    main()
