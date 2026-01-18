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
    """零件节点 - 在画布上表示一个零件"""
    
    # 零件ID到图标文件名的映射
    ICON_MAPPING = {
        'motor_1': 'motor_1.png',
        'motor_2': 'motor_1.png',  # 使用相同图标
        'imu_1': 'imu_1.png',
        'encoder_1': 'imu_1.png',  # 传感器使用IMU图标
        'ctrl_1': 'ctrl_1.png',
        'joint_1': 'joint_1.png',
        'battery_1': 'battery_1.png',
        'battery_2': 'battery_1.png',
    }
    
    def __init__(self, canvas, part_id, part_data, x, y):
        self.canvas = canvas
        self.part_id = part_id
        self.part_data = part_data
        self.x = x
        self.y = y
        self.selected = False
        
        self.rect = None
        self.text = None
        self.image_obj = None
        self.photo = None  # 保持图片引用
        
        self.draw()
        
    def draw(self):
        """绘制零件节点"""
        # 尝试加载图标
        icon_loaded = self.load_icon()
        
        if not icon_loaded:
            # 降级到矩形
            self.rect = self.canvas.create_rectangle(
                self.x, self.y, self.x + 60, self.y + 60,
                fill='lightblue', outline='blue', width=2,
                tags=('part', self.part_id)
            )
        
        # 文本标签（总是显示）
        model_name = self.part_data.get('model', 'Unknown')
        self.text = self.canvas.create_text(
            self.x + 30, self.y + 70,
            text=model_name[:10],  # 限制长度
            font=('Arial', 9),
            tags=('part', self.part_id)
        )
        
    def load_icon(self):
        """
        加载零件图标
        
       Returns:
            bool: 是否成功加载图标
        """
        try:
            from PIL import Image, ImageTk
            from pathlib import Path
            
            # 获取图标文件名
            base_id = self.part_id.rsplit('_', 1)[0]  # 移除后缀数字
            icon_filename = self.ICON_MAPPING.get(base_id)
            
            if not icon_filename:
                return False
            
            # 构建图标路径
            icon_path = Path("assets") / "thumbnails" / "small" / icon_filename
            
            if not icon_path.exists():
                # 尝试标准尺寸
                icon_path = Path("assets") / "thumbnails" / icon_filename
            
            if not icon_path.exists():
                return False
            
            # 加载并调整图片
            img = Image.open(icon_path)
            img = img.resize((60, 60), Image.LANCZOS)
            self.photo = ImageTk.PhotoImage(img)
            
            # 在画布上显示图片
            self.image_obj = self.canvas.create_image(
                self.x + 30, self.y + 30,
                image=self.photo,
                tags=('part', self.part_id)
            )
            
            return True
            
        except Exception as e:
            print(f"加载图标失败 {self.part_id}: {e}")
            return False
        
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
        """搜索功能 - 实时过滤零件"""
        search_term = self.search_var.get().lower()
        
        # 清空树
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        # 重新加载并过滤
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
            # 过滤零件
            filtered_parts = [
                p for p in parts 
                if not search_term or 
                search_term in p['model'].lower() or 
                search_term in p['id'].lower() or
                search_term in category.lower()
            ]
            
            # 只显示有匹配零件的分类
            if filtered_parts:
                cat_id = self.tree.insert('', 'end', text=category, open=True)
                for part in filtered_parts:
                    self.tree.insert(cat_id, 'end', text=part['model'], values=(part['id'],))
        
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
        self.current_part_id = None
        self.param_vars = {}
        self.auto_sync_var = None
        
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
        
        # 保存当前零件ID
        self.current_part_id = part_id
            
        # 显示零件信息
        info_frame = ttk.LabelFrame(self.content_frame, text="基本信息")
        info_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(info_frame, text=f"零件ID: {part_id}").pack(anchor=tk.W, padx=5, pady=2)
        ttk.Label(info_frame, text=f"型号: {part_data.get('model', 'N/A')}").pack(anchor=tk.W, padx=5, pady=2)
        
        # 参数编辑区
        params_frame = ttk.LabelFrame(self.content_frame, text="参数调整")
        params_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 实时同步选项
        sync_frame = ttk.Frame(params_frame)
        sync_frame.pack(fill=tk.X, padx=5, pady=3)
        
        self.auto_sync_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            sync_frame, 
            text="实时同步到Godot", 
            variable=self.auto_sync_var
        ).pack(side=tk.LEFT)
        
        ttk.Label(sync_frame, text="（需先连接Godot）", foreground='gray').pack(side=tk.LEFT, padx=5)
        
        # 示例参数
        self.param_vars = {}
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
            self.param_vars[param_name] = var
            
            # 滑块改变时的回调
            def on_param_change(value, name=param_name):
                # 更新输入框
                # 如果启用了实时同步，则自动同步
                if self.auto_sync_var.get():
                    self.sync_to_godot()
            
            scale = ttk.Scale(
                param_frame,
                from_=min_val, to=max_val,
                variable=var,
                orient=tk.HORIZONTAL,
                command=on_param_change
            )
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            entry = ttk.Entry(param_frame, textvariable=var, width=6)
            entry.pack(side=tk.LEFT)
        
        # 应用按钮
        button_frame = ttk.Frame(params_frame)
        button_frame.pack(pady=5)
        
        ttk.Button(button_frame, text="✓ 应用更改", command=self.apply_changes).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="🔄 同步到Godot", command=self.sync_to_godot).pack(side=tk.LEFT, padx=2)
        
    def sync_to_godot(self):
        """同步参数到Godot"""
        if not hasattr(self.app, 'feedback_panel'):
            return
            
        feedback = self.app.feedback_panel
        
        if not feedback.godot_client or not feedback.godot_client.is_connected():
            if self.auto_sync_var.get():
                # 只在手动点击时提示
                pass
            return
        
        # 收集参数
        params = {}
        param_mapping = {
            '功率倍增': 'motor_power_multiplier',
            '刚度': 'joint_stiffness',
            '阻尼': 'joint_damping'
        }
        
        for display_name, param_key in param_mapping.items():
            if display_name in self.param_vars:
                params[param_key] = self.param_vars[display_name].get()
        
        # 发送到Godot
        success = feedback.godot_client.update_parameters(params)
        
        if success and not self.auto_sync_var.get():
            # 只在手动同步时显示提示
            messagebox.showinfo("成功", "参数已同步到Godot")
        
    def apply_changes(self):
        """应用参数更改"""
        # 同步到Godot（如果连接）
        self.sync_to_godot()
        messagebox.showinfo("成功", "参数已更新")


class FeedbackPanel(ttk.Frame):
    """数据反馈面板（支持真实Godot仿真）"""
    
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.godot_client = None
        self.simulation_running = False
        
        # 标题
        ttk.Label(self, text="仿真反馈", font=('Arial', 12, 'bold')).pack()
        
        # Godot连接控制
        connect_frame = ttk.LabelFrame(self, text="Godot连接")
        connect_frame.pack(fill=tk.X, padx=5, pady=5)
        
        conn_ctrl_frame = ttk.Frame(connect_frame)
        conn_ctrl_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(conn_ctrl_frame, text="地址:").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value="127.0.0.1")
        ttk.Entry(conn_ctrl_frame, textvariable=self.host_var, width=12).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(conn_ctrl_frame, text="端口:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar(value="9999")
        ttk.Entry(conn_ctrl_frame, textvariable=self.port_var, width=6).pack(side=tk.LEFT, padx=2)
        
        self.connect_btn = ttk.Button(conn_ctrl_frame, text="🔌 连接", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.conn_status_label = ttk.Label(conn_ctrl_frame, text="● 未连接", foreground='gray')
        self.conn_status_label.pack(side=tk.LEFT, padx=5)
        
        # 仿真控制
        control_frame = ttk.Frame(self)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.start_btn = ttk.Button(control_frame, text="▶️ 启动仿真", command=self.start_simulation, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT, padx=2)
        
        self.stop_btn = ttk.Button(control_frame, text="⏹️ 停止", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        
        ttk.Button(control_frame, text="🔄 同步参数", command=self.sync_parameters).pack(side=tk.LEFT, padx=2)
        
        # 模式切换
        self.mode_var = tk.StringVar(value="godot")
        ttk.Radiobutton(control_frame, text="Godot", variable=self.mode_var, value="godot").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(control_frame, text="模拟", variable=self.mode_var, value="mock").pack(side=tk.LEFT)
        
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
        self.ax.grid(True)
        
        self.canvas = FigureCanvasTkAgg(self.fig, self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 数据缓冲
        self.time_data = []
        self.position_data = []
        self.start_time = None
        
    def toggle_connection(self):
        """切换Godot连接"""
        if self.godot_client and self.godot_client.is_connected():
            # 断开连接
            self.godot_client.disconnect()
            self.godot_client = None
            self.connect_btn.config(text="🔌 连接")
            self.conn_status_label.config(text="● 未连接", foreground='gray')
            self.start_btn.config(state=tk.DISABLED)
        else:
            # 连接到Godot
            if GodotSimulationClient is None:
                messagebox.showerror("错误", "Godot客户端模块未安装\n\n请查看 docs/GODOT_INTEGRATION_GUIDE.md")
                return
                
            host = self.host_var.get()
            try:
                port = int(self.port_var.get())
            except ValueError:
                messagebox.showerror("错误", "端口必须是数字")
                return
            
            self.status_label.config(text="正在连接...", foreground='orange')
            self.conn_status_label.config(text="● 连接中...", foreground='orange')
            
            # 在后台线程连接
            def do_connect():
                self.godot_client = GodotSimulationClient(host, port)
                self.godot_client.set_data_callback(self.on_godot_data)
                
                success = self.godot_client.connect(timeout=3.0)
                
                # 回到主线程更新UI
                self.after(0, lambda: self.on_connect_result(success))
            
            thread = threading.Thread(target=do_connect, daemon=True)
            thread.start()
    
    def on_connect_result(self, success):
        """连接结果回调"""
        if success:
            self.connect_btn.config(text="🔌 断开")
            self.conn_status_label.config(text="● 已连接", foreground='green')
            self.status_label.config(text="状态: 已连接到Godot", foreground='green')
            self.start_btn.config(state=tk.NORMAL)
            
            messagebox.showinfo("成功", f"已连接到Godot服务器\n{self.host_var.get()}:{self.port_var.get()}")
        else:
            self.godot_client = None
            self.conn_status_label.config(text="● 失败", foreground='red')
            self.status_label.config(text="状态: 连接失败", foreground='red')
            
            messagebox.showerror(
                "连接失败",
                "无法连接到Godot服务器\n\n请确保：\n1. Godot已启动\n2. TCP服务器已运行\n3. 地址和端口正确\n\n详见: docs/GODOT_INTEGRATION_GUIDE.md"
            )
        
    def start_simulation(self):
        """启动仿真"""
        mode = self.mode_var.get()
        
        if mode == "godot":
            # Godot模式
            if not self.godot_client or not self.godot_client.is_connected():
                messagebox.showwarning("警告", "请先连接到Godot")
                return
            
            # 收集机器人配置
            robot_config = self.get_robot_config()
            
            # 发送启动命令
            success = self.godot_client.start_simulation(robot_config)
            
            if success:
                self.simulation_running = True
                self.start_btn.config(state=tk.DISABLED)
                self.stop_btn.config(state=tk.NORMAL)
                self.status_label.config(text="状态: Godot仿真中", foreground='green')
                
                # 清空数据
                self.time_data.clear()
                self.position_data.clear()
                self.start_time = None
            else:
                messagebox.showerror("错误", "启动仿真失败")
        else:
            # 模拟模式
            self.simulation_running = True
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(text="状态: 模拟运行中", foreground='green')
            
            # 清空数据
            self.time_data.clear()
            self.position_data.clear()
            
            # 启动模拟更新
            self.update_mock_simulation()
        
    def stop_simulation(self):
        """停止仿真"""
        if self.godot_client and self.mode_var.get() == "godot":
            self.godot_client.stop_simulation()
        
        self.simulation_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="状态: 已停止", foreground='red')
        
    def sync_parameters(self):
        """同步参数到Godot"""
        if not self.godot_client or not self.godot_client.is_connected():
            messagebox.showwarning("警告", "请先连接到Godot")
            return
        
        # TODO: 从属性面板获取实际参数
        params = {
            'motor_power_multiplier': 1.0,
            'joint_stiffness': 1.0,
            'joint_damping': 0.5
        }
        
        success = self.godot_client.update_parameters(params)
        if success:
            self.status_label.config(text="参数已同步", foreground='green')
        else:
            messagebox.showerror("错误", "参数同步失败")
    
    def get_robot_config(self):
        """从画布获取机器人配置"""
        parts = []
        for node_id, node in self.app.canvas_panel.nodes.items():
            parts.append({
                'id': node_id,
                'type': node.part_data.get('id', ''),
                'model': node.part_data.get('model', ''),
                'position': [node.x, node.y]
            })
        
        connections = []
        for conn in self.app.canvas_panel.connections:
            connections.append({
                'from': conn['from'],
                'to': conn['to']
            })
        
        return {
            'parts': parts,
            'connections': connections
        }
    
    def on_godot_data(self, data):
        """接收Godot数据的回调（在后台线程调用）"""
        # 调度到主线程更新UI
        self.after(0, lambda: self.update_display(data))
    
    def update_display(self, data):
        """更新显示（在主线程）"""
        if data.get('type') == 'simulation_data':
            # 提取数据
            pos = data.get('position', 0)
            vel = data.get('velocity', 0)
            bat = data.get('battery', 100)
            timestamp = data.get('timestamp', time.time())
            
            # 添加到缓冲
            if self.start_time is None:
                self.start_time = timestamp
            t = timestamp - self.start_time
            
            self.time_data.append(t)
            self.position_data.append(pos)
            
            # 更新标签
            self.position_label.config(text=f"位置: {pos:.2f}m")
            self.velocity_label.config(text=f"速度: {vel:.2f}m/s")
            self.battery_label.config(text=f"电量: {bat:.1f}%")
            
            # 更新图表
            self.ax.clear()
            self.ax.plot(self.time_data, self.position_data, 'b-', label='位置')
            self.ax.set_title("实时数据（Godot）")
            self.ax.set_xlabel("时间 (s)")
            self.ax.set_ylabel("位置 (m)")
            self.ax.grid(True)
            self.ax.legend()
            self.canvas.draw()
    
    def update_mock_simulation(self):
        """更新模拟数据"""
        if self.stop_btn['state'] == tk.NORMAL and self.mode_var.get() == "mock":
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
            self.ax.plot(self.time_data, self.position_data, 'g-', label='位置（模拟）')
            self.ax.set_title("实时数据（模拟）")
            self.ax.set_xlabel("时间 (s)")
            self.ax.set_ylabel("位置 (m)")
            self.ax.grid(True)
            self.ax.legend()
            self.canvas.draw()
            
            # 继续更新
            self.after(100, self.update_mock_simulation)


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
