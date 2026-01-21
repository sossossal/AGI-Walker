"""
任务编辑器 GUI
基于 PyQt6 的可视化任务编辑界面
"""

import sys
from typing import Optional

try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QLineEdit, QComboBox, QPushButton, QTextEdit, QTabWidget,
        QFormLayout, QDoubleSpinBox, QGroupBox
    )
    from PyQt6.QtCore import Qt
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False
    print("⚠️ PyQt6 未安装，GUI 不可用")

from python_api.task_editor import TaskEditor, TaskConfig


class TaskEditorGUI(QMainWindow):
    """任务编辑器 GUI"""
    
    def __init__(self):
        super().__init__()
        self.editor = TaskEditor()
        self.current_task: Optional[TaskConfig] = None
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("AGI-Walker 任务编辑器")
        self.setGeometry(100, 100, 900, 700)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # 顶部工具栏
        toolbar = self.create_toolbar()
        layout.addLayout(toolbar)
        
        # 主编辑区域
        tabs = QTabWidget()
        tabs.addTab(self.create_basic_tab(), "基础信息")
        tabs.addTab(self.create_env_tab(), "环境参数")
        tabs.addTab(self.create_reward_tab(), "奖励函数")
        tabs.addTab(self.create_compare_tab(), "虚实对比")
        layout.addWidget(tabs)
    
    def create_toolbar(self):
        layout = QHBoxLayout()
        
        new_btn = QPushButton("新建任务")
        new_btn.clicked.connect(self.new_task)
        layout.addWidget(new_btn)
        
        save_btn = QPushButton("保存")
        save_btn.clicked.connect(self.save_task)
        layout.addWidget(save_btn)
        
        load_btn = QPushButton("加载")
        load_btn.clicked.connect(self.load_task)
        layout.addWidget(load_btn)
        
        layout.addStretch()
        return layout
    
    def create_basic_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        
        self.name_edit = QLineEdit()
        layout.addRow("任务名称:", self.name_edit)
        
        self.desc_edit = QLineEdit()
        layout.addRow("描述:", self.desc_edit)
        
        self.difficulty_combo = QComboBox()
        self.difficulty_combo.addItems(["easy", "medium", "hard"])
        layout.addRow("难度:", self.difficulty_combo)
        
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(["quadruped", "biped", "manipulator"])
        layout.addRow("机器人类型:", self.robot_combo)
        
        return widget
    
    def create_env_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        group = QGroupBox("环境参数")
        form = QFormLayout(group)
        
        self.env_params = {}
        params = ["stair_height", "stair_depth", "num_stairs"]
        for param in params:
            spin = QDoubleSpinBox()
            spin.setRange(0, 10)
            spin.setSingleStep(0.01)
            form.addRow(f"{param}:", spin)
            self.env_params[param] = spin
        
        layout.addWidget(group)
        layout.addStretch()
        return widget
    
    def create_reward_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        group = QGroupBox("奖励权重")
        form = QFormLayout(group)
        
        self.reward_weights = {}
        weights = ["forward", "stability", "energy"]
        for weight in weights:
            spin = QDoubleSpinBox()
            spin.setRange(-10, 10)
            spin.setSingleStep(0.1)
            form.addRow(f"{weight}:", spin)
            self.reward_weights[weight] = spin
        
        layout.addWidget(group)
        layout.addStretch()
        return widget
    
    def create_compare_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        compare_btn = QPushButton("对比虚拟与现实")
        compare_btn.clicked.connect(self.compare_tasks)
        layout.addWidget(compare_btn)
        
        self.compare_text = QTextEdit()
        self.compare_text.setReadOnly(True)
        layout.addWidget(self.compare_text)
        
        return widget
    
    def new_task(self):
        self.current_task = self.editor.create_task("new_task", "quadruped")
        self.name_edit.setText("new_task")
    
    def save_task(self):
        if not self.current_task:
            return
        
        # 更新任务配置
        self.current_task.name = self.name_edit.text()
        self.current_task.description = self.desc_edit.text()
        
        filepath = f"examples/tasks/configs/{self.current_task.name}.json"
        self.editor.save_task(self.current_task, filepath)
    
    def load_task(self):
        # TODO: 添加文件选择对话框
        pass
    
    def compare_tasks(self):
        # TODO: 实现对比功能
        self.compare_text.setText("对比功能开发中...")


def main():
    if not PYQT_AVAILABLE:
        print("请安装 PyQt6: pip install PyQt6")
        return
    
    app = QApplication(sys.argv)
    window = TaskEditorGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
