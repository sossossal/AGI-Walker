"""
Skills 浏览面板 - AGI-Walker GUI

提供可视化的Skills系统界面。
"""

import logging
logger = logging.getLogger(__name__)
import sys
from pathlib import Path
from typing import Optional

try:
    from PyQt6.QtWidgets import (
        QApplication,
        QMainWindow,
        QWidget,
        QVBoxLayout,
        QHBoxLayout,
        QListWidget,
        QListWidgetItem,
        QTextEdit,
        QLineEdit,
        QPushButton,
        QLabel,
        QSplitter,
        QComboBox,
        QMessageBox,
    )
    from PyQt6.QtCore import Qt, QSize
    from PyQt6.QtGui import QFont, QIcon
except ImportError:
    logger.warning("PyQt6未安装,GUI功能不可用")
    logger.info("安装命令: pip install PyQt6")
    sys.exit(1)

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from agi_walker.skills_loader import get_skills_loader, SkillMetadata


class SkillsPanel(QWidget):
    """Skills浏览面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.loader = get_skills_loader()
        self.current_skill: Optional[SkillMetadata] = None

        self.init_ui()
        self.load_skills()

    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()

        # 顶部工具栏
        toolbar = self.create_toolbar()
        layout.addLayout(toolbar)

        # 主内容区域 (左右分割)
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # 左侧: Skills列表
        left_panel = self.create_skills_list()
        splitter.addWidget(left_panel)

        # 右侧: Skill详情
        right_panel = self.create_details_view()
        splitter.addWidget(right_panel)

        # 设置分割比例
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)

        layout.addWidget(splitter)

        self.setLayout(layout)

    def create_toolbar(self) -> QHBoxLayout:
        """创建工具栏"""
        toolbar = QHBoxLayout()

        # 搜索框
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("搜索Skills...")
        self.search_input.textChanged.connect(self.on_search)
        toolbar.addWidget(self.search_input)

        # 分类过滤
        self.category_filter = QComboBox()
        self.category_filter.addItem("所有分类")
        categories = self.loader.get_categories()
        for cat in sorted(categories):
            self.category_filter.addItem(cat)
        self.category_filter.currentTextChanged.connect(self.on_filter_category)
        toolbar.addWidget(self.category_filter)

        # 刷新按钮
        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.load_skills)
        toolbar.addWidget(refresh_btn)

        return toolbar

    def create_skills_list(self) -> QWidget:
        """创建Skills列表"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 标题
        title = QLabel("可用 Skills")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(12)
        title.setFont(title_font)
        layout.addWidget(title)

        # 列表
        self.skills_list = QListWidget()
        self.skills_list.itemClicked.connect(self.on_skill_selected)
        layout.addWidget(self.skills_list)

        widget.setLayout(layout)
        return widget

    def create_details_view(self) -> QWidget:
        """创建详情视图"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 标题区域
        self.detail_title = QLabel("选择一个Skill查看详情")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(14)
        self.detail_title.setFont(title_font)
        layout.addWidget(self.detail_title)

        # 基本信息
        self.detail_info = QLabel()
        self.detail_info.setWordWrap(True)
        layout.addWidget(self.detail_info)

        # 文档内容
        doc_label = QLabel("完整文档:")
        doc_label.setStyleSheet("margin-top: 10px; font-weight: bold;")
        layout.addWidget(doc_label)

        self.detail_doc = QTextEdit()
        self.detail_doc.setReadOnly(True)
        layout.addWidget(self.detail_doc)

        # 操作按钮
        btn_layout = QHBoxLayout()

        self.open_dir_btn = QPushButton("打开目录")
        self.open_dir_btn.clicked.connect(self.on_open_directory)
        self.open_dir_btn.setEnabled(False)
        btn_layout.addWidget(self.open_dir_btn)

        self.view_api_btn = QPushButton("查看API文档")
        self.view_api_btn.clicked.connect(self.on_view_api)
        self.view_api_btn.setEnabled(False)
        btn_layout.addWidget(self.view_api_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        widget.setLayout(layout)
        return widget

    def load_skills(self):
        """加载Skills列表"""
        self.skills_list.clear()

        skills = self.loader.get_skills_list()

        for skill in sorted(skills, key=lambda s: s.category):
            item = QListWidgetItem(f"{skill.emoji} {skill.name}")
            item.setData(Qt.ItemDataRole.UserRole, skill)

            # 添加工具提示
            tooltip = f"{skill.description}\n\n分类: {skill.category}"
            item.setToolTip(tooltip)

            self.skills_list.addItem(item)

        # 更新计数
        f"可用 Skills ({len(skills)} 个)"
        if hasattr(self, "skills_list"):
            # 更新标题 (如果需要的话)
            pass

    def on_search(self, text: str):
        """搜索Skills"""
        if not text:
            # 显示所有
            for i in range(self.skills_list.count()):
                self.skills_list.item(i).setHidden(False)
        else:
            # 过滤
            text_lower = text.lower()
            for i in range(self.skills_list.count()):
                item = self.skills_list.item(i)
                skill: SkillMetadata = item.data(Qt.ItemDataRole.UserRole)

                match = (
                    text_lower in skill.name.lower()
                    or text_lower in skill.description.lower()
                    or text_lower in skill.category.lower()
                )

                item.setHidden(not match)

    def on_filter_category(self, category: str):
        """按分类过滤"""
        if category == "所有分类":
            # 显示所有
            for i in range(self.skills_list.count()):
                self.skills_list.item(i).setHidden(False)
        else:
            # 过滤
            for i in range(self.skills_list.count()):
                item = self.skills_list.item(i)
                skill: SkillMetadata = item.data(Qt.ItemDataRole.UserRole)
                item.setHidden(skill.category != category)

    def on_skill_selected(self, item: QListWidgetItem):
        """选中Skill"""
        skill: SkillMetadata = item.data(Qt.ItemDataRole.UserRole)
        self.current_skill = skill

        # 更新标题
        self.detail_title.setText(f"{skill.emoji} {skill.name}")

        # 更新基本信息
        info_html = f"""
        <p><b>分类:</b> {skill.category}</p>
        <p><b>描述:</b> {skill.description}</p>
        <p><b>路径:</b> {skill.skill_dir}</p>
        """

        if skill.requires:
            info_html += "<p><b>依赖:</b></p><ul>"
            for dep_type, deps in skill.requires.items():
                info_html += f"<li>{dep_type}: {', '.join(deps)}</li>"
            info_html += "</ul>"

        self.detail_info.setText(info_html)

        # 加载文档
        try:
            doc = self.loader.get_skill_doc(skill.name)
            self.detail_doc.setMarkdown(doc)
        except Exception as e:
            self.detail_doc.setPlainText(f"加载文档失败: {e}")

        # 启用按钮
        self.open_dir_btn.setEnabled(True)
        self.view_api_btn.setEnabled(True)

    def on_open_directory(self):
        """打开Skill目录"""
        if self.current_skill:
            import os
            import subprocess

            path = str(self.current_skill.skill_dir)

            # Windows
            if sys.platform == "win32":
                os.startfile(path)
            # macOS
            elif sys.platform == "darwin":
                subprocess.run(["open", path])
            # Linux
            else:
                subprocess.run(["xdg-open", path])

    def on_view_api(self):
        """查看API文档"""
        if self.current_skill:
            api_path = self.current_skill.skill_dir / "references" / "api.md"

            if api_path.exists():
                # 在新窗口显示
                dialog = QMessageBox(self)
                dialog.setWindowTitle(f"{self.current_skill.name} - API文档")
                dialog.setText(api_path.read_text(encoding="utf-8"))
                dialog.exec()
            else:
                QMessageBox.information(self, "提示", "该Skill没有API文档")


class SkillsBrowserWindow(QMainWindow):
    """Skills浏览器主窗口"""

    def __init__(self):
        super().__init__()

        self.setWindowTitle("AGI-Walker Skills 浏览器")
        self.setGeometry(100, 100, 1200, 800)

        # 创建面板
        self.skills_panel = SkillsPanel()
        self.setCentralWidget(self.skills_panel)

        # 创建菜单栏
        self.create_menus()

        # 状态栏
        self.statusBar().showMessage("就绪")

    def create_menus(self):
        """创建菜单"""
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu("文件")

        refresh_action = file_menu.addAction("刷新")
        refresh_action.triggered.connect(self.skills_panel.load_skills)

        file_menu.addSeparator()

        exit_action = file_menu.addAction("退出")
        exit_action.triggered.connect(self.close)

        # 帮助菜单
        help_menu = menubar.addMenu("帮助")

        about_action = help_menu.addAction("关于")
        about_action.triggered.connect(self.show_about)

    def show_about(self):
        """显示关于对话框"""
        QMessageBox.about(
            self,
            "关于",
            """
            <h3>AGI-Walker Skills 浏览器</h3>
            <p>版本: 0.1.0</p>
            <p>可视化的Skills系统管理工具</p>
            <p>整合自 Moltbot Skills 架构</p>
            """,
        )


def main():
    """主函数"""
    app = QApplication(sys.argv)

    # 设置样式
    app.setStyle("Fusion")

    # 创建主窗口
    window = SkillsBrowserWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
