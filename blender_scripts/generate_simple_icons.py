"""
纯Python替代方案 - 为零件生成简单的图标图片
不需要Blender，使用PIL/Pillow生成基础图标

使用方法:
    python blender_scripts/generate_simple_icons.py
"""

from PIL import Image, ImageDraw, ImageFont
from pathlib import Path


def create_motor_icon(size=256):
    """创建电机图标"""
    img = Image.new('RGBA', (size, size), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    
    # 主体（圆柱体侧视图）
    center_x, center_y = size // 2, size // 2
    radius = size // 3
    
    # 本体
    draw.ellipse(
        [center_x - radius, center_y - radius*1.5, 
         center_x + radius, center_y + radius*1.5],
        fill=(50, 50, 50), outline=(30, 30, 30), width=3
    )
    
    # 输出轴
    shaft_width = radius // 4
    draw.rectangle(
        [center_x - shaft_width, center_y - radius*2,
         center_x + shaft_width, center_y - radius*1.5],
        fill=(80, 80, 80), outline=(60, 60, 60), width=2
    )
    
    # 高光
    draw.ellipse(
        [center_x - radius//2, center_y - radius,
         center_x + radius//4, center_y - radius//4],
        fill=(120, 120, 120, 100)
    )
    
    return img


def create_imu_icon(size=256):
    """创建IMU传感器图标"""
    img = Image.new('RGBA', (size, size), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    
    center_x, center_y = size // 2, size // 2
    width, height = size // 2, size // 3
    
    # PCB板
    draw.rectangle(
        [center_x - width, center_y - height,
         center_x + width, center_y + height],
        fill=(40, 120, 60), outline=(30, 90, 45), width=3
    )
    
    # 芯片
    chip_size = width // 2
    draw.rectangle(
        [center_x - chip_size//2, center_y - chip_size//2,
         center_x + chip_size//2, center_y + chip_size//2],
        fill=(30, 30, 30), outline=(20, 20, 20), width=2
    )
    
    # 引脚（装饰）
    for i in range(4):
        x = center_x - width + i * (width // 2)
        draw.rectangle([x-2, center_y+height, x+2, center_y+height+10],
                      fill=(180, 180, 180))
    
    return img


def create_controller_icon(size=256):
    """创建Raspberry Pi图标"""
    img = Image.new('RGBA', (size, size), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    
    center_x, center_y = size // 2, size // 2
    width, height = int(size * 0.6), int(size * 0.4)
    
    # 主板
    draw.rectangle(
        [center_x - width//2, center_y - height//2,
         center_x + width//2, center_y + height//2],
        fill=(50, 140, 80), outline=(40, 110, 60), width=3
    )
    
    # USB端口（侧面）
    for i in range(2):
        y = center_y - height//4 + i * (height//2)
        draw.rectangle(
            [center_x + width//2, y - 10,
             center_x + width//2 + 15, y + 10],
            fill=(180, 180, 180), outline=(150, 150, 150), width=2
        )
    
    # GPIO针脚（上部）
    gpio_count = 10
    gpio_spacing = width // (gpio_count + 1)
    for i in range(gpio_count):
        x = center_x - width//2 + (i+1) * gpio_spacing
        draw.rectangle(
            [x-2, center_y - height//2 - 8,
             x+2, center_y - height//2],
            fill=(220, 200, 60)
        )
    
    return img


def create_joint_icon(size=256):
    """创建关节图标"""
    img = Image.new('RGBA', (size, size), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    
    center_x, center_y = size // 2, size // 2
    
    # 基座
    draw.ellipse(
        [center_x - 60, center_y - 20,
         center_x + 60, center_y + 20],
        fill=(80, 80, 80), outline=(60, 60, 60), width=3
    )
    
    # 轴承环
    draw.ellipse(
        [center_x - 40, center_y - 40,
         center_x + 40, center_y + 40],
        fill=None, outline=(100, 100, 100), width=8
    )
    
    # 连接臂
    draw.rectangle(
        [center_x, center_y - 15,
         center_x + 80, center_y + 15],
        fill=(90, 90, 90), outline=(70, 70, 70), width=2
    )
    
    # 高光
    draw.arc(
        [center_x - 35, center_y - 35,
         center_x, center_y],
        start=200, end=340, fill=(150, 150, 150), width=3
    )
    
    return img


def create_battery_icon(size=256):
    """创建电池包图标"""
    img = Image.new('RGBA', (size, size), (255, 255, 255, 0))
    draw = ImageDraw.Draw(img)
    
    center_x, center_y = size // 2, size // 2
    width, height = size // 2 + 20, size // 3
    
    # 电池盒主体
    draw.rectangle(
        [center_x - width//2, center_y - height//2,
         center_x + width//2, center_y + height//2],
        fill=(20, 80, 180), outline=(15, 60, 140), width=3
    )
    
    # 正极（右侧）
    draw.rectangle(
        [center_x + width//2, center_y - 8,
         center_x + width//2 + 12, center_y + 8],
        fill=(200, 80, 80), outline=(160, 60, 60), width=2
    )
    
    # 电量指示（装饰）
    segments = 4
    seg_width = width // (segments + 1)
    for i in range(segments):
        x = center_x - width//2 + (i+1) * seg_width
        alpha = 255 - i * 50
        draw.rectangle(
            [x - seg_width//3, center_y - height//3,
             x + seg_width//3, center_y + height//3],
            fill=(255, 255, 255, alpha)
        )
    
    return img


def main():
    """生成所有图标"""
    
    # 创建输出目录
    output_dir = Path("assets/thumbnails")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("生成零件图标（PIL方案）...")
    print("=" * 60)
    
    icons = {
        'motor_1.png': create_motor_icon,
        'imu_1.png': create_imu_icon,
        'ctrl_1.png': create_controller_icon,
        'joint_1.png': create_joint_icon,
        'battery_1.png': create_battery_icon,
    }
    
    for filename, create_func in icons.items():
        output_path = output_dir / filename
        
        # 生成256x256图标
        icon = create_func(256)
        icon.save(output_path, 'PNG')
        
        print(f"✓ 已生成: {output_path}")
    
    print("=" * 60)
    print(f"✓ 完成! 生成了 {len(icons)} 个图标")
    print(f"✓ 位置: {output_dir}")
    print("=" * 60)
    
    # 生成小尺寸版本（用于GUI显示）
    print("\n生成小尺寸版本（64x64）...")
    small_dir = output_dir / "small"
    small_dir.mkdir(exist_ok=True)
    
    for filename, create_func in icons.items():
        small_icon = create_func(64)
        small_path = small_dir / filename
        small_icon.save(small_path, 'PNG')
        print(f"✓ 已生成小尺寸: {small_path}")
    
    print("\n全部完成!")


if __name__ == "__main__":
    main()
