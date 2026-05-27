import struct
import random
import os

# ================= 配置参数 =================
# 1. PGM 和 YAML 的导出路径
MAP_EXPORT_DIR = "./src/nav_slam/map"

# 2. World 文件的导出路径
WORLD_EXPORT_DIR = "./src/gazebo_modele/world"

RESOLUTION = 0.05  # 0.05m/pixel
# 设定为 11m x 11m 的画布，以容纳 10x10m 的净空和四周 0.5m 厚的墙体
WIDTH_PIXELS = int(11.0 / RESOLUTION)   # 220 像素
HEIGHT_PIXELS = int(11.0 / RESOLUTION)  # 220 像素
ORIGIN_X = -5.5
ORIGIN_Y = -5.5

# 全局定位优化参数
NUM_OBSTACLES = 15            # 10x10空间减少随机障碍物数量，避免过于拥挤
MIN_SIZE = 0.4                # 障碍物最小尺寸 (米)
MAX_SIZE = 1.2                # 障碍物最大尺寸 (米)
SAFE_RADIUS = 1.5             # 中心点 (0,0) 的绝对安全半径，方便底盘出生和初始化

# 初始化障碍物列表
OBSTACLES = [
    # --- 0~3: 外围实体高墙 (2.0m高, 0.5m厚, 圈出 10x10m 内部净空) ---
    (0, 5.25, 11, 0.5), (0, -5.25, 11, 0.5), 
    (-5.25, 0, 0.5, 10), (5.25, 0, 0.5, 10),
    
    # --- 4~6: 强特征固定参考物（打破房间对称性，极大地帮助 2D 雷达全局定位） ---
    (3.0, 3.0, 2.0, 0.5),   # 右上角长条挡墙
    (-3.0, 2.0, 0.8, 0.8),  # 左上角方柱
    (1.5, -3.5, 1.5, 0.5)   # 右下角横向短墙
]

# --- 生成随机障碍物 ---
print(f"正在生成 {NUM_OBSTACLES} 个随机障碍物...")
for _ in range(NUM_OBSTACLES):
    w = random.uniform(MIN_SIZE, MAX_SIZE)
    h = random.uniform(MIN_SIZE, MAX_SIZE)
    
    # 将随机范围限制在内部 10x10m 区域内 (留出边界裕度)
    x = random.uniform(-4.5 + w/2, 4.5 - w/2)
    y = random.uniform(-4.5 + h/2, 4.5 - h/2)
    
    # 避开原点出生区域
    if abs(x) < SAFE_RADIUS and abs(y) < SAFE_RADIUS:
        continue
        
    OBSTACLES.append((x, y, w, h))

# ================= 1. 生成 Gazebo .world =================
def generate_world():
    world_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="localization_10m_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="localization_maze">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
"""
    world_footer = """    </model>
  </world>
</sdf>
"""
    links = ""
    for i, (x, y, w, h) in enumerate(OBSTACLES):
        if i < 4:
            # 外围墙体
            z_height = 2.0 
            z_pose = z_height / 2.0
            color = "Gazebo/Bricks"
        else:
            # 内部特征及随机障碍物，高度设置为1.0左右即可，完全满足2D雷达的扫描平面
            z_height = random.uniform(0.8, 1.2)
            z_pose = z_height / 2.0
            color = "Gazebo/Wood" # 内部用木纹区分

        links += f"""      <link name="obs_{i}">
        <pose>{x:.3f} {y:.3f} {z_pose:.3f} 0 0 0</pose>
        <collision name="col_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z_height:.3f}</size></box></geometry></collision>
        <visual name="vis_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z_height:.3f}</size></box></geometry><material><script><name>{color}</name></script></material></visual>
      </link>\n"""
    
    file_path = os.path.join(WORLD_EXPORT_DIR, 'localization_10m.world')
    with open(file_path, 'w') as f:
        f.write(world_header + links + world_footer)
    print(f"✅ 已生成 Gazebo 世界文件: {file_path}")

# ================= 2. 生成 PGM 栅格地图 =================
def generate_pgm():
    grid = [[255 for _ in range(WIDTH_PIXELS)] for _ in range(HEIGHT_PIXELS)]
    
    for (x_center, y_center, w, h) in OBSTACLES:
        for r in range(HEIGHT_PIXELS):
            for c in range(WIDTH_PIXELS):
                x = ORIGIN_X + c * RESOLUTION
                y = ORIGIN_Y + (HEIGHT_PIXELS - 1 - r) * RESOLUTION
                if abs(x - x_center) <= w/2.0 and abs(y - y_center) <= h/2.0:
                    grid[r][c] = 0

    file_path = os.path.join(MAP_EXPORT_DIR, 'localization_10m.pgm')
    with open(file_path, 'wb') as f:
        f.write(f"P5\n{WIDTH_PIXELS} {HEIGHT_PIXELS}\n255\n".encode('ascii'))
        for r in range(HEIGHT_PIXELS):
            for c in range(WIDTH_PIXELS):
                f.write(struct.pack('B', grid[r][c]))
    print(f"✅ 已生成高精度对齐地图: {file_path}")

# ================= 3. 生成 YAML 配置文件 =================
def generate_yaml():
    yaml_content = f"""image: localization_10m.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
mode: trinary
"""
    file_path = os.path.join(MAP_EXPORT_DIR, 'localization_10m.yaml')
    with open(file_path, 'w') as f:
        f.write(yaml_content)
    print(f"✅ 已生成地图配置文件: {file_path}")

if __name__ == "__main__":
    for directory in [MAP_EXPORT_DIR, WORLD_EXPORT_DIR]:
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"📁 已自动创建目录: {directory}")

    print("正在构建 2D全局定位 测试环境 (10x10m)...")
    generate_world()
    generate_pgm()
    generate_yaml()
    print("🎉 顺利完成！")