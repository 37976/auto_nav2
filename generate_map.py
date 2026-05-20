import struct
import random
import os  # 引入 os 库来处理路径

# ================= 配置参数 =================
# 🌟 1. PGM 和 YAML 的导出路径（例如：地图包的 maps 目录）
MAP_EXPORT_DIR = "./src/nav_slam/map"

# 🌟 2. World 文件的导出路径（例如：仿真包的 worlds 目录）
WORLD_EXPORT_DIR = "./src/gazebo_modele/world"


RESOLUTION = 0.05  # 0.05m/pixel
WIDTH_PIXELS = int(52.0 / RESOLUTION)   # 1040 像素
HEIGHT_PIXELS = int(52.0 / RESOLUTION)  # 1040 像素
ORIGIN_X = -26.0
ORIGIN_Y = -26.0

# 随机化参数配置
NUM_OBSTACLES = 120           # 随机障碍物的数量
MIN_SIZE = 0.5                # 障碍物最小尺寸 (米)
MAX_SIZE = 4.0                # 障碍物最大尺寸 (米)
SAFE_RADIUS = 4.0             # 中心点 (0,0) 的绝对安全半径

# 初始化障碍物列表
OBSTACLES = [
    # --- 0. 外围实体高墙 (2.5m高, 1m厚, 圈出 50x50m 净空) ---
    (0, 25.5, 52, 1), (0, -25.5, 52, 1), 
    (-25.5, 0, 1, 50), (25.5, 0, 1, 50)
]

# --- 生成随机障碍物 ---
print(f"正在生成 {NUM_OBSTACLES} 个随机障碍物...")
for _ in range(NUM_OBSTACLES):
    w = random.uniform(MIN_SIZE, MAX_SIZE)
    h = random.uniform(MIN_SIZE, MAX_SIZE)
    
    x = random.uniform(-24.5 + w/2, 24.5 - w/2)
    y = random.uniform(-24.5 + h/2, 24.5 - h/2)
    
    if abs(x) < SAFE_RADIUS and abs(y) < SAFE_RADIUS:
        continue
        
    OBSTACLES.append((x, y, w, h))

# ================= 1. 生成 Gazebo .world =================
def generate_world():
    world_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="voronoi_50m_bricks_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="voronoi_maze">
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
            z_height = 2.5 
            z_pose = z_height / 2.0
        else:
            z_height = random.uniform(0.5, 2.0)
            z_pose = z_height / 2.0
            
        color = "Gazebo/Bricks"

        links += f"""      <link name="obs_{i}">
        <pose>{x:.3f} {y:.3f} {z_pose:.3f} 0 0 0</pose>
        <collision name="col_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z_height:.3f}</size></box></geometry></collision>
        <visual name="vis_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z_height:.3f}</size></box></geometry><material><script><name>{color}</name></script></material></visual>
      </link>\n"""
    
    # 🌟 使用独立的 World 路径
    file_path = os.path.join(WORLD_EXPORT_DIR, 'voronoi_50m.world')
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

    # 🌟 使用独立的 Map 路径
    file_path = os.path.join(MAP_EXPORT_DIR, 'voronoi_50m.pgm')
    with open(file_path, 'wb') as f:
        f.write(f"P5\n{WIDTH_PIXELS} {HEIGHT_PIXELS}\n255\n".encode('ascii'))
        for r in range(HEIGHT_PIXELS):
            for c in range(WIDTH_PIXELS):
                f.write(struct.pack('B', grid[r][c]))
    print(f"✅ 已生成高精度对齐地图: {file_path}")

# ================= 3. 生成 YAML 配置文件 =================
def generate_yaml():
    # 因为 PGM 和 YAML 在同一个目录下，所以这里的 image 保持相对路径文件名即可，ROS 能自动识别
    yaml_content = f"""image: voronoi_50m.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
mode: trinary
"""
    # 🌟 使用独立的 Map 路径
    file_path = os.path.join(MAP_EXPORT_DIR, 'voronoi_50m.yaml')
    with open(file_path, 'w') as f:
        f.write(yaml_content)
    print(f"✅ 已生成地图配置文件: {file_path}")

if __name__ == "__main__":
    # 自动检查并创建不存在的文件夹
    for directory in [MAP_EXPORT_DIR, WORLD_EXPORT_DIR]:
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"📁 已自动创建目录: {directory}")

    print("正在构建 全砖墙版 测试环境...")
    generate_world()
    generate_pgm()
    generate_yaml()
    print("🎉 顺利完成！")