import struct
import random

# ================= 配置参数 =================
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
            # 边界墙保持 2.5m 高
            z_height = 2.5 
            z_pose = z_height / 2.0
        else:
            # 内部随机障碍物高度错落有致
            z_height = random.uniform(0.5, 2.0)
            z_pose = z_height / 2.0
            
        # 🌟 核心修改：所有障碍物统一使用红砖材质！
        color = "Gazebo/Bricks"

        links += f"""      <link name="obs_{i}">
        <pose>{x:.3f} {y:.3f} {z_pose:.3f} 0 0 0</pose>
        <collision name="col_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z_height:.3f}</size></box></geometry></collision>
        <visual name="vis_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z_height:.3f}</size></box></geometry><material><script><name>{color}</name></script></material></visual>
      </link>\n"""
    
    with open('voronoi_50m.world', 'w') as f:
        f.write(world_header + links + world_footer)
    print("✅ 已生成 全红砖材质 Gazebo 世界文件: voronoi_50m.world")

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

    with open('voronoi_50m.pgm', 'wb') as f:
        f.write(f"P5\n{WIDTH_PIXELS} {HEIGHT_PIXELS}\n255\n".encode('ascii'))
        for r in range(HEIGHT_PIXELS):
            for c in range(WIDTH_PIXELS):
                f.write(struct.pack('B', grid[r][c]))
    print("✅ 已生成高精度对齐地图: voronoi_50m.pgm")

# ================= 3. 生成 YAML 配置文件 =================
def generate_yaml():
    yaml_content = f"""image: voronoi_50m.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
mode: trinary
"""
    with open('voronoi_50m.yaml', 'w') as f:
        f.write(yaml_content)
    print("✅ 已生成地图配置文件: voronoi_50m.yaml")

if __name__ == "__main__":
    print("正在构建 全砖墙版 测试环境...")
    generate_world()
    generate_pgm()
    generate_yaml()
    print("🎉 顺利完成！去 Gazebo 里看看你的红砖迷宫吧！")