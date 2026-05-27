import struct
import os

# ================= 配置参数 =================
MAP_EXPORT_DIR = "./src/nav_slam/map"
WORLD_EXPORT_DIR = "./src/gazebo_modele/world"

RESOLUTION = 0.05  # 0.05m/pixel
# 设定为 30m x 20m 的办公楼层
WIDTH_PIXELS = int(30.0 / RESOLUTION)   # 600 像素
HEIGHT_PIXELS = int(20.0 / RESOLUTION)  # 400 像素
ORIGIN_X = -15.0
ORIGIN_Y = -10.0

# 数据格式: (x, y, w, h, z_height, material)
# 材质说明: Bricks(墙壁), Wood(木制家具), Grey(金属/杂物), Blue(软体/沙发)

WALLS = [
    # 1. 外部主承重墙 (2.5m高)
    (0, 9.9, 30, 0.2, 2.5, "Gazebo/Bricks"),
    (0, -9.9, 30, 0.2, 2.5, "Gazebo/Bricks"),
    (-14.9, 0, 0.2, 20, 2.5, "Gazebo/Bricks"),
    (14.9, 0, 0.2, 20, 2.5, "Gazebo/Bricks"),
    
    # 2. 走廊上半部分横墙 (留出3个门洞)
    (-12.5, 1.5, 4.8, 0.2, 2.5, "Gazebo/White"), # 左段
    (-4.5, 1.5, 8.8, 0.2, 2.5, "Gazebo/White"),  # 中段
    (6.0, 1.5, 8.8, 0.2, 2.5, "Gazebo/White"),   # 右段
    (13.3, 1.5, 3.4, 0.2, 2.5, "Gazebo/White"),  # 极右段
    
    # 3. 走廊下半部分横墙 (留出2个门洞)
    (-10.0, -1.5, 9.8, 0.2, 2.5, "Gazebo/White"),
    (2.5, -1.5, 12.8, 0.2, 2.5, "Gazebo/White"),
    (12.5, -1.5, 4.8, 0.2, 2.5, "Gazebo/White"),
    
    # 4. 房间纵向隔断墙
    (-5.0, 5.7, 0.2, 8.2, 2.5, "Gazebo/White"), # 左/中 隔断
    (5.0, 5.7, 0.2, 8.2, 2.5, "Gazebo/White"),  # 中/右 隔断
    (0.0, -5.7, 0.2, 8.2, 2.5, "Gazebo/White"), # 下方两房间隔断
]

FURNITURE = [
    # --- 房间 1: 左上会议室 ---
    (-10.0, 5.7, 5.0, 1.8, 0.8, "Gazebo/Wood"), # 巨型会议桌
    (-10.0, 7.5, 4.0, 0.4, 0.5, "Gazebo/Blue"), # 北侧椅子排
    (-10.0, 3.9, 4.0, 0.4, 0.5, "Gazebo/Blue"), # 南侧椅子排
    
    # --- 房间 2: 中上主管办公室 ---
    (0.0, 8.0, 2.5, 1.0, 0.8, "Gazebo/Wood"),   # 办公大桌
    (3.5, 4.5, 1.0, 2.5, 0.6, "Gazebo/Blue"),   # 侧边会客沙发
    
    # --- 房间 3: 右上员工隔间区 (密集特征区) ---
    (8.0, 7.5, 1.6, 1.6, 1.2, "Gazebo/Wood"),   # 工位 1
    (12.0, 7.5, 1.6, 1.6, 1.2, "Gazebo/Wood"),  # 工位 2
    (8.0, 4.0, 1.6, 1.6, 1.2, "Gazebo/Wood"),   # 工位 3
    (12.0, 4.0, 1.6, 1.6, 1.2, "Gazebo/Wood"),  # 工位 4
    (10.0, 5.7, 6.0, 0.2, 1.5, "Gazebo/White"), # 挡板
    
    # --- 房间 4: 左下机器人联调实验室 ---
    (-8.0, -8.5, 6.0, 1.0, 1.0, "Gazebo/Wood"), # 靠墙长条工作台
    (-12.0, -5.0, 3.0, 0.2, 0.4, "Gazebo/White"), # 矮墙围栏(测试区)上沿
    (-10.5, -6.5, 0.2, 3.0, 0.4, "Gazebo/White"), # 矮墙围栏(测试区)右沿
    
    # --- 房间 5: 右下茶水间与杂物区 ---
    (13.0, -8.0, 1.2, 1.0, 2.0, "Gazebo/Grey"), # 自动售货机 1
    (11.5, -8.0, 1.2, 1.0, 2.0, "Gazebo/Grey"), # 自动售货机 2
    (6.0, -4.0, 1.2, 1.2, 0.8, "Gazebo/Wood"),  # 圆桌(用方块近似)
    (9.0, -3.0, 1.2, 1.2, 0.8, "Gazebo/Wood"),  # 圆桌
]

CLUTTER = [
    # --- 走廊杂物 (故意制造S型避障路线) ---
    (-6.0, 0.6, 1.5, 0.8, 1.0, "Gazebo/Wood"),  # 走廊上半部分堆放的木箱
    (-1.5, -0.6, 1.2, 1.0, 1.2, "Gazebo/Grey"), # 走廊下半部分堆放的铁皮箱
    (7.0, 0.8, 0.5, 0.5, 0.8, "Gazebo/Blue"),   # 垃圾桶 1
    (12.0, -0.8, 0.5, 0.5, 0.8, "Gazebo/Blue"), # 垃圾桶 2
    (-13.5, 0.0, 0.6, 0.6, 1.5, "Gazebo/White"),# 走廊尽头的饮水机
]

ALL_ENTITIES = WALLS + FURNITURE + CLUTTER

# ================= 1. 生成 Gazebo .world =================
def generate_world():
    world_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="office_30x20_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="office_building">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
"""
    world_footer = """    </model>
  </world>
</sdf>
"""
    links = ""
    for i, (x, y, w, h, z, color) in enumerate(ALL_ENTITIES):
        z_pose = z / 2.0
        links += f"""      <link name="obj_{i}">
        <pose>{x:.3f} {y:.3f} {z_pose:.3f} 0 0 0</pose>
        <collision name="col_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z:.3f}</size></box></geometry></collision>
        <visual name="vis_{i}"><geometry><box><size>{w:.3f} {h:.3f} {z:.3f}</size></box></geometry><material><script><name>{color}</name></script></material></visual>
      </link>\n"""
    
    file_path = os.path.join(WORLD_EXPORT_DIR, 'office_30x20.world')
    with open(file_path, 'w') as f:
        f.write(world_header + links + world_footer)
    print(f"✅ 已生成 Gazebo 世界文件: {file_path}")

# ================= 2. 生成 PGM 栅格地图 =================
def generate_pgm():
    grid = [[255 for _ in range(WIDTH_PIXELS)] for _ in range(HEIGHT_PIXELS)]
    
    for (x_center, y_center, w, h, _, _) in ALL_ENTITIES:
        for r in range(HEIGHT_PIXELS):
            for c in range(WIDTH_PIXELS):
                x = ORIGIN_X + c * RESOLUTION
                y = ORIGIN_Y + (HEIGHT_PIXELS - 1 - r) * RESOLUTION
                # 判断当前像素是否在实体的矩形范围内
                if abs(x - x_center) <= w/2.0 and abs(y - y_center) <= h/2.0:
                    grid[r][c] = 0

    file_path = os.path.join(MAP_EXPORT_DIR, 'office_30x20.pgm')
    with open(file_path, 'wb') as f:
        f.write(f"P5\n{WIDTH_PIXELS} {HEIGHT_PIXELS}\n255\n".encode('ascii'))
        for r in range(HEIGHT_PIXELS):
            for c in range(WIDTH_PIXELS):
                f.write(struct.pack('B', grid[r][c]))
    print(f"✅ 已生成高精度对齐地图: {file_path}")

# ================= 3. 生成 YAML 配置文件 =================
def generate_yaml():
    yaml_content = f"""image: office_30x20.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
mode: trinary
"""
    file_path = os.path.join(MAP_EXPORT_DIR, 'office_30x20.yaml')
    with open(file_path, 'w') as f:
        f.write(yaml_content)
    print(f"✅ 已生成地图配置文件: {file_path}")

if __name__ == "__main__":
    for directory in [MAP_EXPORT_DIR, WORLD_EXPORT_DIR]:
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"📁 已自动创建目录: {directory}")

    print("正在构建 办公楼层(30x20m) 仿真环境...")
    generate_world()
    generate_pgm()
    generate_yaml()
    print("🎉 顺利完成！你可以直接在 Launch 文件中加载 office_30x20.world")