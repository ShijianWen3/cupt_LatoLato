import pybullet as p
import pybullet_data
import yaml
import time
import math
from create_rope import create_rope
# Load parameters from YAML file

with open("/home/shijian/cupt/rope_control_pybullet-main/config/config.yaml", "r") as config_file:
    config = yaml.safe_load(config_file)

# Extract parameters from config
rope_params = config["rope"]
sim_params = config["simulation"]

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Set the additional search path for URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Reset simulation and set gravity
p.resetSimulation()
p.setGravity(*sim_params["gravity"])

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# Rope setup
rope_length = rope_params["length"]
num_segments = rope_params["num_segments"]
segment_length = rope_length / num_segments
segment_radius = rope_params["segment_radius"]
segment_mass = rope_params["segment_mass"]
rope_segments = []
pivot_height = 4

# Set a starting position for the first rope segment directly below but offset from the robot's end-effector
# start_pos = rope_params["start_position"]
# start_pos = [-rope_length/2,0,pivot_height]
start_pos_1 = [0,0,pivot_height-rope_length]
start_pos_2 = [0,0,0]
# Create rope segments
rope1 = create_rope(num_segments,start_pos_1, rope_params)
# rope2 = create_rope(num_segments, start_pos_2, rope_params)
start_point_1 = [-(rope_length-0.1), 0, 3.5]
start_point_2 = [(rope_length-0.1), 0, 3.5]
# 创建两个小球，分别在绳子的两端
ball_radius = 0.05
ball_mass = 5

ball1 = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
ball2 = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)

ball1_id = p.createMultiBody(
    baseMass=ball_mass,
    baseCollisionShapeIndex=ball1,
    # basePosition=[0, 0, pivot_height-rope_length]
    basePosition=[-rope_length, 0, pivot_height]

)

ball2_id = p.createMultiBody(
    baseMass=ball_mass,
    baseCollisionShapeIndex=ball2,
    basePosition=[rope_length , 0, pivot_height]
)


pivot_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    basePosition=[0, 0, pivot_height]
)

p1 = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    basePosition=start_point_1
)
p2 = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    basePosition=start_point_2
)





# Attach the rope to the robot's end-effector

p.createConstraint(
    pivot_id,
    -1,
    rope1[-1],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
)

c1 = p.createConstraint(
    p1,
    -1,
    rope1[0],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
)

rope2 = create_rope(num_segments,start_pos_1, rope_params)

p.createConstraint(
    pivot_id,
    -1,
    rope2[0],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
)
c2 = p.createConstraint(
    p2,
    -1,
    rope2[-1],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
)


p.createConstraint(
    ball1_id,
    -1,
    rope1[0],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [ball_radius, 0, -ball_radius],
    [0, 0, 0],
)

p.createConstraint(
    ball2_id,
    -1,
    rope2[-1],
    -1,
    p.JOINT_FIXED,
    [0, 0, 0],
    [-ball_radius, 0, -ball_radius],
    [0, 0, 0],
)


# Simulation loop without dynamic motion to test the initial configuration
t = 0
dt = sim_params["time_step"]
simulation_duration = sim_params["duration"]
# p.setGravity(*[0,0,0])

p.removeConstraint(c1)
p.removeConstraint(c2)




# 设置空气密度和物体参数
rho_air = 1.225  # 空气密度 (kg/m^3)
drag_coefficient = 0.47  # 阻力系数 (假设是一个球体)
object_area = math.pi * ball_radius**2  # 物体的迎风面积 (m^2)


start_time = time.time()
oscillation_amplitude = 0.5  # 振幅
oscillation_frequency = 0.5  # 振荡频率
while t < simulation_duration:
    t += dt


    # 获取物体的速度
    velocity, angular_velocity = p.getBaseVelocity(ball1_id)
    v = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)  # 物体的速度大小

    if v:
        # 计算空气阻力
        drag_force_magnitude = 0.5 * drag_coefficient * rho_air * object_area * v**2

        # 计算空气阻力的方向，通常是速度方向的反向
        drag_force = [-drag_force_magnitude * velocity[0] / v, 
                    -drag_force_magnitude * velocity[1] / v, 
                    -drag_force_magnitude * velocity[2] / v]
        

        # 应用空气阻力
        p.applyExternalForce(ball1_id, linkIndex=-1, forceObj=drag_force, posObj=[0, 0, 0], flags=p.WORLD_FRAME)

    # 获取物体的速度
    velocity, angular_velocity = p.getBaseVelocity(ball2_id)
    v = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)  # 物体的速度大小

    if v:
             # 计算空气阻力
        drag_force_magnitude = 0.5 * drag_coefficient * rho_air * object_area * v**2

        # 计算空气阻力的方向，通常是速度方向的反向
        drag_force = [-drag_force_magnitude * velocity[0] / v, 
                    -drag_force_magnitude * velocity[1] / v, 
                    -drag_force_magnitude * velocity[2] / v]

        # 应用空气阻力
        p.applyExternalForce(ball2_id, linkIndex=-1, forceObj=drag_force, posObj=[0, 0, 0], flags=p.WORLD_FRAME)

    
    
    # 计算支点的振荡位置
    current_time = time.time() - start_time
    oscillation = pivot_height+oscillation_amplitude * math.sin(2 * math.pi * oscillation_frequency * current_time)
    
    # 设置支点的位置
    p.resetBasePositionAndOrientation(pivot_id, [0, 0, oscillation], [0, 0, 0, 1])

    
    # Step the simulation
    p.stepSimulation()
    time.sleep(dt)

# Disconnect from simulation
p.disconnect()