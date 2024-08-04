import pybullet as p
import time
import math
import numpy as np
import random


############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-60, cameraPitch=271, cameraTargetPosition=[4, 4, 8])
useRealTimeSim = 0


p.setRealTimeSimulation(useRealTimeSim)  # either this

# load plane
track = p.loadURDF("data/plane/plane.urdf")
# load car
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0, 0, 0])
# load obstacles, in this projects, we used six cube as obstacles

def random_obstacles():
    np.random.seed()
    xy_position = [0, 0]
    xy_position_float = np.random.rand(2)
    x_poistion_range = np.random.randint(1, 10)
    y_poistion_range = np.random.randint(1, 10)

    xy_position[0] = xy_position_float[0]+x_poistion_range
    xy_position[1] = xy_position_float[1]+y_poistion_range

    z_position = np.random.rand(1)
    np.asarray(xy_position)
    position = np.append(xy_position, z_position)
    return position


# Total eight different cubes. The cube's position is changing for each time.
total_cubes_number = 10
data_path = 'data/'
cube_list = []
cube_name_list = ['cube_black',
                  'cube_green',
                  'cube']

for i in range(total_cubes_number):
    cube_list.append(random.choice(cube_name_list))

# Only two objects are loaded in stable positon.
# You need to design URDF files to load objects, otherwise the code will not execute.
cube_stable_position_1 = p.loadURDF('data/cube_black/marble_cube.urdf', (5, 5, 0.2))
cube_stable_position_2 = p.loadURDF('data/cube/marble_cube.urdf', (3, 3, 0.2))

# 10 objects are loaded in random positions.
cube_1_position = random_obstacles()
cube_1 = p.loadURDF('data/'+cube_list[0]+'/marble_cube.urdf', cube_1_position)

cube_2_position = random_obstacles()
cube_2 = p.loadURDF('data/'+cube_list[1]+'/marble_cube.urdf', cube_2_position)

cube_3_position = random_obstacles()
cube_3 = p.loadURDF('data/'+cube_list[2]+'/marble_cube.urdf', cube_3_position)

cube_4_position = random_obstacles()
cube_4 = p.loadURDF('data/'+cube_list[3]+'/marble_cube.urdf', cube_4_position)

cube_5_position = random_obstacles()
cube_5 = p.loadURDF('data/'+cube_list[4]+'/marble_cube.urdf', cube_5_position)

cube_6_position = random_obstacles()
cube_6 = p.loadURDF('data/'+cube_list[5]+'/marble_cube.urdf', cube_6_position)

cube_7_position = random_obstacles()
cube_7 = p.loadURDF('data/'+cube_list[6]+'/marble_cube.urdf', cube_7_position)

cube_8_position = random_obstacles()
cube_8 = p.loadURDF('data/'+cube_list[7]+'/marble_cube.urdf', cube_8_position)

cube_9_position = random_obstacles()
cube_9 = p.loadURDF('data/'+cube_list[8]+'/marble_cube.urdf', cube_9_position)

cube_10_position = random_obstacles()
cube_10 = p.loadURDF('data/'+cube_list[9]+'/marble_cube.urdf', cube_10_position)

for wheel in range(p.getNumJoints(car)):
    print("joint[", wheel, "]=", p.getJointInfo(car, wheel))
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, wheel)

wheels = [8, 15]
print("----------------")

# p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

hokuyo_joint = 4

#
# targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 50, 0)
# maxForceSlider = p.addUserDebugParameter("maxForce", 0, 50, 20)
# steeringSlider = p.addUserDebugParameter("steering", -1, 1, 0)

replaceLines = True
# numRays = 100
numRays = 100
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
rayLen = 4
rayStartLen = 0.25
for i in range(numRays):
    rayFrom.append([rayStartLen * math.sin(0.2 * 0.25 * 2. * math.pi + 0.38 * 2. * math.pi * float(i) / numRays),
                    rayStartLen * math.cos(-0.2 * 0.25 * 2. * math.pi + 0.38 * 2. * math.pi * float(i) / numRays), 0])
    rayTo.append([rayLen * math.sin(0.2 * 0.25 * 2. * math.pi + 0.38 * 2. * math.pi * float(i) / numRays),
                  rayLen * math.cos(0.2 * 0.25 * 2. * math.pi + 0.38 * 2. * math.pi * float(i) / numRays), 0])
    if replaceLines:
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)

def move_car(carPos, steering_angle):
    target_position = [11, 11]
    if steering_angle > 1:
        steering_angle = 1
    if steering_angle < -1:
        steering_angle = -1
    car_dist = np.sqrt(np.square(target_position[0] - carPos[0]) + np.square(target_position[1] - carPos[1]))
    if car_dist < 1:
        target_velocity = 0
    else:
        target_velocity = 25
    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steering_angle)
    for wheel in wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=20)

def find_car_path(carPos, carOrn, sensor_readings):
    target_position = [11, 11]
    car_euler = p.getEulerFromQuaternion(carOrn)
    hit_to_angle = []
    hit_to_fraction = []
    angle_from_final_to_car = np.arctan2((target_position[1] - carPos[1]), (target_position[0] - carPos[0])) - car_euler[2]
    angle_from_final_to_car = (angle_from_final_to_car + math.pi) % (2 * math.pi) - math.pi
    for index, sensor_reading in enumerate(sensor_readings):
        local_hit_To = [rayFrom[index][0] + sensor_reading[2] * (rayTo[index][0] - rayFrom[index][0]),
                      rayFrom[index][1] + sensor_reading[2] * (rayTo[index][1] - rayFrom[index][1]),
                      rayFrom[index][2] + sensor_reading[2] * (rayTo[index][2] - rayFrom[index][2])]
        hit_to_angle.append(np.arctan2((local_hit_To[1] - rayFrom[index][1]), (local_hit_To[0] - rayFrom[index][0])))
        hit_to_fraction.append(sensor_reading[2])
    minimum_angle_change = 2 * math.pi
    for index in range(len(hit_to_fraction)):
        angle_change = abs(hit_to_angle[index] - angle_from_final_to_car)
        if angle_change < minimum_angle_change:
            minimum_angle_change = angle_change
    obstacle_distance = 0.19
    if any(sensor_reading[2] < obstacle_distance for sensor_reading in sensor_readings):
        obstacle_angle = hit_to_angle[np.argmin([sensor_reading[2] for sensor_reading in sensor_readings])]
        if obstacle_angle > 0:
            return -np.radians(22)
        elif obstacle_angle < 0:
            return np.radians(19)
    return angle_from_final_to_car

lastLidarTime = time.time()
interval = 0
frame = 0
while True:
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    interval += 0.00277778
    if frame > 0:
        nowTime = time.time()
        nowLidarTime = time.time()
        # Lidar at 1000Hz
        if nowLidarTime - lastLidarTime > .001:
            numThreads = 0
            results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            carPos, carOrn = p.getBasePositionAndOrientation(car)
            for i in range(numRays):
                hitObjectUid = results[i][0]
                hitFraction = results[i][2]
                hitPosition = results[i][3]
                if hitFraction == 1.:
                    p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i],
                                       parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
                else:
                    localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                                  rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                                  rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                    p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor, replaceItemUniqueId=rayIds[i],
                                       parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            steering_angle = find_car_path(carPos, carOrn, results)
            move_car(carPos, steering_angle)
            lastLidarTime = nowLidarTime
    if interval > 1:
        frame += 1
        interval = 0
    p.stepSimulation()
