import sys
sys.path.append('PythonAPI')
import numpy as np
import math
import time
import threading
# import cv2
import matplotlib.pylab as plt
import random
import os
from datetime import date



try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',10999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to Vrep')

# get the handles of arm joints
armjoint_handle = []
for i in range(6):
    err_code, current_armjoint_handle = sim.simxGetObjectHandle(clientID,"UR5_joint" + str(i+1), sim.simx_opmode_blocking)
    armjoint_handle.append(current_armjoint_handle)
    sim.simxSetObjectIntParameter(clientID, current_armjoint_handle, 2000, 1, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, current_armjoint_handle, 2001, 1, sim.simx_opmode_oneshot)

# get the handle of hand
err_code, hand_handle = sim.simxGetObjectHandle(clientID,"BarrettHand", sim.simx_opmode_blocking)
print(hand_handle)
# get the handles of hand joints
handjoint_handle = []
err_code, finger1_base_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointB_1", sim.simx_opmode_blocking)
handjoint_handle.append(finger1_base_handle)
err_code, finger1_coupled_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointC_1", sim.simx_opmode_blocking)
handjoint_handle.append(finger1_coupled_handle)  
err_code, finger0_rotate_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointA_0", sim.simx_opmode_blocking)
handjoint_handle.append(finger0_rotate_handle)
err_code, finger0_base_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointB_0", sim.simx_opmode_blocking)
handjoint_handle.append(finger0_base_handle)
err_code, finger0_coupled_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointC_0", sim.simx_opmode_blocking)
handjoint_handle.append(finger0_coupled_handle) 
err_code, finger2_rotate_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointA_2", sim.simx_opmode_blocking)
handjoint_handle.append(finger2_rotate_handle)
err_code, finger2_base_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointB_2", sim.simx_opmode_blocking)
handjoint_handle.append(finger2_base_handle)
err_code, finger2_coupled_handle = sim.simxGetObjectHandle(clientID,"BarrettHand_jointC_2", sim.simx_opmode_blocking)
handjoint_handle.append(finger2_coupled_handle) 
for i in range(8):
    sim.simxSetObjectIntParameter(clientID, handjoint_handle[i], 2000, 1, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, handjoint_handle[i], 2001, 1, sim.simx_opmode_oneshot)

# get the handle of the tactile sensors
tactile_sensor_handle = []
# print(palm_tactile_handle)
for i in range(24):
    err_code, handle = sim.simxGetObjectHandle(clientID,"BarrettHand_handSensor" + str(i), sim.simx_opmode_blocking)
    tactile_sensor_handle.append(handle)

for i in range(100):
    err_code, handle = sim.simxGetObjectHandle(clientID,"BarrettHand_fingerTipSensor0_" + str(i), sim.simx_opmode_blocking)
    tactile_sensor_handle.append(handle)

for i in range(100):
    err_code, handle = sim.simxGetObjectHandle(clientID,"BarrettHand_fingerTipSensor1_" + str(i), sim.simx_opmode_blocking)
    tactile_sensor_handle.append(handle)

for i in range(100):
    err_code, handle = sim.simxGetObjectHandle(clientID,"BarrettHand_fingerTipSensor2_" + str(i), sim.simx_opmode_blocking)
    tactile_sensor_handle.append(handle)

# get the handle of balls
ball_handle = []
ball_handle.append(sim.simxGetObjectHandle(clientID,"Sphere", sim.simx_opmode_blocking)[1])
for i in range(299):
    err_code, handle = sim.simxGetObjectHandle(clientID,"Sphere" + str(i), sim.simx_opmode_blocking)
    ball_handle.append(handle)

# function to control the arm pose, order joint1 to joint 6
def set_armpose(armjoint_angle):
    sim.simxPauseCommunication(clientID,True)
    for i in range(6):
        armjoint_angle[i] = round(armjoint_angle[i] / 180.0 * math.pi, 3)
        # print(armjoint_handle[i])
        sim.simxSetJointTargetPosition(clientID, armjoint_handle[i], armjoint_angle[i], sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID,False)
    time.sleep(6)

# function to control the hand pose, order spread, finger0(fixed finger), finger1, finger2, finger0_couple, finger1_couple, finger2_couple.
def set_handpose(handjoint_angle):
    sim.simxPauseCommunication(clientID,True)
    input_handjoint_angle = []
    for i in range(7):
        input_handjoint_angle.append(round(handjoint_angle[i] / 180.0 * math.pi, 3)) 
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[2], input_handjoint_angle[0] / 2.0 - 1.57, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[5], input_handjoint_angle[0] / 2.0 - 1.57, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[0], input_handjoint_angle[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[3], input_handjoint_angle[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[6], input_handjoint_angle[3], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[1], input_handjoint_angle[4] + 0.733, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[4], input_handjoint_angle[5] + 0.733, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, handjoint_handle[7], input_handjoint_angle[6] + 0.733, sim.simx_opmode_oneshot)

    sim.simxPauseCommunication(clientID,False)
    time.sleep(0.5)



# hand initialization
set_handpose([120, 0, 0, 0, 0, 0, 0])
while ((sim.simxGetJointPosition(clientID, handjoint_handle[2], sim.simx_opmode_buffer)[1] + 1.57) * 2 * 180 / math.pi - 120 > 5
        and sim.simxGetJointPosition(clientID, handjoint_handle[0], sim.simx_opmode_buffer)[1]* 180 / math.pi > 5
        and sim.simxGetJointPosition(clientID, handjoint_handle[3], sim.simx_opmode_buffer)[1]* 180 / math.pi > 5
        and sim.simxGetJointPosition(clientID, handjoint_handle[6], sim.simx_opmode_buffer)[1]* 180 / math.pi > 5):
    time.sleep(0.1)
print('Hand initialization done!')

# check ball height
time.sleep(1.0)
for i in range(300):
    err_code, position = sim.simxGetObjectPosition(clientID, ball_handle[i], -1, sim.simx_opmode_buffer)
    if position[2] > 0.65:
        sim.simxSetObjectPosition(clientID, ball_handle[i], -1, [position[0], position[1], 0.5], sim.simx_opmode_oneshot)



armpose_initialization = [0, 0, 0, 0, -90, 0]
armpose_preparation = [0, 0, 60, 30, -90, 0]
armpose_grasp = [0, 0, 90, 0, -90, 0]
armpose_lift_little = [0, 0, 83, 7, -90, 0]
armpose_lift = [0, 0, 50, 40, -90, 0]
armpose_drop = [47, 40, 10, 40, -90, 0]
handpose_release = [30, 60, 60, 60, 20, 20, 20]

# Please specify the handpose in radient
handpose_grasp = ([0, 1.52,	1.52, 1.83,	0.888, 0.783, 1.03])
set_armpose([0, 0, 60, 30, -90, 0])
time.sleep(2)
target_pose = handpose_grasp

# Finger will move randomly until reach to torque threshold or maximum position
# the velocity set of finger, 70% move forward, 20% stay the same, 10% move backward
step_set = [3, 3, 3, 3, 3, 3, 3, 0, 0, -3]
termination_signal = 0
finger0_base_termination = 0
finger1_base_termination = 0
finger2_base_termination = 0
finger0_couple_termination = 0
finger1_couple_termination = 0
finger2_couple_termination = 0
base_threshold_set = [-0.8, -0.9, -1, -1.1, -1.2]
base_threshold = base_threshold_set[random.randint(0, 4)]
coupled_threshold_set = [-0.1, -0.2, -0.3]
coupled_threshold = coupled_threshold_set[random.randint(0, 2)]
# step control of the movement
time_list = []
while(termination_signal == 0):   
    start_time = time.time()
    # the step size for joint0
    if finger0_base_termination == 1:
        random_0 = 7
        if finger0_couple_termination == 1:
            random_00 = 7
        else:
            random_00 = 0
    else:
        random_0 = random.randint(0, 9)
        if finger0_couple_termination == 1:
            random_00 = 7
        else:
            random_00 = random_0 
    # the step size for joint1
    if finger1_base_termination == 1:
        random_1 = 7
        if finger1_couple_termination == 1:
            random_11 = 7
        else:
            random_11 = 0
    else:
        random_1 = random.randint(0, 9)
        if finger1_couple_termination == 1:
            random_11 = 7
        else:
            random_11 = random_1
    # the step size for joint2
    if finger2_base_termination == 1:
        random_2 = 7
        if finger2_couple_termination == 1:
            random_22 = 7
        else:
            random_22 = 0
    else:
        random_2 = random.randint(0, 9)
        if finger2_couple_termination == 1:
            random_22 = 7
        else:
            random_22 = random_2
    step_size = [0, step_set[random_0], step_set[random_1], step_set[random_2], step_set[random_00]/3, step_set[random_11]/3, step_set[random_22]/3]
    for j in range(7):
        target_pose[j] += step_size[j]
    set_handpose(target_pose)
    if count > 5:
        if sim.simxGetJointForce(clientID, handjoint_handle[0], sim.simx_opmode_buffer)[1] < base_threshold:
            step_size[2] = 0
            finger1_base_termination = 1
        if sim.simxGetJointForce(clientID, handjoint_handle[3], sim.simx_opmode_buffer)[1] < base_threshold:
            step_size[1] = 0
            finger0_base_termination = 1
        if sim.simxGetJointForce(clientID, handjoint_handle[6], sim.simx_opmode_buffer)[1] < base_threshold:
            step_size[3] = 0
            finger2_base_termination = 1
        if sim.simxGetJointForce(clientID, handjoint_handle[1], sim.simx_opmode_buffer)[1] < coupled_threshold and step_size[2] == 0:
            step_size[5] = 0
            finger1_couple_termination = 1
        if sim.simxGetJointForce(clientID, handjoint_handle[4], sim.simx_opmode_buffer)[1] < coupled_threshold and step_size[1] == 0:
            step_size[4] = 0
            finger0_couple_termination = 1
        if sim.simxGetJointForce(clientID, handjoint_handle[7], sim.simx_opmode_buffer)[1] < coupled_threshold and step_size[3] == 0:
            step_size[6] = 0
            finger2_couple_termination = 1
        print(step_size)
        if sum([abs(number) for number in step_size]) < 2:
            count_terminate += 1
        if count_terminate > 4:
            termination_signal = 1
        if count > 50:
            termination_signal = 1
        count += 1
        end_time = time.time()
        time_list.append(end_time - start_time)

    print('average time of control loop is: ' + str(sum(time_list) / len(time_list)))

# Lift the arm out of the pile
set_armpose(armpose_lift)
time.sleep(5)
set_armpose(armpose_drop)
time.sleep(5.0)
set_handpose(handpose_release)
time.sleep(5.0)
set_armpose(armpose_lift)
time.sleep(5.0)

print ('Program ended')

