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
clientID=sim.simxStart('127.0.0.1',31000,True,True,5000,5) # Connect to CoppeliaSim
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

# get the handle of balls
ball_handle = []
for i in range(30):
    err_code, handle = sim.simxGetObjectHandle(clientID,"Sphere" + str(i), sim.simx_opmode_blocking)
    ball_handle.append(handle)

# function to control the arm pose, order joint1 to joint 6
def set_armpose(armjoint_angle):
    sim.simxPauseCommunication(clientID,True)
    input_armjoint_angle = np.zeros(6)
    for i in range(6):
        input_armjoint_angle[i] = round(armjoint_angle[i] / 180.0 * math.pi, 3)
        # print(armjoint_handle[i])
        sim.simxSetJointTargetPosition(clientID, armjoint_handle[i], input_armjoint_angle[i], sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID,False)
    time.sleep(0.5)

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

# read arm pose
def readarmpose():
    armpose_reading = np.zeros(6)
    for i in range(6):
        err, position = sim.simxGetJointPosition(clientID, armjoint_handle[i], sim.simx_opmode_blocking)
        armpose_reading[i] = position
    armpose_reading = armpose_reading / math.pi * 180.0
    print("Current arm pose is: " + str(armpose_reading))


terminate = 0
count = 0

while(terminate == 0):
    selection = int(input("Please select which action to take (0 or 1):  "))
    if selection == 0:

        data_path = os.path.join('/home/rpal/tianze/RA Letter Video/40mm_sphere', str(667), 'grasping_result.csv')
        data = np.loadtxt(data_path, delimiter = ',')
        # set the handpose
        length = np.shape(data)[0]
        handpose_to_use = np.array([(data[length - 3] + data[length - 6]) / 2, data[length - 8], data[length - 5], data[length - 2], data[length - 7], data[length - 4], data[length - 1]])
        handpose_to_use = handpose_to_use / math.pi * 180

        # set_handpose(handpose_to_use)
        offset_x = 0.0796 - data[4]
        offset_y = 0.0559 - data[5]
        offset_z = 0.594 - data[6]

        # readarmpose()
        armpose_lift = [0, 0, 50, 40, -90, 0]
        armpose_grasp = [0, 0, 90, 0, -90, 0]
        armpose_drop = [47, 40, 10, 40, -90, 0]
        handpose_release = [30, 60, 60, 60, 20, 20, 20]

        # trial 1, 53
        # handpose_to_use_temp = handpose_to_use[2]
        # handpose_to_use[2] = handpose_to_use[3]
        # handpose_to_use[3] = handpose_to_use_temp
        # handpose_to_use_temp = handpose_to_use[5]
        # handpose_to_use[5] = handpose_to_use[6]
        # handpose_to_use[6] = handpose_to_use_temp

        handpose_to_use[3] = 120
        handpose_to_use[2] = 110
        handpose_to_use[1] = 95
        # handpose_to_use[5] = 43

        print(handpose_to_use)
        handpose_pre = np.zeros(7)
        handpose_pre[0] = handpose_to_use[0]
        for i in range(1, 4):
            handpose_pre[i] = handpose_to_use[i] - 45
            handpose_pre[i + 3] = handpose_to_use[i + 3] - 15

        set_handpose(handpose_pre)
        time.sleep(5.0)
        set_armpose(armpose_grasp)
        time.sleep(5.0)
        print(handpose_to_use)
        # handpose_to_use_flip = np.zeros(7)
        # for i in range(7):
        #     handpose_to_use_flip[i] = handpose_to_use[i]
        # handpose_to_use_flip[2] = handpose_to_use[3]
        # handpose_to_use_flip[3] = handpose_to_use[2]
        # handpose_to_use_flip[5] = handpose_to_use[6]
        # handpose_to_use_flip[6] = handpose_to_use[5]
        for iteration in range(np.random.randint(2, 5)):
            handpose_middle = np.zeros(7)
            for i in range (7):
                handpose_middle[i] = handpose_to_use[i] - np.random.randint(3, 6)
            set_handpose(handpose_middle)
            time.sleep(1.0)
        set_handpose(handpose_to_use)
        time.sleep(1.0)

        sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot)
        time.sleep(1.0)
        # set the object pose
        for iteration_ball in range(int(data[3])):
            ball_position = np.array([data[10 + iteration_ball * 3 + int(data[3]) * 3], data[11 + iteration_ball * 3 + int(data[3]) * 3], data[12 + iteration_ball * 3 + int(data[3]) * 3]])
            ball_position[0] += offset_x
            ball_position[1] += offset_y
            ball_position[2] += offset_z
            sim.simxSetObjectPosition(clientID, ball_handle[iteration_ball + count], -1, ball_position, sim.simx_opmode_oneshot)    

        # input('haha')
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        time.sleep(1.0)

        set_armpose(armpose_lift)
        time.sleep(5.0)
        set_armpose(armpose_drop)
        time.sleep(5.0)
        set_handpose(handpose_release)
        time.sleep(5.0)
        set_armpose(armpose_lift)
        time.sleep(5.0)


    if selection == 1:
        data_path = os.path.join('/home/rpal/tianze/RA Letter Video/40mm_sphere', str(716), 'grasping_result.csv')
        data = np.loadtxt(data_path, delimiter = ',')
        # set the handpose
        length = np.shape(data)[0]
        handpose_to_use = np.array([(data[length - 3] + data[length - 6]) / 2, data[length - 8], data[length - 5], data[length - 2], data[length - 7], data[length - 4], data[length - 1]])
        handpose_to_use = handpose_to_use / math.pi * 180

        # set_handpose(handpose_to_use)
        offset_x = 0.0796 - data[4]
        offset_y = 0.0559 - data[5] - 0.02
        offset_z = 0.594 - data[6]

        # readarmpose()
        armpose_lift = [0, 0, 50, 40, -90, 0]
        armpose_grasp = [0, 0, 90, 0, -90, 0]
        armpose_drop = [47, 40, 10, 40, -90, 0]
        handpose_release = [30, 60, 60, 60, 20, 20, 20]

        # trial 1, 53
        handpose_to_use_temp = handpose_to_use[2]
        handpose_to_use[2] = handpose_to_use[3]
        handpose_to_use[3] = handpose_to_use_temp
        handpose_to_use_temp = handpose_to_use[5]
        handpose_to_use[5] = handpose_to_use[6]
        handpose_to_use[6] = handpose_to_use_temp

        handpose_to_use[3] = 120
        handpose_to_use[2] = 125
        # handpose_to_use[1] = 95
        # handpose_to_use[5] = 43

        print(handpose_to_use)
        handpose_pre = np.zeros(7)
        handpose_pre[0] = handpose_to_use[0]
        for i in range(1, 4):
            handpose_pre[i] = handpose_to_use[i] - 45
            handpose_pre[i + 3] = handpose_to_use[i + 3] - 15

        set_handpose(handpose_pre)
        time.sleep(5.0)
        set_armpose(armpose_grasp)
        time.sleep(5.0)
        print(handpose_to_use)
        # handpose_to_use_flip = np.zeros(7)
        # for i in range(7):
        #     handpose_to_use_flip[i] = handpose_to_use[i]
        # handpose_to_use_flip[2] = handpose_to_use[3]
        # handpose_to_use_flip[3] = handpose_to_use[2]
        # handpose_to_use_flip[5] = handpose_to_use[6]
        # handpose_to_use_flip[6] = handpose_to_use[5]
        for iteration in range(np.random.randint(2, 5)):
            handpose_middle = np.zeros(7)
            for i in range (7):
                handpose_middle[i] = handpose_to_use[i] - np.random.randint(3, 6)
            set_handpose(handpose_middle)
            time.sleep(1.0)
        set_handpose(handpose_to_use)
        time.sleep(1.0)

        sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot)
        time.sleep(1.0)
        # set the object pose
        for iteration_ball in range(int(data[3])):
            ball_position = np.array([data[10 + iteration_ball * 3 + int(data[3]) * 3], data[11 + iteration_ball * 3 + int(data[3]) * 3], data[12 + iteration_ball * 3 + int(data[3]) * 3]])
            ball_position[0] += offset_x
            ball_position[1] += offset_y
            ball_position[2] += offset_z
            sim.simxSetObjectPosition(clientID, ball_handle[iteration_ball + count], -1, ball_position, sim.simx_opmode_oneshot)    

        # input('haha')
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        time.sleep(1.0)

        set_armpose(armpose_lift)
        time.sleep(5.0)
        set_armpose(armpose_drop)
        time.sleep(5.0)
        set_handpose(handpose_release)
        time.sleep(5.0)
        set_armpose(armpose_lift)
        time.sleep(5.0)



    terminate = int(input('wheteher to terminate the progeam (Y=1, N=0)?  '))
    count += 5

print ('Program ended')

