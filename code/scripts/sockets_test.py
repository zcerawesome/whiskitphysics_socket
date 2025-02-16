# -*- coding: utf-8 -*-

"""
TODO: copyright...
"""

import math
import numpy as np
import socket
import json
import time
import pandas

def collision_waiting():
    global received_data
    global change_trajectory
    global target_position
    global current_position
    global hit_position
    if target_position == None and received_data != None:
        for i in received_data['whiskers']:
            if received_data['whiskers'][i]['Collision']:
                target_position = [float(x) for x in received_data['whiskers'][i]['position']]
                hit_position = [float(x) for x in target_position]
                # print(type(target_position))
                for j in range(3):
                    target_position[j] = current_position[j] - (target_position[j] - current_position[j])/4
                change_trajectory = True
                break

def dynamic_nose_touch(json_data):
    global change_trajectory
    global target_position
    global current_position
    global current_trajectory
    global current_angle, target_angle
    if target_position == None:
        return
    acceleration = 20
    looking_speed = 15
    total_distance = abs(json_data['Position'][0] - target_position[0]) + abs(json_data['Position'][1]- target_position[1]) + abs(json_data['Position'][2]- target_position[2])
    x = abs(json_data['Position'][0] - target_position[0])
    y = abs(json_data['Position'][1] - target_position[1])
    z = abs(json_data['Position'][2] - target_position[2])
    distance = math.sqrt(x**2 + y**2 + z**2)
    # print(distance)
    if distance < 4.8 and target_position != hit_position:
        target_position = [x for x in hit_position]
    nose_pos_distance = math.sqrt((nose_position[0] - target_position[0]) ** 2 + (nose_position[1] - target_position[1]) ** 2)
    # print('Nose Pos', end=': ')
    # for i in nose_position:
    #     print(i, end = ' ')
    # print()
    # print('Target Pos', end=': ')
    # for i in target_position:
    #     print(i, end = ' ')
    # print()
    x_y_target_position = math.sqrt((target_position[0] - current_position[0]) ** 2 + (target_position[1]- current_position[1]) ** 2)
    print(x_y_target_position)
    if target_position == hit_position and abs(x_y_target_position - nose_pos_distance) < .4:
        return {'x_velocity': 0, 
            'y_velocity': 0, 
            'z_velocity': 0,
            'x_rotational_velocity': 0, 
            'y_rotational_velocity': 0, 
            'z_rotational_velocity': 0}
    new_trajectory = []
    for i in range(3):
        new_trajectory.append((-acceleration) * (json_data['Position'][i] - target_position[i]) / distance)
    if target_angle == None:
        target_angle = [x for x in current_angle]
        target_angle[2] = current_angle[2] - math.atan2(hit_position[0]-current_position[0], hit_position[1]-current_position[1])
    
    z_angle_velocity = -looking_speed if current_angle[2] > target_angle[2] else looking_speed
    if abs(current_angle[2] - target_angle[2]) <= 0.02:
        z_angle_velocity = 0
    # print(current_angle[2])
    return {'x_velocity': new_trajectory[0] + current_trajectory['x_velocity'], 
            'y_velocity': new_trajectory[1] + current_trajectory['y_velocity'], 
            'z_velocity': new_trajectory[2] + current_trajectory['z_velocity'],
            'x_rotational_velocity': 0, 
            'y_rotational_velocity': 0, 
            'z_rotational_velocity': z_angle_velocity}


def read_csv(index):
    rathead_trajectory_dir = '../data/rathead_trajectory_sample.csv'
    df = pandas.read_csv(rathead_trajectory_dir, header=None)
    data = df.iloc[index]
    x_velocity = data[3]
    y_velocity = data[4]
    z_velocity = data[5]
    x_rotational_velocity = data[6]
    y_rotational_velocity = data[7]
    z_rotational_velocity = data[8]
    return {'x_velocity': x_velocity, 
            'y_velocity': y_velocity, 
            'z_velocity': z_velocity,
            'x_rotational_velocity': x_rotational_velocity, 
            'y_rotational_velocity': y_rotational_velocity, 
            'z_rotational_velocity': z_rotational_velocity}
    
def average_force(json_data):
    if json_data == None:
        return None
    average_force = 0
    for i in json_data['whiskers']:
        average_force += np.linalg.norm(np.array(json_data['whiskers'][i]['force']))
    average_force /= len(json_data['whiskers'])
    return average_force

received_data = None
force = 0
change_trajectory = False
current_position = None
nose_position = [0,0,0]
current_trajectory =  {'x_velocity': 0, 
            'y_velocity': 0, 
            'z_velocity': 0,
            'x_rotational_velocity': 0, 
            'y_rotational_velocity': 0, 
            'z_rotational_velocity': 0}
target_position = None
hit_position = []
current_angle = None
target_angle = None


def handle_client(client_socket, index):
    global received_data
    global current_position
    global force
    global change_trajectory
    global target_position
    global current_trajectory
    global nose_position
    global current_angle, target_angle

    # threshold = .005
    # send a packet
    response_data = {'message' : 'JSON data received'}
    # norm_force  = average_force(received_data)
    if received_data != None:
        current_position = [float(x) for x in received_data['Position']]
        current_angle = [x for x in received_data['Angle']]

        for i in current_position:
            print(i, end=' ')
        print()
        roll = current_angle[0]
        pitch = current_angle[1]
        yaw = current_angle[2]
        R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
        ])

        R_y = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0,             1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])

        R_x = np.array([
            [1, 0,            0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch),  np.cos(pitch)]
        ])

        R = R_z @ R_y @ R_x
        A = np.array([0, 8.3, 2.9872])
        A_rotated = R @ A
        B = np.array([x for x in current_position])

        A_world = B + A_rotated
        for i in range(3):
            nose_position[i] = A_world[i]
    
                                                    
    if not change_trajectory:
        current_trajectory = read_csv(index)
        collision_waiting()
    else:
        current_trajectory = dynamic_nose_touch(received_data)

    response_data.update(current_trajectory)
    response_json = json.dumps(response_data)
    client_socket.send(response_json.encode('utf-8'))

    # receive a packet
    data = client_socket.recv(20048)
    json_data = None
    if data:
        json_data = json.loads(data.decode('utf-8'))
        received_data = json_data
        print("Received JSON data:")
    #client_socket.close()


def main():
    host = '127.0.0.1'
    port = 12346
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print("Server listening on", host, "port", port)
    index = 0
    while True:
        client_socket, client_address = server_socket.accept()
        print("Connected by", client_address)
        while True:
            handle_client(client_socket, index)
            index += 1

    server_socket.close()
    
if __name__ == "__main__":
    main()
