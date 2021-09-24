#!/usr/bin/env python3
from pymyolinux.core.myo import MyoDongle
import numpy as np
import matplotlib.pyplot as plt
# import torch
import os
# from torch import nn
# from torch.nn import init
from scipy.spatial.transform import Rotation as R

denominator = [0, 0, 0, 0, 0, 0, 0, 0]
total_emg_list = []
right_emg_list = []
x = list(range(1, 101))
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# count = 0

# 位置
position = [0, 0, 0]
velocity = [0, 0, 0]

# class simpleNet(nn.Module):
#     def __init__(self):
#         super(simpleNet, self).__init__()
#         self.layer1 = nn.Linear(8, 8)
#         self.layer2 = nn.Sigmoid()
#         self.layer3 = nn.Linear(8, 4)
#         self.layer5 = nn.Softmax(dim=1)
#
#     def forward(self, x):
#         x = self.layer1(x)
#         x = self.layer2(x)
#         x = self.layer3(x)
#         x = self.layer5(x)
#         return x

# def _init_parameters(self):
#     for m in self.modules():
#         init.normal_(m.weight, 0, 0.01)
#         init.constant_(m.bias, 0)
raw_velocity = [0, 0, 0]


def get_position(quat, accel):
    global position, velocity, raw_velocity
    raw_velocity += np.array(accel) / 200
    r = np.array(R.from_quat(quat).as_matrix())
    accel_world = np.dot(r, np.array(accel))

    # print('accel_world:',accel_world)
    velocity += accel_world
    # print('raw_velocity:', raw_velocity)
    position += velocity
    # print('position:',position)


def joint_event_handler1(emg_list, orient_w, orient_x, orient_y, orient_z, accel_1, accel_2, accel_3, gyro_1, gyro_2,
                         gyro_3, sample_num):
    # Accelerometer values are multipled by the following constant (and are in units of g)
    MYOHW_ACCELEROMETER_SCALE = 2048.0

    # Gyroscope values are multipled by the following constant (and are in units of deg/s)
    MYOHW_GYROSCOPE_SCALE = 16.0

    # Orientation values are multipled by the following constant (units of a unit quaternion)
    MYOHW_ORIENTATION_SCALE = 16384.0
    print("-------------------------------------------------------------------------------------------")
    orient_w = orient_w / MYOHW_ORIENTATION_SCALE
    orient_x = orient_x / MYOHW_ORIENTATION_SCALE
    orient_y = orient_y / MYOHW_ORIENTATION_SCALE
    orient_z = orient_z / MYOHW_ORIENTATION_SCALE
    accel_1 = accel_1 / MYOHW_ACCELEROMETER_SCALE
    accel_2 = accel_2 / MYOHW_ACCELEROMETER_SCALE
    accel_3 = accel_3 / MYOHW_ACCELEROMETER_SCALE
    gyro_1 = gyro_1 / MYOHW_GYROSCOPE_SCALE
    gyro_2 = gyro_2 / MYOHW_GYROSCOPE_SCALE
    gyro_3 = gyro_3 / MYOHW_GYROSCOPE_SCALE


    abs_emg_list = np.abs(emg_list)
    filter_emg = abs_emg_list
    # get_position([orient_w, orient_x, orient_y, orient_z], [accel_1, accel_2, accel_3])
    get_position([orient_w, orient_x, orient_y, orient_z], [gyro_1, gyro_2, gyro_3])
    global denominator
    if (len(total_emg_list) > 20):
        denominator -= total_emg_list[len(total_emg_list) - 21]
        filter_emg = denominator / 20
    denominator += abs_emg_list
    total_emg_list.append(abs_emg_list)
    right_emg_list.append(filter_emg)
    # print(denominator)
    # print(filter_emg)
    # global count
    # 识别emg信号
    # if count > 50:
    #     float_emg = np.float32(filter_emg)
    #     float_emg_list = []
    #     float_emg_list.append(float_emg)
    #     input = torch.tensor(float_emg_list).to(device)
    #     output = model(input)
    #     predict = torch.argmax(output)
    #     if predict == 0:
    #         print('normal')
    #     elif predict == 1:
    #         print('left')
    #     elif predict == 2:
    #         print('right')
    #     elif predict == 3:
    #         print('punch')
    #     count = 0
    # else:
    #     count += 1
    # print(len(right_emg_list))
    print('1')


def joint_event_handler2(emg_list, orient_w, orient_x, orient_y, orient_z, accel_1, accel_2, accel_3, gyro_1, gyro_2,
                         gyro_3, sample_num):
    # Accelerometer values are multipled by the following constant (and are in units of g)
    MYOHW_ACCELEROMETER_SCALE = 2048.0

    # Gyroscope values are multipled by the following constant (and are in units of deg/s)
    MYOHW_GYROSCOPE_SCALE = 16.0

    # Orientation values are multipled by the following constant (units of a unit quaternion)
    MYOHW_ORIENTATION_SCALE = 16384.0
    print("-------------------------------------------------------------------------------------------")
    orient_w = orient_w / MYOHW_ORIENTATION_SCALE
    orient_x = orient_x / MYOHW_ORIENTATION_SCALE
    orient_y = orient_y / MYOHW_ORIENTATION_SCALE
    orient_z = orient_z / MYOHW_ORIENTATION_SCALE
    accel_1 = accel_1 / MYOHW_ACCELEROMETER_SCALE
    accel_2 = accel_2 / MYOHW_ACCELEROMETER_SCALE
    accel_3 = accel_3 / MYOHW_ACCELEROMETER_SCALE
    gyro_1 = gyro_1 / MYOHW_GYROSCOPE_SCALE
    gyro_2 = gyro_2 / MYOHW_GYROSCOPE_SCALE
    gyro_3 = gyro_3 / MYOHW_GYROSCOPE_SCALE

    abs_emg_list = np.abs(emg_list)
    filter_emg = abs_emg_list
    get_position([orient_w, orient_x, orient_y, orient_z], [accel_1, accel_2, accel_3])
    # get_position([orient_w, orient_x, orient_y, orient_z], [gyro_1, gyro_2, gyro_3])
    global denominator
    if (len(total_emg_list) > 20):
        denominator -= total_emg_list[len(total_emg_list) - 21]
        filter_emg = denominator / 20
    denominator += abs_emg_list
    total_emg_list.append(abs_emg_list)
    right_emg_list.append(filter_emg)
    # print(denominator)
    # print(filter_emg)
    # global count
    # 识别emg信号
    # if count > 50:
    #     float_emg = np.float32(filter_emg)
    #     float_emg_list = []
    #     float_emg_list.append(float_emg)
    #     input = torch.tensor(float_emg_list).to(device)
    #     output = model(input)
    #     predict = torch.argmax(output)
    #     if predict == 0:
    #         print('normal')
    #     elif predict == 1:
    #         print('left')
    #     elif predict == 2:
    #         print('right')
    #     elif predict == 3:
    #         print('punch')
    #     count = 0
    # else:
    #     count += 1
    # print(len(right_emg_list))
    print('2')
    # filename = 'punch.txt'
    # with open(filename, 'a') as file_object:
    #     file_object.write(str(filter_emg) + '\n')


if __name__ == "__main__":
    scan_time = 10
    # EMG MODEL#
    # model = torch.load(os.path.dirname(os.path.dirname(__file__))+'/model/classify.pkl').to(device)
    # device_1.add_imu_handler()
    # device_1.add_emg_handler()

    # DEVICE1#
    device_1 = MyoDongle("/dev/ttyACM3")
    device_1.clear_state()
    myo_devices = device_1.discover_myo_devices()
    if len(myo_devices) > 0:
        device_1.connect(myo_devices[0])
    else:
        print("Device 1 not found, exiting...")
        exit()
    device_1.enable_imu_readings()
    device_1.enable_emg_readings()
    device_1.add_joint_emg_imu_handler(joint_event_handler1)
    # device_1.scan_for_data_packets_conditional()

    # DEVICE2#
    device_2 = MyoDongle("/dev/ttyACM0")
    device_2.clear_state()

    if len(myo_devices) > 1:
        device_2.connect(myo_devices[1])
    else:
        print("Device 2 not found, exiting...")
        exit()
    device_2.enable_imu_readings()
    device_2.enable_emg_readings()
    device_2.add_joint_emg_imu_handler(joint_event_handler2)
    # device_2.scan_for_data_packets_conditional()
    while (True):
        device_1.scan_for_data_packets(scan_time)
        print('loop')
        device_2.scan_for_data_packets(scan_time)
