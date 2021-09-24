#!/usr/bin/env python3
from pymyolinux.core.myo import MyoDongle
import numpy as np
import rospy
import time
import os
import joblib
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

emg_sum_white = [0, 0, 0, 0, 0, 0, 0, 0]
emg_sum_black = [0, 0, 0, 0, 0, 0, 0, 0]
total_emg_list_white = []
right_emg_list_white = []
total_emg_list_black = []
right_emg_list_black = []
x = list(range(1, 101))
# count = 0
last_time1 = 0
last_time2 = 0
## white
# 位置
position = [0, 0, 0]
centerofSphere = [0, 0, 0]
R = 1
# 计数
count = 0
max_accel = [0, 0, 0]
max_accel_sum = 0
quat_world_to_refer = [0, 0, 0, 0]
#控制命令发送时间
send_control_time_white = 0
send_control_time_black = 0
# 控制命令序列及及其更新时间
control_labels_white = []
control_time_white = 0
control_labels_black = []
control_time_black = 0
#允许发布四元数
enable_emg = 0

## black
#gyroscope
gyro_list = [0, 0, 0]
#emg标签集
white_labels = []
black_labels = []
#white_labels_count = {'[\'normal\']':0, '[\'outside\']':0, '[\'inside\']':0, '[\'punch\']':0}
white_labels_count = {'normal':0, 'outside':0, 'inside':0, 'punch':0}
black_labels_count = {'normal':0, 'outside':0, 'inside':0, 'punch':0}




# filter the emg signals
def accel_filter(accel):
    global max_accel, max_accel_sum
    sum = accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2
    # if sum > max_accel_sum:
    #     max_accel = accel
    #     max_accel_sum = sum
    # print(max_accel)
    if sum < 6:
        accel = [0, 0, 0]
    # print(max_accel)
    return accel


# get myo's current position
def get_position(quat, last_time):
    global position, count, R, centerofSphere
    count += 1
    t = time.time()  # get current time
    if last_time != 0:
        period = t - last_time
        x = quat[1]
        y = quat[2]
        z = quat[3]
        denom = (x ** 2 + y ** 2 + z ** 2) ** (1 / 2)
        if denom != 0:
            position[0] = x / denom * R + centerofSphere[0]
            position[2] = -y / denom * R + centerofSphere[1]
            position[1] = -z / denom * R + centerofSphere[2]
        print(position)
    # print('accel_world:',accel_world)
    # print('raw_velocity:', raw_velocity)
    # position += raw_velocity
    # print('position:',position)
    return t  # return period


# transform the quaternion from world Coordinate System to refer Coordinate System
def quat_transform(quat):
    w = quat[0]
    x = -quat[1]
    y = -quat[2]
    z = -quat[3]
    global quat_world_to_refer
    a = quat_world_to_refer[0]
    b = quat_world_to_refer[1]
    c = quat_world_to_refer[2]
    d = quat_world_to_refer[3]
    w_refer = w * a - x * b - y * c - z * d
    x_refer = w * b + x * a + y * d - z * c
    y_refer = w * c - x * d + y * a + z * b
    z_refer = w * d + x * c - y * b + z * a
    return [w_refer, x_refer, y_refer, z_refer]


# transform accel using quaternion
def accel_transform(accel):
    global quat_world_to_refer
    qw = quat_world_to_refer[0]
    qx = -quat_world_to_refer[1]
    qy = -quat_world_to_refer[2]
    qz = -quat_world_to_refer[3]
    x = accel[0]
    y = accel[1]
    z = accel[2]
    x_trans = (qw**2 + qx**2 - qy**2 - qz**2)*x + 2*(qx*qy - qw*qz)*y + 2*(qx*qz + qw*qy)*z
    y_trans = 2*(qx*qy + qw*qz)*x +(qw**2 - qx**2 + qy**2 - qz**2)*y + 2*(qy*qz - qw*qx)*z
    z_trans = 2*(qx*qz - qw*qy)*x + 2*(qy*qz + qw*qx)*y + (qw**2 - qx**2 - qy**2 + qz**2)*z
    return [x_trans, y_trans, z_trans]


# predict the label of emg using knn
def predict_white_emg(emg):
    global white_labels, white_labels_count
    label = str(white_knn.predict([emg])).strip('[').strip(']').strip('\'')
    white_labels_count[label] += 1
    if len(white_labels) > 20:
        white_labels_count[white_labels[0]] -= 1
        del(white_labels[0])
    white_labels.append(label)
    final_label = max(white_labels_count, key=white_labels_count.get)
    # print(final_label)
    return final_label


# predict the label of emg using knn
def predict_black_emg(emg):
    global black_labels, black_labels_count
    label = str(black_knn.predict([emg])).strip('[').strip(']').strip('\'')
    black_labels_count[label] += 1
    if len(black_labels) > 20:
        black_labels_count[black_labels[0]] -= 1
        del(black_labels[0])
    black_labels.append(label)
    final_label = max(black_labels_count, key=black_labels_count.get)
    # print(final_label)
    return final_label


# get angular
def integrity_gyro(gyro):
    global gyro_list, last_time2
    current_time = time.time()
    if last_time2 == 0:
        last_time2 = current_time
    else:
        period = current_time - last_time2
        sum = gyro[0]**2 + gyro[1]**2 + gyro[2]**2
        if sum < 7:
            gyro = [0., 0., 0.]
        gyro_list += np.array(gyro)
        # print(gyro_list)


def joint_event_handler1(emg_list, orient_w, orient_x, orient_y, orient_z, accel_1,
                         accel_2, accel_3, gyro_1, gyro_2, gyro_3, sample_num):
    # Accelerometer values are multiplied by the following constant (and are in units of g)
    MYOHW_ACCELEROMETER_SCALE = 2048.0

    # Gyroscope values are multiplied by the following constant (and are in units of deg/s)
    MYOHW_GYROSCOPE_SCALE = 16.0

    # Orientation values are multiplied by the following constant (units of a unit quaternion)
    MYOHW_ORIENTATION_SCALE = 16384.0
    # print("-------------------------------------------------------------------------------------------")
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
    quat_refer = quat_transform([orient_w, orient_x, orient_y, orient_z])
    # print(quaternion, quat_refer)
    abs_emg_list = np.abs(emg_list)
    filter_emg = abs_emg_list
    global last_time1
    # get_position([orient_w, orient_x, orient_y, orient_z], [accel_1, accel_2, accel_3])
    last_time1 = get_position(quat_refer, last_time1)
    global emg_sum_white, control_time_white, control_labels_white, send_control_time_white, quat_world_to_refer
    if len(total_emg_list_white) > 20:
        emg_sum_white -= total_emg_list_white[len(total_emg_list_white) - 21]
        filter_emg = emg_sum_white / 20
    emg_sum_white += abs_emg_list
    total_emg_list_white.append(abs_emg_list)
    right_emg_list_white.append(filter_emg)
    emg_label = predict_white_emg(filter_emg)
    # print(emg_label)
    current_time = time.time()
    if control_time_white == 0:
        control_time_white = current_time
    elif current_time - control_time_white > 0.2:
        control_time_white = current_time
        if len(control_labels_white) > 2:
            del(control_labels_white[0])
        control_labels_white.append(emg_label)
    if send_control_time_white == 0:
        send_control_time_white = current_time
    elif current_time - send_control_time_white > 1:
        send_control_time_white = current_time
        if control_labels_white[0] == control_labels_white[1] and control_labels_white[1] == control_labels_white[2]:
            if control_labels_white[0] == 'outside':
                global enable_emg, position
                quat_world_to_refer = [orient_w, orient_x, orient_y, orient_z]
                enable_emg = 1
                position = [0, 0, 0]
            elif control_labels_white[0] == 'punch':
                pose_msg = Pose()
                pose_msg.orientation.w = quat_refer[0]
                pose_msg.orientation.x = quat_refer[1]
                pose_msg.orientation.y = quat_refer[2]
                pose_msg.orientation.z = quat_refer[3]
                pose_msg.position.x = position[0]
                pose_msg.position.y = position[1]
                pose_msg.position.z = position[2]
                white_control_pub.publish(pose_msg)




def joint_event_handler2(emg_list, orient_w, orient_x, orient_y, orient_z, accel_1, accel_2, accel_3, gyro_1, gyro_2,
                         gyro_3, sample_num):
    # Accelerometer values are multiplied by the following constant (and are in units of g)
    MYOHW_ACCELEROMETER_SCALE = 2048.0

    # Gyroscope values are multiplied by the following constant (and are in units of deg/s)
    MYOHW_GYROSCOPE_SCALE = 16.0

    # Orientation values are multiplied by the following constant (units of a unit quaternion)
    MYOHW_ORIENTATION_SCALE = 16384.0
    # print("-------------------------------------------------------------------------------------------")
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
    integrity_gyro([gyro_1, gyro_2, gyro_3])
    quat_refer = quat_transform([orient_w, orient_x, orient_y, orient_z])
    abs_emg_list = np.abs(emg_list)
    filter_emg = abs_emg_list
    global last_time2
    # get_position([orient_w, orient_x, orient_y, orient_z], [gyro_1, gyro_2, gyro_3])
    global emg_sum_black, total_emg_list_black, emg_sum_black, control_time_black, control_labels_black, send_control_time_black
    if len(total_emg_list_black) > 20:
        emg_sum_black -= total_emg_list_black[len(total_emg_list_black) - 21]
        filter_emg = emg_sum_black / 20
    emg_sum_black += abs_emg_list
    total_emg_list_black.append(abs_emg_list)
    right_emg_list_black.append(filter_emg)
    # print(filter_emg)
    emg_label = predict_black_emg(filter_emg)
    # print(emg_label)
    current_time = time.time()
    if control_time_black == 0:
        control_time_black = current_time
    elif current_time - control_time_black > 0.2:
        control_time_black = current_time
        if len(control_labels_black) > 2:
            del(control_labels_black[0])
        control_labels_black.append(emg_label)
    if send_control_time_black == 0:
        send_control_time_black = current_time
    elif current_time - send_control_time_black > 1:
        send_control_time_black = current_time
        global gyro_list
        twist = Twist()
        if gyro_list[0] < -4000:
            twist.angular.z = 0.05
        elif gyro_list[0] > 4000:
            twist.angular.z = -0.05
        else:
            twist.angular.z = 0
        if control_labels_black[0] == control_labels_black[1] and control_labels_black[1] == control_labels_black[2]:
            if control_labels_black[0] == 'punch':
                gyro_list = [0, 0, 0]
            elif control_labels_black[0] == 'normal':
                twist.linear.x = 0
            elif control_labels_black[0] == 'outside':
                twist.linear.x = 0.25
            elif control_labels_black[0] == 'inside':
                twist.linear.x = -0.25
        black_control_pub.publish(twist)
    # pub2.publish(myo_msg(gyroscope, acceleration, quaternion, emg_list))
    # filename = 'normal.txt'
    # with open(filename, 'a') as file_object:
    #      file_object.write(str(filter_emg) + '\n')


if __name__ == "__main__":
    scan_time = 0.005
    rospy.init_node('myo_pub', anonymous=True)
    # device_1.add_imu_handler()
    # device_1.add_emg_handler()
    # EMG MODEL#
    white_knn = joblib.load(os.path.dirname(os.path.dirname(__file__)) + '/model/knn_whitemyo')
    black_knn = joblib.load(os.path.dirname(os.path.dirname(__file__)) + '/model/knn_blackmyo')


    black_control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # position & quaternion topic
    white_control_pub = rospy.Publisher('/pose', Pose, queue_size=10)

    # DEVICE1#
    device_1 = MyoDongle("/dev/ttyACM1")
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
    while True:
        device_1.scan_for_data_packets(scan_time)
        device_2.scan_for_data_packets(scan_time)
