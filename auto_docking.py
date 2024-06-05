#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import lz4.frame
import numpy as np
import cv2
import math
import time
import threading
import dubins
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf.transformations
initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
goal_reached=False

marker_size = 0.03
twist = Twist()
prev_twist = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_data = [None, None, None]
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0
goal_is_close = False
goal_arrived = False
flag=False
dubins_path_made = False    #Dubins

k = np.load("/home/sy/catkin_ws/src/myagv_ros/src/myagv_odometry/src/calibration_matrix.npy", allow_pickle=True)
d = np.load("/home/sy/catkin_ws/src/myagv_ros/src/myagv_odometry/src/distortion_coefficients.npy", allow_pickle=True)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()
find_marker_lock = threading.Lock()


def goal_result_callback(msg):
    global goal_reached
    global flag
    if msg.status.status == 3:  # Goal reached successfully
        goal_reached = True
        flag=False
        rospy.loginfo("Goal reached successfully")
        sub_cam()

# Kalman Filter
# kf = cv2.KalmanFilter(6, 3)
# kf.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],
#                                  [0, 1, 0, 0, 0, 0],
#                                  [0, 0, 1, 0, 0, 0]], np.float32)
# kf.transitionMatrix = np.array([[1, 0, 0, 1, 0, 0],
#                                 [0, 1, 0, 0, 1, 0],
#                                 [0, 0, 1, 0, 0, 1],
#                                 [0, 0, 0, 1, 0, 0],
#                                 [0, 0, 0, 0, 1, 0],
#                                 [0, 0, 0, 0, 0, 1]], np.float32)

# kf.processNoiseCov = np.eye(6, dtype=np.float32) * 0.01
# kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1
# kf.errorCovPost = np.eye(6, dtype=np.float32)
# kf.statePost = np.zeros((6, 1), np.float32)

def callback(compressed_img_msg):
    global k, d, goal_is_close, goal_arrived,flag, pose_data, dubins_path_made
    decompressed_img = lz4.frame.decompress(compressed_img_msg.data)
    np_arr = np.frombuffer(decompressed_img, dtype=np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame_with_markers = pose_estimation(frame)

    # if pose_data[0] is not None:
    #     z = np.array(pose_data, dtype=np.float32).reshape(3, 1)
    #     kf.correct(z)
    # kf.predict()

    # filtered_pose_data = kf.statePost[:3].flatten()
    cv2.imshow("Received Image", frame_with_markers)
    cv2.waitKey(1)
    if flag:
        goal_reached=False
        # while True:
        #     if goal_reached==True:
        #         break
        #     time.sleep(1)
        #initialize
        goal_is_close=False
        goal_arrived=False
        # flag=False

    elif pose_data[0] is not None:
        # adjust_pose(filtered_pose_data)
        adjust_pose(pose_data)

        # Dubins
        # if not dubins_path_made:
        #     dubins_path_made=True
        #     makePath_and_go()   #make Dubins path, and move
        # else:
        #     # adjust_pose(filtered_pose_data)
        #     adjust_pose(pose_data)
    else:
        if goal_arrived:
            go(0.0, 0.0, 0.0)
            #time.sleep(12)
            #flag=True
            box_on = input("\nWhen Box is on Conveyor, press 'enter' : ").lower()
            if box_on == '' :
                go(-0.1, 0.0, 0.0)
                time.sleep(4)
                go(0.0, 0.0, 0.0)
                rospy.set_param('/initial_pose_reached', True)
                rospy.loginfo("initial pose reached")
                flag=True


        elif goal_is_close:
            go(0.01, 0.0, 0.0)
            time.sleep(0.8)
            go(0.0, 0.0, 0.0)
            goal_arrived = True
        else:
            if find_marker_lock.acquire(blocking=False):
                try:
                    threading.Thread(target=find_marker).start()
                finally:
                    find_marker_lock.release()

    # cv2.imshow("Received Image", frame_with_markers)
    # cv2.waitKey(1)

def makePath_and_go():
    go(0.0, 0.0, 0.0)
    while True:
        if find_marker_lock.acquire(blocking=False):
            break
    print(pose_data)
    start_x = 0.0  
    start_y = 0.0  
    start_yaw = np.deg2rad(90)  
    end_x = pose_data[0]  
    end_y = pose_data[1] - 10
    end_yaw = np.deg2rad(90-pose_data[2])  
    curvature = 1000
    path_x, path_y, path_yaw, mode, lengths = dubins.plan_dubins_path(start_x,
                                                        start_y,
                                                        start_yaw,
                                                        end_x,
                                                        end_y,
                                                        end_yaw,
                                                        curvature,
                                                        # selected_types=['RSL','LSR'],
                                                        step_size=0.05)

    
    linear_length=np.linalg.norm(np.array([pose_data[0], pose_data[1]])-np.array([0,0]))
    distance_efficiency = sum(lengths)/linear_length

    rot_angle=np.rad2deg(abs(-start_yaw+path_yaw[int(len(path_yaw)/2)])) /23
    last_rot_angle=np.rad2deg(abs(end_yaw-path_yaw[int(len(path_yaw)/2)])) /23
 
    print("path yaw length", len(path_yaw))
    print("start yaw, end_yaw", start_yaw, end_yaw)
    print("middle of path_yaw", path_yaw[int(len(path_yaw)/2)])
    print("rot angle", rot_angle)
    print("lsat rot angle", last_rot_angle)

    if (distance_efficiency>1.3):
        print("No Dubins Path")
        print(distance_efficiency)
        pass
    else:
        print("Use Dubins Path")
        print(start_x, start_y, start_yaw, end_x, end_y, end_yaw,curvature)
        print(mode, lengths)

        for direction, distance in zip(mode, lengths):
            if direction=='R':
                go(0.02, 0.0, -0.1)
                print("R", rot_angle)                
                time.sleep(rot_angle)
                rot_angle=last_rot_angle
                go(0.0, 0.0, 0.0)
                time.sleep(0.5)

            elif direction=='S':
                go(0.1, 0.0, 0.0)
                speed=2.2
                print("S", distance/speed)                
                time.sleep(distance/speed)
                go(0.0, 0.0, 0.0)
                time.sleep(0.5)

            elif direction=='L':
                go(0.02, 0.0, 0.1)
                print("L", rot_angle)                
                time.sleep(rot_angle)
                rot_angle=last_rot_angle
                go(0.0, 0.0, 0.0)
                time.sleep(0.5)
        go(0.0, 0.0, 0.0)
        print("go 0 0 0")

def adjust_pose(pose):
    global goal_is_close
    if pose[1] > 10:
        if pose[2] > 5:
            go(0.1, 0.0, 0.2)
        elif pose[2] < -5:
            go(0.1, 0.0, -0.2)
        elif pose[0] > 0.1:
            go(0.05, -0.1, 0.0)
        elif pose[0] < -0.1:
            go(0.05, 0.1, 0.0)
        else:
            go(0.1, 0.0, 0.0)
    elif pose[1] > 6:
        if pose[0] > 0.1:
            go(0.05, -0.1, 0.0)
        elif pose[0] < -0.1:
            go(0.05, 0.1, 0.0)
        elif pose[2] > 3:
            go(0.1, 0.0, 0.2)
        elif pose[2] < -3:
            go(0.1, 0.0, -0.2)
        else:
            go(0.1, 0.0, 0.0)
    elif pose[1] > 2.5:
        go(0.0, 0.0, 0.0)
        if pose[0] > 0.1:
            go(0.0, -0.02, 0.0)
        elif pose[0] < -0.1:
            go(0.0, 0.02, 0.0)
        elif pose[2] > 3:
            go(0.0, 0.0, 0.1)
        elif pose[2] < -3:
            go(0.0, 0.0, -0.1)
        else:
            go(0.01, 0.0, 0.0)
    elif pose[1] > 2.0:
        go(0.0, 0.0, 0.0)
        if pose[0] > 0.1:
            go(0.0, -0.02, 0.0)
        elif pose[0] < -0.1:
            go(0.0, 0.01, 0.0)
        elif pose[2] > 3:
            go(0.0, 0.0, 0.1)
        elif pose[2] < -3:
            go(0.0, 0.0, -0.1)
        else:
            goal_is_close = True
            go(0.02, 0.0, 0.0)
    else:
        goal_is_close = True
        go(0.02, 0.0, 0.0)

def find_marker():
    global find_marker_lock
    with find_marker_lock:
        go(0.0, 0.0, -0.1)
        time.sleep(0.5)
        go(0.0, 0.0, 0.0)
        time.sleep(0.1)

def pose_estimation(frame):
    global pose_data, aruco_dict, parameters
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
            if ids[i] == target_marker_id:
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
                rvec = rvec[0][0]
                tvec = tvec[0][0]
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(R_flip * R_tc)
                pose_data[0] = tvec[0] * marker_size * 1000
                pose_data[1] = tvec[2] * marker_size * 1000
                pose_data[2] = math.degrees(pitch_marker)
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.aruco.drawAxis(frame, k, d, rvec, tvec, 0.01)

                marker_id = ids[i][0]
                marker_center = tuple(np.mean(corners[i][0], axis=0).astype(int))
                cv2.putText(frame, f"ID: {marker_id}", marker_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        pose_data[0] = None
        pose_data[1] = None
        pose_data[2] = None
    return frame

def _is_rotation_matrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def _rotation_matrix_to_euler_angles(R):
    assert (_is_rotation_matrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def go(x, y, theta):
    global prev_twist
    if x != prev_twist.linear.x or y != prev_twist.linear.y or theta != prev_twist.angular.z:
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = theta
        pub.publish(twist)
        
        prev_twist = twist


def sub_cam():
    #if goal_reached-> get_param destination & main()
    global target_marker_id
    target_marker_id=rospy.get_param('target_marker_id')
    print("target marker id :", target_marker_id)
    print('waiting for image')
    global image_subscriber
    try:
        image_subscriber = rospy.Subscriber('/camera/compressed_image', CompressedImage, callback)
        
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_result_callback)
    while not rospy.is_shutdown():
        print('waiting for goal_reached')
        rospy.spin()
