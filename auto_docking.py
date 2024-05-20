#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import lz4.frame
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
import time
from filterpy.kalman import KalmanFilter
import threading

marker_size = 0.03
twist = Twist()
prev_twist = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_data = [None, None, None, None]
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0
flag = 0

# Initialize the Kalman filter
kf = KalmanFilter(dim_x=4, dim_z=4)
kf.x = np.zeros((4, 1))  # State vector [x, y, z, pitch]
kf.F = np.eye(4)  # State transition matrix
kf.H = np.eye(4)  # Measurement matrix
kf.P *= 1000.  # Covariance matrix
kf.R = np.eye(4) * 0.01  # Measurement noise
kf.Q = np.eye(4) * 0.1  # Process noise

goal_is_close=False
goal_arrived=False

marker_lock = threading.Lock()

def callback(compressed_img_msg):
    global bridge, k, d, flag, kf, goal_is_close, goal_arrived

    decompressed_img = lz4.frame.decompress(compressed_img_msg.data)
    np_arr = np.frombuffer(decompressed_img, dtype=np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame_with_markers = pose_estimation(frame, k, d)

    if pose_data[0] is not None:
        kf.predict()
        z = np.array(pose_data).reshape((4, 1))
        kf.update(z)

        filtered_pose_data = kf.x.flatten()
        rospy.loginfo(f"Filtered Pose Data: {filtered_pose_data}")

        if filtered_pose_data[2] > 10:
            if filtered_pose_data[3] > 5:
                go(0.1, 0.0, 0.2)
            elif filtered_pose_data[3] < -5:
                go(0.1, 0.0, -0.2)
            elif filtered_pose_data[0] > 0.1:
                go(0.05, -0.1, 0.0)
            elif filtered_pose_data[0] < -0.1:
                go(0.05, 0.1, 0.0)
            else:
                go(0.1, 0.0, 0.0)

        elif filtered_pose_data[2]>5:
            if filtered_pose_data[0]>0.1:
                go(0.05, -0.1, 0.0)
            elif filtered_pose_data[0]<-0.1:
                go(0.05, 0.1, 0.0)
            elif filtered_pose_data[3]>3:
                go(0.1, 0.0, 0.2)
            elif filtered_pose_data[3]<-3:
                go(0.1, 0.0, -0.2)
            else:
                go(0.1, 0.0, 0.0)

        elif filtered_pose_data[2]>2.7:
            # go(0.0, 0.0, 0.0)
            if filtered_pose_data[0]>0.1:
                # go(0.0, 0.0, 0.0)
                go(0.0, -0.01, 0.0)
            elif filtered_pose_data[0]<-0.1:
                # go(0.0, 0.0, 0.0)
                go(0.0, 0.01, 0.0)
            elif filtered_pose_data[3]>3:
                go(0.0, 0.0, 0.1)
            elif filtered_pose_data[3]<-3:
                go(0.0, 0.0, -0.1)
            else:
                go(0.01, 0.0, 0.0)
        elif filtered_pose_data[2]>2.0:
            goal_is_close=True
            go(0.0, 0.0, 0.0)
            if filtered_pose_data[0]>0.1:
                go(0.0, -0.01, 0.0)
            elif filtered_pose_data[0]<-0.1:
                go(0.0, 0.01, 0.0)
            elif filtered_pose_data[3]>3:
                go(0.0, 0.0, 0.1)
            elif filtered_pose_data[3]<-3:
                go(0.0, 0.0, -0.1)
            else:
                go(0.01, 0.0, 0.0)
        else:
            go(0.01, 0.0, 0.0)
    else:
        if goal_arrived:
            go(0.0, 0.0, 0.0)
        elif goal_is_close:
            go(0.1, 0.0, 0.0)
            time.sleep(0.8)
            go(0.0, 0.0, 0.0)
            goal_arrived=True
        elif marker_lock.acquire(blocking=False):
            try:
                threading.Thread(target=find_marker).start()
            finally:
                marker_lock.release()
    cv2.imshow("Received Image", frame_with_markers)
    cv2.waitKey(1)

def find_marker():
    global marker_lock
    with marker_lock:
        go(0.0, 0.0, 0.2)
        time.sleep(0.5)
        go(0.0, 0.0, 0.0)
        time.sleep(0.1)

def pose_estimation(frame, matrix_coefficients, distortion_coefficients):
    global pose_data
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            rvec = rvec[0][0]
            tvec = tvec[0][0]
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T
            roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(R_flip * R_tc)
            pose_data[0] = tvec[0] * marker_size * 1000
            pose_data[1] = tvec[1] * marker_size * 1000
            pose_data[2] = tvec[2] * marker_size * 1000
            pose_data[3] = math.degrees(pitch_marker)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

            # Put text indicating the ID of the marker
            marker_id = ids[i][0]
            marker_center = tuple(np.mean(corners[i][0], axis=0).astype(int))
            cv2.putText(frame, f"ID: {marker_id}", marker_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        pose_data[0] = None
        pose_data[1] = None
        pose_data[2] = None
        pose_data[3] = None
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
        # rospy.loginfo(twist)
        
        prev_twist = twist

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    bridge = CvBridge()
    k = np.load("/home/eunji/catkin_ws/src/myagv_ros/src/myagv_odometry/src/calibration_matrix.npy", allow_pickle=True)
    d = np.load("/home/eunji/catkin_ws/src/myagv_ros/src/myagv_odometry/src/distortion_coefficients.npy", allow_pickle=True)
    print("OpenCV version:", cv2.__version__)
    time.sleep(1)
    try:
        image_subscriber = rospy.Subscriber('/camera/compressed_image', CompressedImage, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass