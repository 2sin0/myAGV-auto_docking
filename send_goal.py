#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
import socket
import threading

destination_queue = [1,3]

# x, y, oz, ow

goal_dict = {
    1: [-0.358185350894928,  0.11131536960601807, -0.715335548607076, 0.6987811194494407], 
    2: [-0.358185350894928,  0.11131536960601807, -0.715335548607076, 0.6987811194494407], 
    3: [-1.5979013442993164, 1.5172653198242188,  - 0.9999941849474459, 0.00341028903372672],
    4: [-1.5189454555511475, 1.2149187326431274,  -0.9999987363773453, 0.0015897307044523007]
}
initial_pose_dict = {
    1: [-0.0630316436290741, 0.40619581937789917, 0.00836721823101029, 0.9999649942168349], 
    2: [-0.049162253737449646, 0.7080976963043213, -0.013825346763484552, 0.9999044253261755], 
    3: [-1.550438404083252, 0.9865538477897644, -0.7095144480495683, 0.7046908882686908],
    4: [-2.243189811706543, 1.2377580404281616, -0.7228235120537086, 0.6910326840478257]
}

def create_goal(n):
    """Create a PoseStamped goal message."""
    dst = goal_dict[n]
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose.position.x = dst[0]
    goal_msg.pose.position.y = dst[1]
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = dst[2]
    goal_msg.pose.orientation.w = dst[3]
    return goal_msg

def create_initial_pose(n):
    """Create a PoseWithCovarianceStamped initial pose message."""
    initial_pose = initial_pose_dict[n]
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.frame_id = "map"
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.pose.pose.position.x = initial_pose[0]
    initial_pose_msg.pose.pose.position.y = initial_pose[1]
    initial_pose_msg.pose.pose.position.z = 0.0
    initial_pose_msg.pose.pose.orientation.x = 0.0
    initial_pose_msg.pose.pose.orientation.y = 0.0
    initial_pose_msg.pose.pose.orientation.z = initial_pose[2]
    initial_pose_msg.pose.pose.orientation.w = initial_pose[3]
    initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    return initial_pose_msg



def main():
    rospy.init_node('simple_navigation_goals', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.loginfo("Publisher initialized")
    rospy.set_param('initial_pose_reached', False)
    rospy.sleep(1)
    
    while not rospy.is_shutdown():
        if destination_queue:
            print("dst qeueue :",destination_queue)
            destination = destination_queue.pop(0)
            rospy.set_param('target_marker_id', destination)
            print("target marker id : ", destination)
            goal_msg = create_goal(destination)
            goal_pub.publish(goal_msg)
            print("goal published!")

            rospy.loginfo(f"Waiting for parameter {'/initial_pose_reached'} to be set to True...")
            while rospy.get_param('initial_pose_reached',False) != True:
                        pass

            print("initial_pose_reached")
            # initial_pose_reached = False
            rospy.set_param('initial_pose_reached', False)
            initial_pose_msg = create_initial_pose(destination)
            initial_pose_pub.publish(initial_pose_msg)
            print("initial pose published!")
            rospy.sleep(1)
        rospy.sleep(1)


def start_server(host='172.30.1.78', port=8080):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (host, port)
    print(f'Starting server on {server_address[0]}:{server_address[1]}')
    server_socket.bind(server_address)
    server_socket.listen(1)
    
    while True:
        print('Waiting for a connection...')
        connection, client_address = server_socket.accept()
        
        try:
            print(f'Connection from {client_address}')
            while True:
                data = connection.recv(16)
                if data:
                    print(f'Received: {data.decode()}')
                    dst = int(data.decode())
                    if dst == 1:
                        destination_queue.append(1)
                        destination_queue.append(3)
                    if dst == 2:
                        destination_queue.append(2)
                        destination_queue.append(4)
                    else:
                        destination_queue.append(dst)
                    print("destination queue:", destination_queue)
                else:
                    break
        finally:
            connection.close()

def server_thread():
    server = threading.Thread(target=start_server)
    server.daemon = True
    server.start()

if __name__ == '__main__':
    server_thread()
    main()
