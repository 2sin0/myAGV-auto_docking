#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
import socket
import threading

destination_queue = []

# x, y, oz, ow
goal_dict = {
    2: [0.15891988575458527, 0.0010085701942443848, 0.6998400914998777, 0.714299549439479], 
    1: [-0.09311291575431824, 0.008935928344726562, 0.7113993827986584, 0.7027879610193162], 
    3: [-0.7695512771606445, -1.2955982685089111, -0.7122843602996504, 0.7018910101094884],
    4: [-0.14700953662395477, -1.3220847845077515, -0.6996046105798955, 0.7145301875045957]
}

initial_pose_dict = {
    2: [0.15891988575458527, 0.0010085701942443848, 0.6998400914998777, 0.714299549439479], 
    1: [-0.09311291575431824, 0.008935928344726562, 0.7113993827986584, 0.7027879610193162], 
    3: [-0.7695512771606445, -1.2955982685089111, -0.7122843602996504, 0.7018910101094884],
    4: [-0.14700953662395477, -1.3220847845077515, -0.6996046105798955, 0.7145301875045957]
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
    initial_pose_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                        0, 0.1, 0, 0, 0, 0,
                                        0, 0, 0.1, 0, 0, 0,
                                        0, 0, 0, 0.1, 0, 0,
                                        0, 0, 0, 0, 0.1, 0,
                                        0, 0, 0, 0, 0, 0.1]
    return initial_pose_msg

# def goal_result_callback(msg):
#     global goal_reached
#     if msg.status.status == 3:  # Goal reached successfully
#         goal_reached = True
#         rospy.loginfo("Goal reached successfully")

def main():
    rospy.init_node('simple_navigation_goals', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    # rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_result_callback)
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


def start_server(host='172.30.1.29', port=8080):
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
