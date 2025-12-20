#!/usr/bin/env python3

import rclpy                            #----------Python Client Library for ROS 2

#----------Topic messages
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

#----------Others
from cv_bridge import CvBridge          #----------ROS2 package to convert between ROS and OpenCV Images
import cv2                              #----------Python OpenCV library
import numpy as np

SCREEN_SIZE=(620,720,3)

class MinimalSubsPubs(Node):
    def __init__(self):
        super().__init__('minimal_subspubs')

        #----------SUBSCRIBER
        self.window_name = "camera"
        self.subscription = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.subscription               #----------prevent unused variable warning

        #Trajectory points
        self.test_traj = {"points":[[0.0, 3.0, 0.0, -2.0, 0.0, 0.0]],
                          "time":[Duration(sec=5)]}

        #----------PUBLISHER
        self.UR_publisher = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.declare_parameter('rect_size', 200)
        self.rect_size = self.get_parameter('rect_size').value
        print(f"self.rect_size: {self.rect_size}")
        self.point = None
        self.timer = None

    ## Draw a square when clicking on the black window and publish data to UR5
    def listener_callback(self, image_data):
        cv_image = np.zeros(SCREEN_SIZE, np.uint8)

        cv2.line(cv_image, (0, (int)(SCREEN_SIZE[0] / 2)),                  #----------Divider line
                 (SCREEN_SIZE[1], int(SCREEN_SIZE[0] / 2)), (0, 0, 255), 3)

        if(self.point is not None):
            cv2.rectangle(cv_image,self.point,(self.point[0]+self.rect_size,self.point[1]+self.rect_size),(0,250,0),3)

        cv2.imshow(self.window_name,cv_image)
        cv2.waitKey(25)
        cv2.setMouseCallback(self.window_name, self.trajectory_to_robot)

    ## Create and publish data to UR5
    def trajectory_to_robot(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:                      #----------check if mouse event is clicked
            self.point = (x, y)                                 #----------to draw rectangle

            #Create suitable topic message
            trajectory = JointTrajectory()

            trajectory.joint_names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                                "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
            trajectory.points = [
                JointTrajectoryPoint(
                    positions=self.test_traj["points"][i], time_from_start=self.test_traj["time"][i]
                )
                for i in range(len(self.test_traj["points"]))
            ]

            print(trajectory)                                   #----------for debug
            self.UR_publisher.publish(trajectory)




def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubsPubs()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
