#Updated by Daniil Kavalevich

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        self.window_name = "camera"
        cv2.namedWindow(self.window_name)
        self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.current_positions = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.pub_traj = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.delta = 0.5
        self.last_move_time = self.get_clock().now()

        self.point = None
        self.dlugosc = 100
        cv2.setMouseCallback(self.window_name, self.draw_rectangle)

        self.get_logger().info("MinimalSubscriber uruchomiony")


    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.current_positions = [name_to_pos[j] for j in self.joint_names]
        except KeyError:
            pass


    def listener_callback(self, image_data):
        cv_image = np.zeros((512, 700, 3), np.uint8)

        if self.point is not None:
            cv2.rectangle(
                cv_image,
                self.point,
                (self.point[0] + self.dlugosc, self.point[1] + self.dlugosc),
                (0, 255, 0),
                3
            )

            now = self.get_clock().now()
            if self.current_positions is None:
                self.get_logger().warn("Czekam na joint_states…")
            elif self.point[1] < 256 and (now - self.last_move_time).nanoseconds > 3e9:
                self.get_logger().info(f"Klik: {self.point}, wysyłam trajektorię")
                self.move_arm_right()
                self.last_move_time = now
            elif self.point[1] > 256 and (now - self.last_move_time).nanoseconds > 3e9:
            	self.get_logger().info(f"Klik: {self.point}, wysyłam trajektorię")
                self.move_arm_left()
                self.last_move_time = now
            else:
                self.get_logger().info(f"Klik poza strefą ruchu lub cooldown: {self.point}")

        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(25)


    def draw_rectangle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)
            self.get_logger().info(f"Kliknięto w: {self.point}")


    def move_arm_right(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        new_positions = self.current_positions.copy()
        new_positions[0] += self.delta  

        point.positions = new_positions
        point.time_from_start = Duration(sec=3, nanosec=0)
        traj.points.append(point)

        self.pub_traj.publish(traj)
        self.get_logger().info(f"Wysłano trajektorię: {new_positions}")
        self.current_positions = new_positions 

    def move_arm_left(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        new_positions = self.current_positions.copy()
        new_positions[0] -= self.delta  

        point.positions = new_positions
        point.time_from_start = Duration(sec=3, nanosec=0)
        traj.points.append(point)

        self.pub_traj.publish(traj)
        self.get_logger().info(f"Wysłano trajektorię: {new_positions}")
        self.current_positions = new_positions 	
	
	
    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()



def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Wyłączanie...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

