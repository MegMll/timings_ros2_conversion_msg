#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from ros_interface_mpc.msg import InitialState, Trajectory
from std_msgs.msg import Float64MultiArray
from ros_interface_mpc_utils.conversions import numpy_to_multiarray_float64, numpy_to_multiarray_int8

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('python_trajectory')
        
        self.publisher_ = self.create_publisher(Trajectory, '/trajectory', 10)
        self.subscription_ = self.create_subscription(
            InitialState,
            '/initial_state',
            self.state_callback,
            10
        )

        time_length = 10
        nq = 10

        self.x0_ = np.zeros(time_length)
        self.k0_ = np.zeros((time_length, nq))
        self.us_ = np.zeros((time_length, nq))
        self.xs_ = np.zeros((time_length, nq))
        self.ddq = np.zeros((time_length, nq))
        self.forces = np.zeros((time_length, nq))
        self.contact_states = np.zeros((time_length, nq), dtype=int)

        self.get_logger().info('Python trajectory instantiated')

    def state_callback(self, msg):
        self.x0_ = np.array(msg.x0)

        self.k0_ = np.random.random(self.k0_.shape)
        self.us_ = np.random.random(self.us_.shape)
        self.xs_ = np.random.random(self.xs_.shape)
        self.ddq = np.random.random(self.ddq.shape)
        self.forces = np.random.random(self.forces.shape)
        self.contact_states = np.random.randint(0, 2, self.contact_states.shape)

        msg2send = Trajectory()

        # Transform Eigen-like matrices into ROS MultiArray
        msg2send.k0 = numpy_to_multiarray_float64(self.k0_)
        msg2send.us = numpy_to_multiarray_float64(self.us_)
        msg2send.xs = numpy_to_multiarray_float64(self.xs_)
        msg2send.ddqs = numpy_to_multiarray_float64(self.ddq)
        msg2send.forces = numpy_to_multiarray_float64(self.forces)
        msg2send.contact_states = numpy_to_multiarray_int8(self.contact_states)

        # Set timestamp
        now = self.get_clock().now().to_msg()
        msg2send.stamp = now

        self.publisher_.publish(msg2send)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()