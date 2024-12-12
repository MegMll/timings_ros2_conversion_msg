#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8MultiArray, MultiArrayDimension
from ros_interface_mpc.msg import InitialState, Trajectory
from ros_interface_mpc_utils.conversions import multiarray_to_numpy
import numpy as np

from rclpy.clock import Clock

class StatePublisher(Node):
    def __init__(self):
        super().__init__('python_state')

        self.publisher_ = self.create_publisher(InitialState, '/initial_state', 10)
        self.subscription_ = self.create_subscription(
            Trajectory,
            '/trajectory',
            self.trajectory_callback,
            10
        )

        self.declare_parameter('nq', 10)
        self.declare_parameter('time_length', 10)

        time_length = self.get_parameter('time_length').value
        nq = self.get_parameter('nq').value

        self.x0_ = np.zeros(time_length)
        self.k0_ = np.zeros((time_length, nq))
        self.us_ = np.zeros((time_length, nq))
        self.xs_ = np.zeros((time_length, nq))
        self.ddq = np.zeros((time_length, nq))
        self.forces = np.zeros((time_length, nq))
        self.contact_states = np.zeros((time_length, nq), dtype=int)

        self.get_logger().info('Python state instantiated')

        self.x0_ = np.random.rand(time_length)
        msg2send = InitialState()

        # Convert numpy array to list
        msg2send.x0 = self.x0_.tolist()
        msg2send.stamp = Clock().now().to_msg()

        self.publisher_.publish(msg2send)

        self.delays = []

    def trajectory_callback(self, msg):
        # Process trajectory message
        self.us_ = multiarray_to_numpy(msg.us)
        self.xs_ = multiarray_to_numpy(msg.xs)
        self.k0_ = multiarray_to_numpy(msg.k0)
        self.ddq = multiarray_to_numpy(msg.ddqs)
        self.forces = multiarray_to_numpy(msg.forces)
        self.contact_states = multiarray_to_numpy(msg.contact_states)

        self.trajectory_stamp_ = msg.stamp

        self.x0_ = np.random.rand(self.x0_.shape[0])
        msg2send = InitialState()

        # Convert numpy array to list
        msg2send.x0 = self.x0_.tolist()

        state_time = Clock().now().to_msg()
        msg2send.stamp = state_time

        self.publisher_.publish(msg2send)

        now = Clock().now().to_msg()
        delay = (now.nanosec - msg.stamp.nanosec) / 1e9 + (now.sec - msg.stamp.sec)
        self.delays.append(delay)

        avg_delay = sum(self.delays) / len(self.delays)
        self.get_logger().info(f'Average delay: {avg_delay:.6f} seconds')


def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()

    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        state_publisher.get_logger().info('Shutting down node...')
    finally:
        state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
