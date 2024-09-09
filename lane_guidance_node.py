import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import time

NODE_NAME = 'lane_guidance_node'
CENTROID_TOPIC_NAME = '/centroid'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        self.centroid_subscriber = self.create_subscription(Float32, CENTROID_TOPIC_NAME, self.controller, 10)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('error_threshold', 0.15),
                ('zero_throttle', 0.0),
                ('max_throttle', 0.2),
                ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
            ])
        self.Kp = self.get_parameter('Kp_steering').value
        self.Ki = self.get_parameter('Ki_steering').value
        self.Kd = self.get_parameter('Kd_steering').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value

        self.Ts = float(1 / 20)
        self.ek = 0
        self.ek_1 = 0
        self.proportional_error = 0
        self.derivative_error = 0
        self.integral_error = 0
        self.integral_max = 1E-8

        self.paused = False
        self.pause_service = self.create_service(SetBool, 'toggle_pause', self.toggle_pause_callback)

        self.get_logger().info(
            f'\nKp_steering: {self.Kp}'
            f'\nKi_steering: {self.Ki}'
            f'\nKd_steering: {self.Kd}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
            f'\nmax_throttle: {self.max_throttle}'
            f'\nmin_throttle: {self.min_throttle}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
        )

    def controller(self, data):
        if self.paused:
            self.get_logger().info('Controller paused, skipping control logic.')
            return

        self.ek = data.data
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle) / (1 - self.error_threshold)) * abs(self.ek) + self.inf_throttle
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        self.proportional_error = self.Kp * self.ek
        self.derivative_error = self.Kd * (self.ek - self.ek_1) / self.Ts
        self.integral_error += self.Ki * self.ek * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        steering_float_raw = self.proportional_error + self.derivative_error + self.integral_error
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)

        try:
            self.twist_cmd.angular.z = steering_float
            self.twist_cmd.linear.x = throttle_float
            self.twist_publisher.publish(self.twist_cmd)

            self.ek_1 = self.ek

        except KeyboardInterrupt:
            self.twist_cmd.linear.x = self.zero_throttle
            self.twist_publisher.publish(self.twist_cmd)

    def toggle_pause_callback(self, request, response):
        self.paused = request.data
        if self.paused:
            self.get_logger().info('Controller paused.')
        else:
            self.get_logger().info('Controller resumed.')
        response.success = True
        response.message = f'Paused state set to {self.paused}'
        return response

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound is None:
            lower_bound = -upper_bound
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.twist_cmd.linear.x = path_planner_publisher.zero_throttle
        path_planner_publisher.twist_publisher.publish(path_planner_publisher.twist_cmd)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
