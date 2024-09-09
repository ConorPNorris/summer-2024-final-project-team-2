import rclpy
from rclpy.node import Node
from roboflowoak import RoboflowOak
import cv2
from std_msgs.msg import String
from std_srvs.srv import SetBool
import time

node_name = 'main_node'

class RoboflowOakNode(Node):
    
    def __init__(self):
        super().__init__(node_name)
        self.rf = RoboflowOak(model="mae148-human-identification", confidence=0.05, overlap=0.5,
                              version="4", api_key="0RmqstHKjwDcunOH9wus", rgb=True,
                              depth=True, device=None, blocking=True)
        self.get_logger().info("Roboflow inference node has started.")
        
        self.audio_publisher = self.create_publisher(String, 'play_audio', 10)
        self.toggle_pause_client = self.create_client(SetBool, '/toggle_pause')

        self.human_detection_times = []
        self.human_detection_threshold = 3
        self.human_detection_window = 5

        self.human_detection_disabled = False
        self.checking_for_conor = False
        self.conor_check_start_time = None
        self.conor_check_duration = 10
        self.grace_period_active = False
        self.grace_period_start_time = None
        self.grace_period_duration = 10

        self.run()
     
    def run(self):
        while rclpy.ok():
            t0 = time.time()
            result, frame, raw_frame, depth = self.rf.detect()
            predictions = result["predictions"]
            
            frame_width, frame_height, _ = frame.shape
            temp = 2
            frame = cv2.resize(frame, (int(frame_width / temp), int(frame_height / temp)))

            current_time = time.time()

            if self.grace_period_active:
                if current_time - self.grace_period_start_time >= self.grace_period_duration:
                    self.get_logger().info('Grace period ended. Re-enabling human detection.')
                    self.grace_period_active = False
                    self.human_detection_disabled = False
                else:
                    self.get_logger().info('Grace period active. Human detection disabled.')
                    continue

            if not self.human_detection_disabled:
                if any(p.class_name.lower() == 'human-person' for p in predictions):
                    self.human_detection_times.append(current_time)

                    self.human_detection_times = [t for t in self.human_detection_times if current_time - t <= self.human_detection_window]

                    if len(self.human_detection_times) >= self.human_detection_threshold:
                        self.get_logger().info('Human detected multiple times, calling /toggle_pause and playing sound...')
                        self.call_toggle_pause(True)
                        audio_file_path = '/home/projects/ros2_ws/src/ucsd_robocar_hub2/my_custom_roboflow_package/my_custom_roboflow_package/audio_file/identify-yourself.wav'
                        audio_msg = String()
                        audio_msg.data = audio_file_path
                        self.audio_publisher.publish(audio_msg)
                        self.get_logger().info(f'Published audio file path: {audio_file_path}')
                        self.human_detection_disabled = True
                        self.checking_for_conor = True
                        self.conor_check_start_time = current_time

            if self.checking_for_conor:
                if current_time - self.conor_check_start_time <= self.conor_check_duration:
                    if any(p.class_name.lower() == 'conor' for p in predictions):
                        self.get_logger().info('Conor recognized within the 10-second window!')
                        self.play_audio('/home/projects/ros2_ws/src/ucsd_robocar_hub2/my_custom_roboflow_package/my_custom_roboflow_package/audio_file/thank_you.wav')
                        self.checking_for_conor = False
                        self.start_grace_period(current_time)
                else:
                    self.get_logger().info('10-second window expired, conor not found.')
                    self.play_audio('/home/projects/ros2_ws/src/ucsd_robocar_hub2/my_custom_roboflow_package/my_custom_roboflow_package/audio_file/get_out.wav')
                    self.checking_for_conor = False
                    self.start_grace_period(current_time)

                self.call_toggle_pause(False)

            t = time.time() - t0
            self.get_logger().info(f'FPS: {1 / t}')
    
    def start_grace_period(self, current_time):
        self.grace_period_active = True
        self.grace_period_start_time = current_time
        self.get_logger().info('Started 10-second grace period where human detection is disabled.')

    def call_toggle_pause(self, state):
        if not self.toggle_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('/toggle_pause service is not available.')
            return

        request = SetBool.Request()
        request.data = state
        future = self.toggle_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully called /toggle_pause with state: {state}')
        else:
            self.get_logger().error(f'Failed to call /toggle_pause with state: {state}')

    def play_audio(self, audio_file_path):
        audio_msg = String()
        audio_msg.data = audio_file_path
        self.audio_publisher.publish(audio_msg)
        self.get_logger().info(f'Playing audio file: {audio_file_path}')

def main(args=None):
    rclpy.init(args=args)
    roboflow_node = RoboflowOakNode()
    try:
        rclpy.spin(roboflow_node)
    except KeyboardInterrupt:
        roboflow_node.get_logger().info(f'Shutting down {node_name}...')
        cv2.destroyAllWindows()
    finally:
        roboflow_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()