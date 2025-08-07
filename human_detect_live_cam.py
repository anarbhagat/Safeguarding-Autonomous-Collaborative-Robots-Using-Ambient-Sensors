import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import cv2
import torch
import threading

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control')

        # Publisher to control robot joints
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Flags to track states
        self.person_detected = False
        self.robot_stopped = False

        # Initialize webcam
        self.cap = cv2.VideoCapture('/dev/video0') 
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            self.destroy_node()
            return

        # Reduce frame resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

        # Set the camera frame rate 
        desired_fps = 15  # Use a lower FPS for faster processing on CPU
        self.cap.set(cv2.CAP_PROP_FPS, desired_fps)

        self.get_logger().info("Camera ready!")

        # Load YOLOv7-tiny model
        self.get_logger().info("Loading YOLOv7-tiny model...")
        self.model = torch.hub.load('WongKinYiu/yolov7', 'custom', 'yolov7-tiny.pt', trust_repo=True)
        self.model.eval()
        self.get_logger().info("YOLOv7-tiny model loaded successfully.")

        # Initialize frame capture thread
        self.frame = None
        self.lock = threading.Lock()
        threading.Thread(target=self.capture_frames, daemon=True).start()

    def capture_frames(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def detect_person(self):
        with self.lock:
            if self.frame is not None:
                frame_resized = cv2.resize(self.frame, (160, 120))  # Resize for faster processing
                results = self.model(frame_resized)

                # Filter detections to include only persons (class 0 in COCO dataset)
                person_detections = [d for d in results.xyxy[0] if int(d[5]) == 0]

                # Draw bounding boxes around detected persons
                for det in person_detections:
                    x1, y1, x2, y2 = map(int, det[:4])  # Convert coordinates to integers
                    conf = det[4].item()  # Confidence score
                    label = f'Person {conf:.2f}'
                    cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(self.frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Display the video feed (press 'q' to quit)
                cv2.imshow('Person Detection', self.frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("Exiting due to 'q' key press.")
                    rclpy.shutdown()

                # Check for person detection
                person_detected = len(person_detections) > 0

                # Log detection results
                if person_detected:
                    self.get_logger().info("Person detected!")
                    cv2.imwrite('person_detected.jpg', self.frame)  # Save frame for debugging
                else:
                    self.get_logger().info("No person detected.")

                self.person_detected = person_detected
                return person_detected

    def stop_robot(self):
        # Stop the robot by sending a zero velocity trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Send current positions to stop immediately
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 6
        point.time_from_start.sec = 0  # Immediate stop
        trajectory.points.append(point)

        self.joint_publisher.publish(trajectory)
        self.robot_stopped = True  # Mark the robot as stopped
        self.get_logger().info("Robot stopped.")

    def move_to_position_with_interrupt(self, positions, duration):
        # Create the trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration  # Time in seconds for the motion
        trajectory.points.append(point)

        # Publish the trajectory
        self.joint_publisher.publish(trajectory)
        self.robot_stopped = False  # Mark the robot as moving
        self.get_logger().info(f"Command sent: Move to {positions} in {duration} seconds.")

        # Monitor the movement
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.detect_person():
                self.get_logger().info("Person detected during movement! Stopping robot.")
                self.stop_robot()
                time.sleep(10)  # Halt for 10 seconds
                return False  # Movement was interrupted
            time.sleep(0.1)  # Sleep for a short interval

        self.get_logger().info("Moved successfully.")
        return True  # Movement completed successfully

    def monitor_robot(self):
        # Define positions
        home_position = [-1.932, -1.858, -2.363, -0.411, 1.440, 0.030]
        target_position = [-0.698, -1.975, -1.571, -0.413, 1.457, 0.030]

        while rclpy.ok():
            if self.detect_person():
                self.get_logger().info("Person detected before movement! Stopping robot.")
                self.stop_robot()
                time.sleep(10)  # Halt for 10 seconds before resuming
                continue

            self.get_logger().info("Moving to home position.")
            success = self.move_to_position_with_interrupt(home_position, duration=5)
            if not success:
                continue  # Movement was interrupted, restart loop

            if self.detect_person():
                self.get_logger().info("Person detected before next movement! Stopping robot.")
                self.stop_robot()
                time.sleep(10)  # Halt for 10 seconds before resuming
                continue

            self.get_logger().info("Moving to target position.")
            success = self.move_to_position_with_interrupt(target_position, duration=5)
            if not success:
                continue  # Movement was interrupted, restart loop

    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Camera released.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()

    try:
        node.monitor_robot()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt.")
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

