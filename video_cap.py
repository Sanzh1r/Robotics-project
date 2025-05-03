import cv2

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge



class VideoPublisher(Node):

    def __init__(self):

        super().__init__('video_publisher')

        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        

        if not self.cap.isOpened():

            self.get_logger().error("Failed to open camera")

            return

        

        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

        self.get_logger().info("Starting video stream publisher...")



    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:

            self.get_logger().error("Failed to grab frame")

            return

        

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        self.publisher_.publish(msg)



    def destroy_node(self):

        self.cap.release()

        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node = VideoPublisher()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()