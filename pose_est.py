import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')

        self.subscription = self.create_subscription(

            Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher_ = self.create_publisher(Image, '/pose_estimation/output_image', 10)

        self.bridge = CvBridge()



        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

        self.parameters = cv2.aruco.DetectorParameters()

        self.intrinsic_camera = np.array(((933.15867, 0, 657.59), 

                                         (0, 933.1586, 400.36993), 

                                         (0, 0, 1)))

        self.distortion = np.array((-0.43948, 0.18514, 0, 0))

        

        self.marker_size = 0.02

        self.obj_points = np.array([

            [-self.marker_size / 2, self.marker_size / 2, 0],

            [self.marker_size / 2, self.marker_size / 2, 0],

            [self.marker_size / 2, -self.marker_size / 2, 0],

            [-self.marker_size / 2, -self.marker_size / 2, 0]

        ], dtype=np.float32)

        

        self.get_logger().info(f"OpenCV version: {cv2.__version__}")

        self.get_logger().info("Pose estimation node started...")



    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        

        output = self.pose_estimation(frame)

        

        output_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")

        self.publisher_.publish(output_msg)



    def pose_estimation(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        corners, ids, rejected = detector.detectMarkers(gray)

        

        if ids is not None and len(corners) > 0:

            for i in range(len(ids)):

                img_points = corners[i][0]

                

                success, rvec, tvec = cv2.solvePnP(

                    self.obj_points, img_points, self.intrinsic_camera, self.distortion)

                

                if success:

                    cv2.aruco.drawDetectedMarkers(frame, corners)

                    cv2.drawFrameAxes(frame, self.intrinsic_camera, self.distortion, rvec, tvec, 0.01)

                    

                    self.get_logger().info(f"Marker ID: {ids[i]}")

                    self.get_logger().info(f"Translation (tvec): x={tvec[0][0]:.4f}, y={tvec[1][0]:.4f}, z={tvec[2][0]:.4f} meters")

                    self.get_logger().info(f"Rotation (rvec): x={rvec[0][0]:.4f}, y={rvec[1][0]:.4f}, z={rvec[2][0]:.4f} radians")

                else:

                    self.get_logger().warning(f"Pose estimation failed for Marker ID: {ids[i]}")

        

        return frame



    def destroy_node(self):

        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node = PoseEstimationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()
