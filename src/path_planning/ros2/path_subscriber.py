#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from tests.bayesian_inference_tests_csv import Cone

class PathSubscriber(Node):

    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/fsds/cameracam1/image_color',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, path):
        self.get_logger().info('I heard: "%d"' % path.width)
        img = self.br.imgmsg_to_cv2(path)
        #Processamento de imagem aqui
        cv2.imshow('Right', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    path_subscriber = PathSubscriber()

  
