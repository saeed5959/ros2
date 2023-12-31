import sys

from interfaces.srv import PercToCamera
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class PerceptionToCamera(Node):

    def __init__(self):
        super().__init__("perception")
        self.cli = self.create_client(PercToCamera, 'camera_perception')
        self.req = PercToCamera.Request()

    def send_request(self):
        self.req.m = "hi"
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = PerceptionToCamera()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(f'img path : {response.img_path}')
    

    minimal_client.destroy_node()
    rclpy.shutdown()

###########################################

class PerceptionService(Node):

    def __init__(self):
        super().__init__("perception")
        self.camera_node = rclpy.create_node('camera_node')
        self.cli_camera = self.camera_node.create_client(AddTwoInts, 'camera_perception')
        self.req_camera = AddTwoInts.Request()

        self.lidar_node = rclpy.create_node('lidar_node')
        self.cli_lidar = self.lidar_node.create_client(AddTwoInts, 'lidar_perception')
        self.req_lidar = AddTwoInts.Request()

        self.srv = self.create_service(AddTwoInts, "perception_head", self.head_callback)

    def send_request_camera(self, a, b):
        self.req_camera.a = a
        self.req_camera.b = b
        self.future_camera = self.cli_camera.call_async(self.req_camera)
        rclpy.spin_until_future_complete(self.camera_node, self.future_camera)
        return self.future_camera.result()
    
    def send_request_lidar(self, a, b):
        self.req_lidar.a = a
        self.req_lidar.b = b
        self.future_lidar = self.cli_lidar.call_async(self.req_lidar)
        rclpy.spin_until_future_complete(self.lidar_node, self.future_lidar)
        return self.future_lidar.result()
    
    def head_callback(self, request, response):
        response_camera = self.send_request_camera(request.a , request.b)
        response_lidar = self.send_request_lidar(request.a , request.b)

        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} : {request.b}')
        self.get_logger().info(f'Incoming request: {response_camera.sum} : {response_lidar.sum}')

        return response

def main_1():
    rclpy.init()

    perception_service = PerceptionService()
    rclpy.spin(perception_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()