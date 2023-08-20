import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class CameraService(Node):
    def __init__(self):
        super().__init__("camera")
        self.srv = self.create_service(AddTwoInts, "camera_perception", self.camera_callback)

    def camera_callback(self,request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response



def main():
    rclpy.init()
    camera_service = CameraService()

    rclpy.spin(camera_service)

    rclpy.shutdown()



if __name__=="__main__":
    main()