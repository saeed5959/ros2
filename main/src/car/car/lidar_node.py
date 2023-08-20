import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class LidarService(Node):
    def __init__(self):
        super().__init__("lidar")
        self.srv = self.create_service(AddTwoInts, "lidar_perception", self.lidar_callback)

    def lidar_callback(self,request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response



def main():
    rclpy.init()
    lidar_service = LidarService()

    rclpy.spin(lidar_service)

    rclpy.shutdown()



if __name__=="__main__":
    main()