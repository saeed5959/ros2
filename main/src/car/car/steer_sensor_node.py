import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class SteerService(Node):
    def __init__(self):
        super().__init__("steer_sensor")
        self.srv = self.create_service(AddTwoInts, "steer_sensor_head", self.steer_callback)

    def steer_callback(self,request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response



def main():
    rclpy.init()
    steer_service = SteerService()

    rclpy.spin(steer_service)

    rclpy.shutdown()



if __name__=="__main__":
    main()