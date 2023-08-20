import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class VelocityService(Node):
    def __init__(self):
        super().__init__("velocity_sensor")
        self.srv = self.create_service(AddTwoInts, "velocity_sensor_head", self.velocity_callback)

    def velocity_callback(self,request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response



def main():
    rclpy.init()
    velocity_service = VelocityService()

    rclpy.spin(velocity_service)

    rclpy.shutdown()



if __name__=="__main__":
    main()