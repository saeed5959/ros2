import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Head(Node):

    def __init__(self):
        super().__init__("head")
        self.publisher_steer = self.create_publisher(String, 'head_steer', 10)
        timer_period = 0.5  # seconds
        self.timer_steer = self.create_timer(timer_period, self.timer_callback_steer)

        self.publisher_velocity = self.create_publisher(String, 'head_velocity', 10)
        self.timer_velocity = self.create_timer(timer_period, self.timer_callback_velocity)

        self.perception_in_head = rclpy.create_node('perception_in_head')
        self.cli_perception = self.perception_in_head.create_client(AddTwoInts, 'perception_head')
        self.req_perception = AddTwoInts.Request()

        self.steer_sensor_in_head = rclpy.create_node('steer_sensor_in_head')
        self.cli_steer_sensor = self.steer_sensor_in_head.create_client(AddTwoInts, 'steer_sensor_head')
        self.req_steer_sensor = AddTwoInts.Request()

        self.velocity_sensor_in_head = rclpy.create_node('velocity_sensor_in_head')
        self.cli_velocity_sensor = self.velocity_sensor_in_head.create_client(AddTwoInts, 'velocity_sensor_head')
        self.req_velocity_sensor = AddTwoInts.Request()


    def send_request_perception(self, a, b):
        self.req_perception.a = a
        self.req_perception.b = b
        self.future_perception = self.cli_perception.call_async(self.req_perception)
        rclpy.spin_until_future_complete(self.perception_in_head, self.future_perception)
        return self.future_perception.result()
    
    def send_request_steer_sensor(self, a, b):
        self.req_steer_sensor.a = a
        self.req_steer_sensor.b = b
        self.future_steer_sensor = self.cli_steer_sensor.call_async(self.req_steer_sensor)
        rclpy.spin_until_future_complete(self.steer_sensor_in_head, self.future_steer_sensor)
        return self.future_steer_sensor.result()
    
    def send_request_velocity_sensor(self, a, b):
        self.req_velocity_sensor.a = a
        self.req_velocity_sensor.b = b
        self.future_velocity_sensor = self.cli_velocity_sensor.call_async(self.req_velocity_sensor)
        rclpy.spin_until_future_complete(self.velocity_sensor_in_head, self.future_velocity_sensor)
        return self.future_velocity_sensor.result()

    def timer_callback_steer(self):
        response_perception = self.send_request_perception(1,2)
        response_steer_sensor = self.send_request_steer_sensor(1,2)
        response_velocity_sensor = self.send_request_velocity_sensor(1,2)

        msg = String()
        msg.data = 'Hello World'
        self.publisher_steer.publish(msg)
        self.get_logger().info(f'Publishing in steer : {msg.data}')

    def timer_callback_velocity(self):
        response_perception = self.send_request_perception(1,2)
        response_steer_sensor = self.send_request_steer_sensor(1,2)
        response_velocity_sensor = self.send_request_velocity_sensor(1,2)

        msg = String()
        msg.data = 'Hello World'
        self.publisher_velocity.publish(msg)
        self.get_logger().info(f'Publishing in velocity : {msg.data}')



def main():
    rclpy.init()

    minimal_client = Head()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info('Result of add_two_ints: for {int(sys.argv[1])} , {int(sys.argv[2])} , {response.sum)}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()