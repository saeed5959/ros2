import rclpy
from rclpy.node import Node

from interfaces.srv import PercToCamera

class CameraService(Node):
    def __init__(self):
        super().__init__("camera")
        self.srv = self.create_service(PercToCamera, "camera_perception", self.camera_callback)

    def camera_callback(self,request, response):
        response.img_path = "./x.jpg"
        self.get_logger().info(f'Incoming request : {response.img_path}')

        return response



def main():
    rclpy.init()
    camera_service = CameraService()

    rclpy.spin(camera_service)

    rclpy.shutdown()



if __name__=="__main__":
    main()