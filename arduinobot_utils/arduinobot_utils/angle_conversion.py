#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AnglesConverter(Node):
    def __init__(self):
        super().__init__('angle_conversion_service_server')
        
        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, 'euler_to_quaternion', self.euler_to_quaternion_calback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, 'quaternion_to_euler', self.quaternion_to_euler_calback)

        self.get_logger().info('Angle conversion services ready')

    def euler_to_quaternion_calback(self, req: EulerToQuaternion.Request, res: QuaternionToEuler.Response):
        self.get_logger().info(f'Request to convert Euler angles roll: {req.roll}, pitch: {req.pitch}, yaw: {req.yaw}, into a Quaternion')
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info(f'Corresponding quartenion x:{res.x}, y:{res.y}, z{res.z}, w:{res.w}')
        return res

    def quaternion_to_euler_calback(self, req: QuaternionToEuler.Request, res: QuaternionToEuler.Response):
        self.get_logger().info(f'Request to convert Quaternion x: {req.x}, y: {req.y}, z: {req.z}, w: {req.w} into a Euler')
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info(f'Corresponding Euler angles roll:{res.roll}, pitch:{res.pitch}, yaw:{res.yaw}')
        return res



def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()