import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray  

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.u_pub = self.create_publisher(Float64MultiArray, '/controller/command', 10)

        self.state_sub = self.create_subscription(Float64MultiArray, '/dynamics/state', self.state_cb, 10)
       
        self.dt = 0.1  # seconds ?
        # define parameters        

        self.declare_parameter('setpoint_r_x')
        self.declare_parameter('setpoint_r_y')
        self.declare_parameter('setpoint_r_z')
        self.declare_parameter('setpoint_v_x')
        self.declare_parameter('setpoint_v_y')
        self.declare_parameter('setpoint_v_z')
        

        self.u_msg = Float64MultiArray() #[rx ,ry ,rz ,vx,vy,vz]
        self.state = Float64MultiArray() #[rx ,ry ,rz ,vx,vy,vz]


        time.sleep(1)

        self.setpoint_r_x = self.get_parameter('setpoint_r_x').get_parameter_value().double_value
        self.setpoint_r_y = self.get_parameter('setpoint_r_y').get_parameter_value().double_value
        self.setpoint_r_z = self.get_parameter('setpoint_r_z').get_parameter_value().double_value
        self.setpoint_v_x = self.get_parameter('setpoint_v_x').get_parameter_value().double_value
        self.setpoint_v_y = self.get_parameter('setpoint_v_y').get_parameter_value().double_value
        self.setpoint_v_z = self.get_parameter('setpoint_v_z').get_parameter_value().double_value

        time.sleep(1)
        self.get_logger().info(f"Starting controller with the setpoint r = {[self.setpoint_r_x,self.setpoint_r_y,self.setpoint_r_z]} and v = {[self.setpoint_v_x,self.setpoint_v_y,self.setpoint_v_z]}")


    def state_cb(self, msg):
        self.state = msg.data

   
def main(args=None):
    rclpy.init(args=args)
    controller = Pid()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()