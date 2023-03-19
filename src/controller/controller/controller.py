import rclpy
from rclpy.node import Node
import time
import pyvectorguidance

from std_msgs.msg import Float64MultiArray  

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.u_pub = self.create_publisher(Float64MultiArray, '/controller/command', 10)

        self.state_sub = self.create_subscription(Float64MultiArray, '/dynamics/state', self.state_cb, 10)
       
        self.dt = 0.1 
        self.vg = pyvectorguidance.VectorGuidance()
        # define parameters        

        self.declare_parameter('setpoint_r_x')
        self.declare_parameter('setpoint_r_y')
        self.declare_parameter('setpoint_r_z')
        self.declare_parameter('setpoint_v_x')
        self.declare_parameter('setpoint_v_y')
        self.declare_parameter('setpoint_v_z')
        self.declare_parameter('g') 
        self.declare_parameter('um')

        self.u_msg = Float64MultiArray() #[rx ,ry ,rz ,vx,vy,vz]
        self.state = [] #[rx ,ry ,rz ,vx,vy,vz]
        self.r = []
        self.v = []
        


        time.sleep(1)

        self.setpoint_r_x = self.get_parameter('setpoint_r_x').get_parameter_value().double_value
        self.setpoint_r_y = self.get_parameter('setpoint_r_y').get_parameter_value().double_value
        self.setpoint_r_z = self.get_parameter('setpoint_r_z').get_parameter_value().double_value
        self.setpoint_v_x = self.get_parameter('setpoint_v_x').get_parameter_value().double_value
        self.setpoint_v_y = self.get_parameter('setpoint_v_y').get_parameter_value().double_value
        self.setpoint_v_z = self.get_parameter('setpoint_v_z').get_parameter_value().double_value
        self.g = [0,0,self.get_parameter('setpoint_v_z').get_parameter_value().double_value]
        self.um = self.get_parameter('um').get_parameter_value().double_value


        time.sleep(1)
        self.get_logger().info(f"Starting controller with the setpoint r = {[self.setpoint_r_x,self.setpoint_r_y,self.setpoint_r_z]} and v = {[self.setpoint_v_x,self.setpoint_v_y,self.setpoint_v_z]}")


    def state_cb(self, msg):


        self.r = msg.data[0:2]
        self.v = msg.data[3:5]
        
        tgo = self.vg.soft_landing_tgo_lq(self.r, self.v, self.um, self.g)[0]
        u = self.vg.soft_landing_controller_lq(self.r, self.v, rho_u, tgo, self.g)
        
        self.u_msg.data = u
        self.u_pub.publish(self.u_msg)


   
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()