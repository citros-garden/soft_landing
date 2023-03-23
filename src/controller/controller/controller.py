import rclpy
from rclpy.node import Node
import time
import pyvectorguidance
import numpy as np
from std_msgs.msg import Float64MultiArray  

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.u_pub = self.create_publisher(Float64MultiArray, '/controller/command', 10)
        self.state_sub = self.create_subscription(Float64MultiArray, '/dynamics/state', self.state_cb, 10)
        self.dt = 0.1 
        self.vg = pyvectorguidance.VectorGuidance()

        # define parameters        
        self.declare_parameter('setpoint_r_x', 0.0)
        self.declare_parameter('setpoint_r_y', 0.0)
        self.declare_parameter('setpoint_r_z', 0.0)
        self.declare_parameter('setpoint_v_x', 0.0)
        self.declare_parameter('setpoint_v_y', 0.0)
        self.declare_parameter('setpoint_v_z', 0.0)
        self.declare_parameter('g', 0.0) 
        self.declare_parameter('um', 0.0)
        
        # self.g = [0,0,0]
        # self.um = 0


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
        # self.get_logger().info(f"Starting controller with the setpoint r = {[self.setpoint_r_x,self.setpoint_r_y,self.setpoint_r_z]} and v = {[self.setpoint_v_x,self.setpoint_v_y,self.setpoint_v_z]}")

    def state_cb(self, msg):
        self.r_tmp = msg.data[0:3]
        self.v_tmp = msg.data[3:6]
        self.r =np.array( [self.r_tmp[0],self.r_tmp[1],self.r_tmp[2]])
        self.v =np.array( [self.v_tmp[0],self.v_tmp[1],self.v_tmp[2]])
        # self.get_logger().info(f"msg = {msg.data}, r= {self.r} ,v= {self.v}  ",throttle_duration_sec=0.5)
        # tgo = self.vg.soft_landing_tgo_lq(self.r, self.v, self.um, self.g) #positiv value

        norm_r = np.linalg.norm(self.r)
        norm_v = np.linalg.norm(self.v)
        tgo = norm_r/(norm_v+0.01)
        self.get_logger().info(f"tgo is = {tgo}, r= {self.r},v= {self.v}",throttle_duration_sec=0.5)

        try:
            u = self.vg.soft_landing_controller_lq(self.r, self.v,tgo, self.g)
        except:
            tgo = 0.0
            self.get_logger().info(f"tgo is = {tgo}, r= {self.r},v= {self.v}",throttle_duration_sec=0.5)
            exit()
        
        u = [float(x) for x in u]
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