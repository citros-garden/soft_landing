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
        self.declare_parameter('g', 1.62) 
        self.declare_parameter('um', 20.0)
        self.declare_parameter('e',0.02)
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
        self.e = self.get_parameter('e').get_parameter_value().double_value
        self.r_target = [self.setpoint_r_x,self.setpoint_r_y,self.setpoint_r_z]
        self.v_target = [self.setpoint_v_x,self.setpoint_v_y,self.setpoint_v_z]
        
    def state_cb(self, msg):
        self.r_tmp = msg.data[0:3]
        self.v_tmp = msg.data[3:6]
        self.r =np.array( [self.r_target[0]-self.r_tmp[0],self.r_target[1]-self.r_tmp[1],self.r_target[2]-self.r_tmp[2]])
        self.v =np.array( [self.v_target[0]-self.v_tmp[0],self.v_target[1]-self.v_tmp[1],self.v_target[2]-self.v_tmp[2]])
        tgo = self.vg.soft_landing_tgo_lq(self.r,self.v,self.um,self.g)[0]
        # self.get_logger().info(f"tgo is = {tgo:.3f}, r=  [{self.r[0]:.3f}, {self.r[1]:.3f}, {self.r[2]:.3f}],v= [{self.v[0]:.3f}, {self.v[1]:.3f}, {self.v[2]:.3f}]",throttle_duration_sec=0.1)
        self.miss_distance = np.linalg.norm(self.r)
        self.miss_velocity = np.linalg.norm(self.v)
        self.get_logger().info(f"tgo is = {tgo:.3f}, The miss distance is: [{self.miss_distance}] and the miss velocity is: [{self.miss_velocity}]",throttle_duration_sec=0.1)

        #change the massage to the miss distance and miss velocity and install foxglove
        if tgo > self.e:
            u = self.vg.soft_landing_controller_lq(self.r, self.v,tgo, self.g)
        else:
            u = [0,0,0]
            self.miss_distance = np.linalg.norm(self.r)
            self.miss_velocity = np.linalg.norm(self.v)

            self.get_logger().info(f"The miss distance is: [{self.miss_distance}] and the miss velocity is: [{self.miss_velocity}]")
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