import rclpy
from rclpy.node import Node
import time
import pyvectorguidance
import numpy as np
from std_msgs.msg import Float64MultiArray , Float64 

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.u_pub = self.create_publisher(Float64MultiArray, '/controller/command', 10)
        self.miss_distance_pub = self.create_publisher(Float64, '/controller/miss_distance', 10)
        self.miss_velocity_pub = self.create_publisher(Float64, '/controller/miss_velocity', 10)
        self.t_go_pub = self.create_publisher(Float64, '/controller/t_go', 10)

        self.state_sub = self.create_subscription(Float64MultiArray, '/dynamics/state', self.state_cb, 10)

        self.vg = pyvectorguidance.VectorGuidance()

        # define parameters
        self.declare_parameters(
            namespace='',
            parameters=[('dt', 0.01),
                        ('setpoint_r_x',0.0),
                        ('setpoint_r_y',0.0),
                        ('setpoint_r_z',0.0),
                        ('setpoint_v_x',0.0),
                        ('setpoint_v_y',0.0),
                        ('setpoint_v_z',0.0),
                        ('g',1.62),
                        ('um', 20.0),
                        ('e',0.02)])
      
        self.u_msg = Float64MultiArray() #[rx ,ry ,rz ,vx,vy,vz]
        self.state = [] #[rx ,ry ,rz ,vx,vy,vz]
        self.r = []
        self.v = []
        self.miss_distance_msg = Float64()
        self.miss_velocity_msg = Float64()
        self.t_go_msg = Float64()

        time.sleep(1)

        self.dt = self.get_parameter('dt').get_parameter_value().double_value
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

        self.get_logger().info(f"e = {self.e}")
    def state_cb(self, msg):
        r_tmp = msg.data[0:3]
        v_tmp = msg.data[3:6]
        r =np.array( [self.r_target[0]-r_tmp[0],self.r_target[1]-r_tmp[1],self.r_target[2]-r_tmp[2]])
        v =np.array( [self.v_target[0]-v_tmp[0],self.v_target[1]-v_tmp[1],self.v_target[2]-v_tmp[2]])
        tgo = self.vg.soft_landing_tgo_lq(r,v,self.um,self.g)[0]
        miss_distance = np.linalg.norm(r)
        miss_velocity= np.linalg.norm(v)
        self.get_logger().info(f"tgo is = {tgo:.3f}, The miss distance is: [{miss_distance:.3f}] and the miss velocity is: [{miss_velocity:.3f}]",throttle_duration_sec=1)
        if tgo > self.e:
            u = self.vg.soft_landing_controller_lq(r, v,tgo, self.g)
        else:
            u = [0,0,0]
            miss_distance = np.linalg.norm(r)
            miss_velocity= np.linalg.norm(v)
            self.get_logger().info(f"The miss distance is: [{miss_distance}] and the miss velocity is: [{miss_velocity}]")
            exit()
        u = [float(x) for x in u]
        self.u_msg.data = u
        self.miss_distance_msg.data = miss_distance
        self.miss_velocity_msg.data = miss_velocity
        self.t_go_msg.data = tgo
        self.u_pub.publish(self.u_msg)
        self.miss_distance_pub.publish(self.miss_distance_msg)
        self.miss_velocity_pub.publish(self.miss_velocity_msg)
        self.t_go_pub.publish(self.t_go_msg)
       
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()