import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64MultiArray 

class Dynamics(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/dynamics/state', 10)
        self.u_sub = self.create_subscription(Float64MultiArray , '/controller/command', self.u_cb, 10)

        self.dt = 0.01  # seconds
        # define parameters

        self.declare_parameter('g_x')
        self.declare_parameter('g_y')
        self.declare_parameter('g_z')

        self.declare_parameter('r_x0') #r0
        self.declare_parameter('r_y0')
        self.declare_parameter('r_z0')
        
        self.declare_parameter('v_x0')
        self.declare_parameter('v_y0')
        self.declare_parameter('v_z0')
        

        self.u_cmd =[0,0,0] #[ux ,uy ,uz ]
        self.state_msg = Float64MultiArray() #[rx ,ry ,rz ,vx,vy,vz]
        

        time.sleep(1)

        self.g_x = self.get_parameter('g_x').get_parameter_value().double_value
        self.g_y = self.get_parameter('g_y').get_parameter_value().double_value
        self.g_z = self.get_parameter('g_z').get_parameter_value().double_value

        self.r_x = self.get_parameter('r_x0').get_parameter_value().double_value
        self.r_y = self.get_parameter('r_y0').get_parameter_value().double_value
        self.r_z = self.get_parameter('r_z0').get_parameter_value().double_value
        
        self.v_x = self.get_parameter('v_x0').get_parameter_value().double_value
        self.v_y = self.get_parameter('v_y0').get_parameter_value().double_value
        self.v_z = self.get_parameter('v_z0').get_parameter_value().double_value
       

        time.sleep(1)

        self.timer = self.create_timer(self.dt, self.step)

    def u_cb(self, msg):
        self.u_cmd = msg.data #return list

    def solve_ode(self):


        self.v_x += ((self.u_cmd[0] - self.g_x) * self.dt) # Integral(u - g)+v0 = v
        self.v_y += ((self.u_cmd[1] - self.g_y) * self.dt)
        self.v_z += ((self.u_cmd[2] - self.g_z) * self.dt)
       
            

        self.r_x += (self.v_x * self.dt) # Integral(v)+r0 = x
        self.r_y += (self.v_y * self.dt)
        self.r_z += (self.v_z * self.dt)
        self.r_z = max(0.0,self.r_z)
 
        if self.r_z <= 0.0:
            self.v_z == max(0.0,self.v_z)

    def step(self):

        self.solve_ode()


        self.get_logger().info(f"pose: {[self.r_x , self.r_y ,self.r_z]} velocity {[self.v_x , self.v_y ,self.v_z]}, controller: {self.u_cmd}", throttle_duration_sec=0.5)

        self.state_msg.data = [self.r_x , self.r_y ,self.r_z ,self.v_x ,self.v_y, self.v_z]
        self.state_pub.publish(self.state_msg)
        
        if self.r_z == 0.0 : # end when we leand
            self.get_logger().info(f"r: {[self.r_x , self.r_y ,self.r_z]} velocity {[self.v_x , self.v_y ,self.v_z]}, controller: {self.u_cmd[1]}", throttle_duration_sec=0.5)
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = Dynamics()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()