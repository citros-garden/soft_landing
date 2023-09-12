import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64MultiArray 

class Dynamics(Node):

    def __init__(self):
        super().__init__('dynamics')
        self.state_pub = self.create_publisher(Float64MultiArray , '/dynamics/state', 10)

        self.u_sub = self.create_subscription(Float64MultiArray , '/controller/command', self.u_cb, 10)

        self.u_cmd =[0,0,0] #[ux ,uy ,uz ]
        self.state_msg = Float64MultiArray() #[rx ,ry ,rz ,vx,vy,vz]

        self.declare_parameters(
            namespace='',
            parameters=[('dt', 0.01),
                        ('g_x',0.0),
                        ('g_y',0.0),
                        ('g_z',1.62),
                        ('r_x0',0.0),
                        ('r_y0',0.0),
                        ('r_z0',0.0),
                        ('v_x0',0.0),
                        ('v_y0',0.0),
                        ('v_z0',0.0)])
        
        time.sleep(5)

        self.dt = self.get_parameter('dt').get_parameter_value().double_value
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
        self.state_msg.data = [self.r_x , self.r_y ,self.r_z ,self.v_x ,self.v_y, self.v_z]
        self.state_pub.publish(self.state_msg)
        if self.r_z < 0.01 and self.r_y < 0.01 and self.r_x < 0.01: # end when we leand
            exit()
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = Dynamics()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()