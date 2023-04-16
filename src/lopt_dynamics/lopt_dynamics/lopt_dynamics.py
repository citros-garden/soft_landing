import rclpy
from rclpy.node import Node
import time
import lopt.lopt_m

from lopt.lopt_m import solve

from std_msgs.msg import Float64MultiArray

class Lopt_dynamics(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/lopt_dynamics/state', 10)

        #Defining states
        #Initial state vector
        self.declare_parameter('x_x0')
        self.declare_parameter('x_y0')
        self.declare_parameter('x_z0')
        self.declare_parameter('vx_x0')
        self.declare_parameter('vx_y0')
        self.declare_parameter('vx_z0')

        #Target state vector
        self.declare_parameter('x_xf')
        self.declare_parameter('x_yf')
        self.declare_parameter('x_zf')
        self.declare_parameter('vx_xf')
        self.declare_parameter('vx_yf')
        self.declare_parameter('vx_zf')
        
        #Additional params
        self.declare_parameter('um') #control upper limit
        self.declare_parameter('ul') #control lower limit
        self.declare_parameter('g0') #gravity constant   

        self.declare_parameter('publish_freq', 0.1)     


        self.x_x0 = self.get_parameter('x_x0').get_parameter_value().double_value
        self.x_y0 = self.get_parameter('x_y0').get_parameter_value().double_value
        self.x_z0 = self.get_parameter('x_z0').get_parameter_value().double_value

        self.vx_x0 = self.get_parameter('vx_x0').get_parameter_value().double_value
        self.vx_y0 = self.get_parameter('vx_y0').get_parameter_value().double_value
        self.vx_z0 = self.get_parameter('vx_z0').get_parameter_value().double_value
        
        self.x_xf = self.get_parameter('x_xf').get_parameter_value().double_value
        self.x_yf = self.get_parameter('x_yf').get_parameter_value().double_value
        self.x_zf = self.get_parameter('x_zf').get_parameter_value().double_value

        self.vx_xf = self.get_parameter('vx_xf').get_parameter_value().double_value
        self.vx_yf = self.get_parameter('vx_yf').get_parameter_value().double_value
        self.vx_zf = self.get_parameter('vx_zf').get_parameter_value().double_value

        self.um = self.get_parameter('um').get_parameter_value().double_value
        self.ul = self.get_parameter('ul').get_parameter_value().double_value
        self.g0 = self.get_parameter('g0').get_parameter_value().double_value

        self.i = 0

        self.res_x, self.res_u, self.res_t = solve(dyn_func=dynamics0, term_cost=terminal_cost0, #runn_cost=running_cost0, 
         term_constr=terminal_constraints0, ocp_tf0=tf0, ocp_btf=btf, ocp_lbu=lbu, ocp_lbx=lbx, ocp_ubu=ubu, ocp_ubx=ubx, ocp_x00=x00, ocp_xf0=xf0)
        
        self.state_msg = Float64MultiArray()
        timer_period = self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #            [ x[0],  x[1],  x[2],  x[3],  x[4],  x[5] ]
        #            [   x,     y,     z,    Vx,    Vy,    Vz  ]
        tf0 = 15 # guess tf 
        xf0 = [ self.x_xf,  self.x_yf,  self.x_zf,    0,     0,     0  ] # target conditions
        x00 = [ self.x_x0,  self.x_y0,  self.x_z0,    0,     0,     0  ] #starting conditions
        lbx = [        0,         0,        0, -10000, -10000, -10000  ]
        ubx = [     1000,     10000,    10000,  10000,  10000,  10000  ]
        lbu = [  self.ul,   self.ul,   self.ul] #u_x, u_y, u_z
        ubu = [  self.um,   self.um,   self.um] #u_x, u_y, u_z
        btf = [1, 10000] # tf_min tf_max

        def terminal_cost0(xf, tf, x0, t0):
                return tf
    
        #def running_cost0(x, u, t):
        #    return u[0]

        def dynamics0(x, u, t): #Defining vessel dynamics 
                
                v_x = x[3]
                v_y = x[4]
                v_z = x[5]

                return [v_x, v_y, v_z, u[0], u[1], u[2] - self.g0]
        
        def terminal_constraints0(xf, tf, x0, t0): 

                tc = [  x0[0] - self.x_x0, 
                        x0[1] - self.x_y0,
                        x0[2] - self.x_z0, 
                        x0[3] - 0,
                        x0[4] - 0, 
                        x0[5] - 0,
                        
                        xf[0] - self.x_xf,
                        xf[1] - self.x_yf,
                        xf[2] - self.x_zf,
                        xf[3] - 0, 
                        xf[4] - 0, 
                        xf[5] - 0]
                return tc



    def timer_callback(self):

        self.state_msg.data = [self.res_x[self.i,0], self.res_x[self.i,1], self.res_x[self.i,2], self.res_x[self.i,3], self.res_x[self.i,4], self.res_x[self.i,5]]

        self.state_pub.publish(self.state_msg)
        self.get_logger().info(f"Publishing = {self.state_msg.data}")

        self.i += 1
        if self.i==len(self.res_t):
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = Lopt_dynamics()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()