import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray  

class Pid(Node):

    def __init__(self):
        super().__init__('pid')
        self.u_pub = self.create_publisher(Float64, '/controller/command', 10)

        self.state_sub = self.create_subscription(Float64MultiArray, '/dynamics/state', self.state_cb, 10)
       
        self.dt = 0.1  # seconds ?
        # define parameters

        self.declare_parameter('kp')
        self.declare_parameter('ki')
        self.declare_parameter('kd')
        self.declare_parameter('setpoint')

        self.u_msg = Float64() #its vektor?

        self.state = [0 , 0 , 0 , 0 , 0 , 0] #[rx ,ry ,rz ,vx,vy,vz]
    

        self._last_error = 0.0
        self._last_pose_feedback = 0.0

        time.sleep(1)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.setpoint = self.get_parameter('setpoint').get_parameter_value().double_value

        time.sleep(1)
        self.get_logger().info(f"Starting PID controller with: Kp = {self.kp}, Ki = {self.ki}, Kd = {self.kd}")

        self.timer = self.create_timer(self.dt, self.step)

    def state_cb(self, msg):
        self.state = msg.data

    # def vel_cb(self, msg):
    #     self.vel = msg.data

    def step(self):
        error = self.setpoint - self.state
        i_error = self._last_error + self.ki*error*self.dt
        d_error = -(self.state - self._last_pose_feedback)/self.dt

        p_element = self.kp * (error)
        i_element = self.ki * (i_error)
        d_element = self.kd * (d_error)
        
        self.u_msg.data = p_element + i_element + d_element

        self.u_pub.publish(self.u_msg)

        self._last_error = error
        self._last_pose_feedback = self.state

def main(args=None):
    rclpy.init(args=args)
    controller = Pid()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()