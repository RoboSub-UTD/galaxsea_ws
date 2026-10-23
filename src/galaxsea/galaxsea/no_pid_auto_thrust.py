import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, pi
import numpy as np

from galaxsea_interfaces.msg import ThrusterCommands

class PIDController:
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.05, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_limit = output_limit
        self.integral_limit = integral_limit

    def update(self, error):
        self.integral += error * self.dt
        if self.integral_limit:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
        return output


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')

        self.declare_parameter('max_thruster_command', 1.0)
        self.declare_parameter('max_linear_velocity', 3.0)
        self.declare_parameter('max_linear_velocity_y', 3.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_thrust_force', 500.0) 

        self.declare_parameter('l1_angle', pi / 4)
        self.declare_parameter('r1_angle', -pi / 4)
        self.declare_parameter('l2_angle', 3 * pi / 4)
        self.declare_parameter('r2_angle', -3 * pi / 4)
        
        self.declare_parameter('l1_dist_x', 1.3)
        self.declare_parameter('l1_dist_y', 2.7)
        self.declare_parameter('r1_dist_x', 1.3)
        self.declare_parameter('r1_dist_y', -2.7)
        self.declare_parameter('l2_dist_x', -1.3)
        self.declare_parameter('l2_dist_y', 2.7)
        self.declare_parameter('r2_dist_x', -1.3)
        self.declare_parameter('r2_dist_y', -2.7)

        self.declare_parameter('linear_pid_kp', 250.0)
        self.declare_parameter('linear_pid_ki', 5.0)
        self.declare_parameter('linear_pid_kd', 20.0)
        self.declare_parameter('angular_pid_kp', 600.0)
        self.declare_parameter('angular_pid_ki', 10.0)
        self.declare_parameter('angular_pid_kd', 50.0)

        self.max_command = self.get_parameter('max_thruster_command').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_linear_velocity_y = self.get_parameter('max_linear_velocity_y').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_thrust_force = self.get_parameter('max_thrust_force').value

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular = 0.0

        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_velocity = 0.0

        dt = 1.0 / self.control_rate
        
        self.linear_x_pid = PIDController(
            kp=self.get_parameter('linear_pid_kp').value,
            ki=self.get_parameter('linear_pid_ki').value,
            kd=self.get_parameter('linear_pid_kd').value,
            dt=dt,
            output_limit=self.max_thrust_force,
            integral_limit=self.max_thrust_force * 0.3
        )

        self.linear_y_pid = PIDController(
            kp=self.get_parameter('linear_pid_kp').value,
            ki=self.get_parameter('linear_pid_ki').value,
            kd=self.get_parameter('linear_pid_kd').value,
            dt=dt,
            output_limit=self.max_thrust_force,
            integral_limit=self.max_thrust_force * 0.3
        )

        self.angular_pid = PIDController(
            kp=self.get_parameter('angular_pid_kp').value,
            ki=self.get_parameter('angular_pid_ki').value,
            kd=self.get_parameter('angular_pid_kd').value,
            dt=dt,
            output_limit=self.max_thrust_force,
            integral_limit=self.max_thrust_force * 0.2
        )

        columns = []
        self.thruster_order = ['l1', 'l2', 'r1', 'r2']
        
        for t in self.thruster_order:
            angle = self.get_parameter(f'{t}_angle').value
            dist_x = self.get_parameter(f'{t}_dist_x').value
            dist_y = self.get_parameter(f'{t}_dist_y').value

            fx = cos(angle)
            fy = sin(angle)
            torque = (dist_x * fy) - (dist_y * fx)

            columns.append([fx, fy, torque])

        A = np.array(columns).T  # Shape: (3, 4)
        self.pseudo_inv = np.linalg.pinv(A)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.thruster_pub = self.create_publisher(ThrusterCommands, '/thruster_commands', 10)
        self.create_timer(dt, self.control_loop)

    def odom_callback(self, msg):
        self.current_linear_x = msg.twist.twist.linear.x
        self.current_linear_y = msg.twist.twist.linear.y
        self.current_angular_velocity = msg.twist.twist.angular.z

    def cmd_vel_callback(self, msg):
        self.target_linear_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, msg.linear.x))
        self.target_linear_y = max(-self.max_linear_velocity_y, min(self.max_linear_velocity_y, msg.linear.y))
        self.target_angular = max(-self.max_angular_velocity, min(self.max_angular_velocity, msg.angular.z))

    def control_loop(self):
        linear_error_x = self.target_linear_x - self.current_linear_x
        linear_error_y = self.target_linear_y - self.current_linear_y
        angular_error = self.target_angular - self.current_angular_velocity

        effort_x = self.linear_x_pid.update(linear_error_x)
        effort_y = self.linear_y_pid.update(linear_error_y)
        effort_torque = self.angular_pid.update(angular_error)

        b = np.array([effort_y, effort_x, effort_torque])

        x = self.pseudo_inv @ b

        peak = np.max(np.abs(x))
        if peak > self.max_command:
            x = x * (self.max_command / peak)

        msg = ThrusterCommands()
        msg.front_left = float(-x[0])  
        msg.back_left = float(-x[1])   
        msg.front_right = float(-x[2]) 
        msg.back_right = float(-x[3])  

        self.thruster_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
