import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Initialize global variables
mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# PID Controller constants
Kp = 1.0  # Proportional gain
Ki = 0.0  # Integral gain
Kd = 0.2  # Derivative gain

# PID variables
last_error = 0
integral = 0
target_distance = 0.25  

def timer_callback():
    global pub_, twstmsg_
    if twstmsg_ is not None:
        pub_.publish(twstmsg_)

def clbk_laser(msg):
    global regions_, twstmsg_
   
    regions_ = {
        'front1': find_nearest(msg.ranges[0:5]),
        'front2': find_nearest(msg.ranges[355:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[310:320]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95])
    }
    twstmsg_ = movement()

def find_nearest(lst):
    valid_points = list(filter(lambda item: item > 0.0, lst))  # exclude zero 
    return min(min(valid_points, default=10), 10)  # Return the minimum distance 

# PID controller for wall following
def pid_control(error):
    global last_error, integral
    
    proportional = Kp * error 
    
    integral += error 
    integral_term = Ki * integral   
 
    derivative = error - last_error  
    derivative_term = Kd * derivative
    
    output = proportional + integral_term + derivative_term
    last_error = error
    
    return output
def movement():
    global regions_, mynode_
    regions = regions_
    
    print(f"Min distance in right region: {regions_['right']}")
    msg = Twist()

    current_distance = regions_['right']    
    error = target_distance - current_distance 
    angular_velocity = pid_control(error)
    
    if regions_['front1'] < 0.5 or regions_['front2'] < 0.5:
        msg.linear.x = 0.0  
        msg.angular.z = 0.5  
    else:        
        msg.linear.x = 0.1
        msg.angular.z = angular_velocity
    
    return msg

def stop():
    global pub_
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub_.publish(msg)

def main():
    global pub_, mynode_
    rclpy.init()
    mynode_ = rclpy.create_node('right_edge_following')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
  
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)
    
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  
    except:
        stop()  
    finally:
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

