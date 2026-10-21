
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LIDAR_SUB(Node):
    def __init__(self):
        super().__init__('lidar_follow')
        self.get_logger().info('lidar_follow START !!')

        self.create_subscription(LaserScan,'/scan',self.cb_scan,10)

        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',1)

        self.scan_data = LaserScan()
        self.scan_recevice = False
        self.cmd_data = Twist()

        self.create_timer(0.3,self.cb_timer)        

        self.show_count = 1


    def cb_scan(self, msg):
        self.scan_data = msg
        self.scan_recevice =True
        pass

    def cb_timer(self):
        if not self.scan_recevice : 
            return
                
        index_range = 5

        def get_range(start_index,end_index) :
            d= self.scan_data.ranges[start_index : end_index  +1]
            d= [x for x in d if self.scan_data.range_min <= x <= self.scan_data.range_max ]
            if len(d)> 0 :
                d= sum(d) / len(d)
            else:
                d = 0.0
            return d

        d0= get_range(179 -index_range , 179 + index_range)
        dleft = get_range(179 , 179 + (index_range*2) )
        dright = get_range( 179 - (index_range*2) , 179 )

        if self.show_count >= 100 :
            self.show_count = 1
        else :
            self.show_count += 1

        self.get_logger().info(f'd:{d0:.2f} , dl:{dleft:.2f}, dr:{dright:.2f} , {self.show_count}')

        if 0.45 <= d0 <= 0.7:
            self.cmd_data.linear.x =0.05
            if dleft > dright and abs(dleft - dright) > 0.2 :
                self.cmd_data.angular.z = -0.2
            elif dleft < dright and abs(dleft - dright) > 0.2 :
                self.cmd_data.angular.z = 0.2
            else :
                self.cmd_data.angular.z = 0.0
        else :
            self.cmd_data = Twist()

        self.cmd_pub.publish(self.cmd_data)

        pass

def main():
    rclpy.init()
    node = LIDAR_SUB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        
if __name__ == '__main__':
    main()