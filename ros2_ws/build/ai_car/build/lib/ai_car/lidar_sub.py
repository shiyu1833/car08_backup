#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class LIDAR_SUB(Node):
    def __init__(self):
        super().__init__('lidar_sub')
        self.get_logger().info('lidar_sub START !!')

        # TODO 建立以下一個 Subscriber，綁定對應的 callback
        #   /scan   (LaserScan)   → cb_scan,   queue=10

        self.create_subscription(LaserScan,'/scan', self.cb_scan,10)

        self.scan_received = False

        self.show_count = 1
        self.scan_data = LaserScan()
        self.create_timer(1, self.cb_timer)

    def cb_scan(self, msg):
        # TODO 在此 callback 中儲存接收到的 LaserScan 訊息，並將 scan_received 標記為 True
        #   self.scan_data = ???
        self.scan_received = True
        self.scan_data = msg

        pass

    def cb_timer(self):
        if not self.scan_received:
            return

        # =====================================================================
        # 【練習一】按角度讀取Index索引值及光達距離
        #  顯示 出角度 - 索引 - 距離 對應數據，並將 inf 數值顯示 nan
        # =====================================================================

        index_count = len(self.scan_data.ranges)

        # TODO 
        # 1：計算各方向的 index，i0、i90、i180、i270 分別對應 0、90、180、270 度
        # 2：取出各方向的單點距離 d0、d90、d180、d270
        # 3：無效值處理
        #       超出 range_min ~ range_max 的數值設為 float('nan')
        #       RPLIDAR A1M8：range_min=0.15m, range_max=6.0m
        # 4：印出結果
        #   範例輸出格式：
        #     lidar 筆數: 360
        #     [0度]   index[180], 單點距離: 0.85 m
        #     [90度]  index[270], 單點距離: 1.20 m
        #     [180度] index[0],   單點距離: nan m
        #     [270度] index[90],  單點距離: 0.42 m
        #     角度 = angle_min + index × angle_increment
        #     index = (角度 - angle_min) / angle_increment

        i0 = round ( (math.radians(0) - self.scan_data.angle_min) /self.scan_data.angle_increment )
        i90 = round ( (math.radians(90) - self.scan_data.angle_min) /self.scan_data.angle_increment )
        i180 = round ( (math.radians(180) - self.scan_data.angle_min) /self.scan_data.angle_increment )
        i270 = round ( (math.radians(-90) - self.scan_data.angle_min) /self.scan_data.angle_increment )
        
        d0 = self.scan_data.ranges[i0]   # 179
        d90 = self.scan_data.ranges[i90]   # 179
        d180 = self.scan_data.ranges[i180]   # 179
        d270 = self.scan_data.ranges[i270]   # 179
        
        if not self.scan_data.range_min <= d0 <= self.scan_data.range_max : d0 =  float('nan')  
        if not self.scan_data.range_min <= d90 <= self.scan_data.range_max : d90 =  float('nan')  
        if not self.scan_data.range_min <= d180 <= self.scan_data.range_max : d180 =  float('nan')  
        if not self.scan_data.range_min <= d270 <= self.scan_data.range_max : d270 =  float('nan')  


        # print(f'lider count: {len(self.scan_data.ranges)} \n' 
        #      f'[0] index[{i0}] ,{d0:.2f}  M' )
        
        # print(f'lider count: {len(self.scan_data.ranges)} \n' 
        #      f'[90] index[{i90}] ,{d90:.2f}  M' )
        
        # print(f'lider count: {len(self.scan_data.ranges)} \n' 
        #      f'[180] index[{i180}] ,{d180:.2f}  M' )
        
        # print(f'lider count: {len(self.scan_data.ranges)} \n' 
        #      f'[270] index[{i270}] ,{d270:.2f}  M' )


        # =====================================================================
        # 【練習二】取扇形數據平均讓數值更穩定
        # =====================================================================

        index_range = 5  # 前後各取幾點

        # TODO 
        # 1：使用切片取出各方向的 Index 扇形範圍，d0_avg、d90_avg、d180_avg、d270_avg
        # 2：過濾各陣列中的無效值
        # 3：計算平均值
        #       若過濾後陣列為空，設為 float('nan')
        # 4：印出結果
        #   範例輸出格式：
        #     lidar 筆數: 360
        #     [0度]   index[180], 單點距離: 0.85 m, 平均距離: 0.83 m
        #     [90度]  index[270], 單點距離: 1.20 m, 平均距離: 1.18 m
        #     [180度] index[0],   單點距離: nan m,  平均距離: nan m
        #     [270度] index[90],  單點距離: 0.42 m, 平均距離: 0.44 m

        d0_avg = self.scan_data.ranges[i0 - index_range: i0 + index_range + 1 ]
        d90_avg = self.scan_data.ranges[i90 - index_range: i90 + index_range + 1 ]
        d180_avg = self.scan_data.ranges[i180 - index_range: i180 + index_range + 1 ]
        d270_avg = self.scan_data.ranges[i270 - index_range: i270 + index_range + 1 ]
        
        d0_avg = [x for x in d0_avg if self.scan_data.angle_min <= x <= self.scan_data.angle_max ]
        d90_avg = [x for x in d90_avg if self.scan_data.angle_min <= x <= self.scan_data.angle_max ]
        d180_avg = [x for x in d180_avg if self.scan_data.angle_min <= x <= self.scan_data.angle_max ]
        d270_avg = [x for x in d270_avg if self.scan_data.angle_min <= x <= self.scan_data.angle_max ]

        if len(d0_avg) > 0 :  d0_avg = sum (d0_avg) / len (d0_avg)
        else : d0_avg = float('nan')

        if len(d90_avg) > 0 :  d90_avg = sum (d90_avg) / len (d90_avg)
        else : d90_avg = float('nan')

        if len(d180_avg) > 0 :  d180_avg = sum (d180_avg) / len (d180_avg)
        else : d180_avg = float('nan')

        if len(d270_avg) > 0 :  d270_avg = sum (d270_avg) / len (d270_avg)
        else : d270_avg = float('nan')

        print  (f'lidar count :{ len(self.scan_data.ranges)} \n'
                f'{self.show_count}.[0度]    index [{i0}] , 單點距離: {d0:.2f} ,平均距離: {d0_avg:.2f} \n'
                f'{self.show_count}.[90度]    index [{i90}] , 單點距離: {d90:.2f} 平均距離: {d90_avg:.2f} \n'
                f'{self.show_count}.[180度]    index [{i180}] , 單點距離: {d180:.2f} 平均距離: {d180_avg:.2f} \n' 
                f'{self.show_count}.[270度]    index [{i270}] , 單點距離: {d270:.2f} 平均距離: {d270_avg:.2f} \n' )

        if self.show_count > 100 : self.show_count =1 
        else : self.show_count += 1

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
