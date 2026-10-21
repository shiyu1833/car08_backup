from dynamixel_sdk import PortHandler
from dxl_arm import Arm_Controller
from dxl_wheel import Wheel_Controller
import time

port = PortHandler('/dev/U2D2')
port.openPort()
port.setBaudRate(1000000)

arm = Arm_Controller (port)

arm.set_tool(0)
arm.set_joint([90,90])

time.sleep(2)

arm.reset()






