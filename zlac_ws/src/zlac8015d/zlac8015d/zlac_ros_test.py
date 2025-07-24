# from zlac_controller import ZlacController
from zlac8015d.zlac_controller import ZlacController
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
import threading
import time

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
DRIVER_ID1 = 1

class ZlacTest(Node):
    def __init__(self):
        super().__init__('zlac_move')
        
        self.lock = threading.Lock()
        
        self.vel = 0
        
        self.vel_sub = self.create_subscription(
            Int32,
            'velocity',
            self.velocity_callback,
            10
            )
        
        self.init_zlac()
    
    def init_zlac(self):
        self.zlac = ZlacController(port_name=PORT, baudrate=BAUDRATE, driver_id=DRIVER_ID1)
        time.sleep(1)
        self.get_logger().info("Zlac Dirver ready")
    
    def velocity_callback(self, data):
        with self.lock:
            self.vel = data.data
        
        self.zlac.Target_velocity_sync(self.vel, -self.vel)
    
    def connect(self):
        self.zlac.Control_word(0x08)
        self.zlac.Control_mode(0x03)
    
    def stop(self):
        self.zlac.Control_word(0x07)
    
def main(args=None):
    rclpy.init(args=args)
    zlactest = ZlacTest()
    
    executor = MultiThreadedExecutor()
    executor.add_node(zlactest)
    
    zlactest.connect()
    
    try:
        executor.spin()
    finally:
        zlactest.stop()
        zlactest.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()