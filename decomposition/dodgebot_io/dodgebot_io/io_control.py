import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from operation_interface.msg import IoInfo
import Jetson.GPIO as GPIO
import threading
import time

GPIO.setmode(GPIO.BOARD) 
button_pin = 7

class IoControl(Node):

    def __init__(self):
        super().__init__('io_control')
        self.publisher_ = self.create_publisher(IoInfo, 'dodgebot_io', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.io_msg = IoInfo()

    def timer_callback(self):
        self.publisher_.publish(self.io_msg)
    
    def button_loop(self):
        # GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=blink, bouncetime=10, polltime=0.2)
        while True:
            if GPIO.input(button_pin) == GPIO.LOW:
                self.io_msg.armor_button = True
                time.sleep(0.5)
            else:
                self.io_msg.armor_button = False   

def main(args=None):

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(button_pin, GPIO.IN)
    rclpy.init(args=args)

    io_control_node = IoControl()
    button_thread = threading.Thread(target=io_control_node.button_loop)
    button_thread.start()

    rclpy.spin(io_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    button_thread.join()
    io_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main