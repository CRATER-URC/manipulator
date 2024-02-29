from pynput import keyboard
import rclpy
from rclpy.node         import Node
from std_msgs.msg import String

RATE = 50.0            # Hertz

data = ""
keys = "ADWSIKJLVBNM"
#AD controls x, WS controls y, IK controls z, JL rot about z, VB rot about x, NM rot about y

class InputPublisher(Node):

    def __init__(self):
        super().__init__('keyboard')
        self.publisher_ = self.create_publisher(String, '/key_input', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        global data
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
def on_press(key):
    global data
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in keys:
        data = k
    else:
        data = ""
    print("pressed", data)

# define key release event callback
def on_release(key):
    global data
    data = ""
    # stop on PAUSE
    if key == keyboard.Key.pause:
        print("quit on PAUSE")
        return False 
        
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)
    
    pub = InputPublisher()
    
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while listener.running:
        rclpy.spin_once(pub)

    # Shutdown the node and ROS.
    listener.stop()
    pub.shutdown()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()