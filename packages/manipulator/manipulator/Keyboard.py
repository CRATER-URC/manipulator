from pynput import keyboard
import rclpy
from rclpy.node         import Node
from geometry_msgs.msg  import PointStamped

RATE = 50.0            # Hertz
#AD controls x, WS controls y, IK controls z, JL rot about z, VB rot about x, NM rot about y

class GPSVisualizer(Node):

    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        
        
    
        
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)
    
    node = GPSVisualizer()

    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()