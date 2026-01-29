import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pynput
from pynput.keyboard import Key

class MyNode(Node):
    def __init__(self):
        super().__init__('keyboard_input')
        
        # 1. Publisher (Write)
        self.publisher_ = self.create_publisher(String, 'topic0', 10)
        
        # 2. KEyboard
        self.keyboard = pynput.keyboard.Listener(on_press=self.on_key_press,on_release=self.on_key_release)
        self.keyboard.start()

        self.get_logger().info('keyboard_input has started!')

        self.keys = []

    def send_data(self):
        msg = String()
        linear_strength = 60
        rotational_strength = 5
        vector = [0,0,0,0,0,0]
        if 'w' in self.keys:
            vector[0] += linear_strength
        if 'a' in self.keys:
            vector[1] -= linear_strength
        if 's' in self.keys:
            vector[0] -= linear_strength
        if 'd' in self.keys:
            vector[1] += linear_strength
        if 'e' in self.keys:
            vector[2] += linear_strength
        if 'LEFT_SHIFT' in self.keys:
            vector[2] -= linear_strength
        if 'UP_ARROW' in self.keys:
            vector[4] -= rotational_strength
        if 'DOWN_ARROW' in self.keys:
            vector[4] += rotational_strength
        if 'LEFT_ARROW' in self.keys:
            vector[5] += rotational_strength
        if 'RIGHT_ARROW' in self.keys:
            vector[5] -= rotational_strength

        msg.data = f'{",".join(map(str, vector))}'
        self.publisher_.publish(msg)

    def on_key_press(self, key):
        key_press = None
        try:
            # Handle regular letters/numbers
            key_press = key.char
        except AttributeError:
            # Handle special keys
            if key == Key.space:
                key_press = "SPACE"
            elif key == Key.shift_l:
                key_press = "LEFT_SHIFT"
            elif key == Key.up:
                key_press = "UP_ARROW"
            elif key == Key.down:
                key_press = "DOWN_ARROW"
            elif key == Key.left:
                key_press = "LEFT_ARROW"
            elif key == Key.right:
                key_press = "RIGHT_ARROW"
            else:
                key_press = str(key)
        if not(key_press in self.keys):
            self.keys.append(key_press)
            self.send_data()

    def on_key_release(self, key):
        try:
            # Handle regular letters/numbers
            key_press = key.char
        except AttributeError:
            # Handle special keys
            if key == Key.space:
                key_press = "SPACE"
            elif key == Key.shift_l:
                key_press = "LEFT_SHIFT"
            elif key == Key.up:
                key_press = "UP_ARROW"
            elif key == Key.down:
                key_press = "DOWN_ARROW"
            elif key == Key.left:
                key_press = "LEFT_ARROW"
            elif key == Key.right:
                key_press = "RIGHT_ARROW"
            else:
                key_press = str(key)
        if key_press in self.keys:
            self.keys.remove(key_press)
            self.send_data()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()