import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import Float64

class MyNode(Node):
    def __init__(self):
        super().__init__('reciever')

        self.publishers_ = []
        for i in range(8):
            topic = f'/rov/thruster/t{i}'
            pub = self.create_publisher(Float64, topic, 10)
            self.publishers_.append(pub)
        
        # 2. Subscriber (Read)
        self.subscription = self.create_subscription(String, 'topic0', self.listener_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)


        self.get_logger().info('reciever has started!')

        motor_thrust_vectors = np.array([
            [1, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]])
        motor_position_vectors = np.array([
            [0, 0.25, 0],
            [0, -0.25, 0],
            [0.25, 0, 0],
            [-0.25, 0, 0],
            [0.25, -0.25, 0],
            [0.25, 0.25, 0],
            [-0.25, 0.25, 0],
            [-0.25, -0.25, 0]
        ])
        motor_rotation_vectors = []
        for i in range(motor_thrust_vectors.shape[0]):
            motor_rotation_vectors.append(np.cross(motor_thrust_vectors[i], motor_position_vectors[i]))
        motor_rotation_vectors = np.array(motor_rotation_vectors)
        self.motor_vectors = np.append(motor_thrust_vectors, motor_rotation_vectors, axis=1).T
        self.thruster_forces = [0]*8

    def listener_callback(self, msg):
        goal_vector = np.array(list(map(float, msg.data.split(','))))
        thruster_forces = np.linalg.lstsq(self.motor_vectors, goal_vector)[0]
        thruster_forces = thruster_forces.tolist()
        self.thruster_forces = thruster_forces
        for i in range(len(thruster_forces)):
            thruster_forces[i] = str(round(thruster_forces[i], 2))
        self.get_logger().info(f"thruster force: {thruster_forces}")

    def timer_callback(self):
        for i in range(len(self.thruster_forces)):
            value = float(self.thruster_forces[i])
            self.publishers_[i].publish(Float64(data=value))

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