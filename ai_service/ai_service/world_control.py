from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl, WorldReset
import rclpy
from rclpy.node import Node

SERVICE_PARAMETER = 'world_control_target_service'

class WorldControlClient(Node):
    def __init__(self):
        super().__init__('world_control')
        
        # self.declare_parameter(SERVICE_PARAMETER)
        # svc = self.get_parameter(SERVICE_PARAMETER).get_parameter_value().string_value

        svc = '/world/buoyancy/control'

        self.cli = self.create_client(ControlWorld, svc)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ControlWorld.Request()
    
    def pause(self, pause: bool):
        self.req.world_control = WorldControl(pause=pause)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

    def reset(self):
        self.req.world_control = WorldControl(reset=WorldReset(all=True))
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def step(self, n: int = 0):
        self.req.world_control = WorldControl(step=True, multi_step=n)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = WorldControlClient()
    
    minimal_client.get_logger().info('Defaulting to reset command.')
    
    response = minimal_client.reset()

    minimal_client.get_logger().info(
        'Result of reset: %s' %
        (response))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()