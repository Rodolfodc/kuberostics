"""Dummy lifecycle node for tests and for an empty optional manager (T521, 005-R38).

Does nothing; its only behaviour is the test-only parameter `fail_on_activate` (bool, default false): when
true, the transition to active fails, so the manager that owns the node cannot bring it up.
"""
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class DummyLifecycleNode(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__('dummy_lifecycle_node', **kwargs)
        self.declare_parameter('fail_on_activate', False)

    def on_activate(self, state):
        if self.get_parameter('fail_on_activate').value:
            self.get_logger().error('fail_on_activate is true: activation refused (test helper)')
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)


def main(args=None):
    rclpy.init(args=args)
    node = DummyLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
