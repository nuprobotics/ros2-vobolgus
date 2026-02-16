import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriggerNode(Node):
    def __init__(self):
        super().__init__("trigger_node")
        self.declare_parameter("service_name", "/trigger_service")
        self.declare_parameter("default_string", "No service available")

        self.default_string = (
            self.get_parameter("default_string").get_parameter_value().string_value
        )
        service_name = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )

        self.stored_string = self.default_string

        self.client = self.create_client(Trigger, "/spgc/trigger")

        self.service = self.create_service(Trigger, service_name, self.service_callback)

        self.call_trigger_service()

    def call_trigger_service(self):
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "/spgc/trigger service not available, using default string"
            )
            self.stored_string = self.default_string
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.trigger_response_callback)

    def trigger_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.stored_string = response.message
                self.get_logger().info(
                    f'Stored string from /spgc/trigger: "{self.stored_string}"'
                )
            else:
                self.get_logger().warn(
                    "Trigger service returned failure, using default string"
                )
                self.stored_string = self.default_string
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.stored_string = self.default_string

    def service_callback(self, request, response):
        response.success = True
        response.message = self.stored_string
        self.get_logger().info(
            f'Service called, responding with: "{self.stored_string}"'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
