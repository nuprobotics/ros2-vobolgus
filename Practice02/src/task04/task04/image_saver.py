import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__("image_saver")
        self.declare_parameter("topic_name", "/camera/image/compressed")
        self.declare_parameter("output_dir", "/tmp/saved_images")

        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        os.makedirs(self.output_dir, exist_ok=True)

        self.saving = True
        self.image_count = 0

        self.subscription = self.create_subscription(
            CompressedImage, topic_name, self.image_callback, 10
        )

        self.stop_service = self.create_service(Trigger, "stop", self.stop_callback)

        self.get_logger().info(
            f'Image saver started. Subscribing to "{topic_name}", '
            f'saving to "{self.output_dir}"'
        )

    def image_callback(self, msg: CompressedImage):
        if not self.saving:
            return

        fmt = msg.format.lower()
        if "jpeg" in fmt or "jpg" in fmt:
            ext = ".jpg"
        elif "png" in fmt:
            ext = ".png"
        else:
            ext = ".jpg"

        filename = os.path.join(self.output_dir, f"image_{self.image_count:06d}{ext}")

        with open(filename, "wb") as f:
            f.write(msg.data)

        self.get_logger().info(f"Saved {filename}")
        self.image_count += 1

    def stop_callback(self, request, response):
        self.saving = False
        response.success = True
        response.message = f"Stopped saving images. Total saved: {self.image_count}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
