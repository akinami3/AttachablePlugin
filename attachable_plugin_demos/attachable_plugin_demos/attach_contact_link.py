from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Empty
import rclpy
from rclpy.node import Node
import subprocess
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class AttachContactLink(Node):
    def __init__(self):
        super().__init__('attach_contact_link')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.contact_sub = self.create_subscription(
            Contacts,
            '/contact',
            self.contact_callback,
            qos_profile
        )
        self.attach_model1_name = None
        self.attach_link1_name = None
        self.attach_model2_name = None
        self.attach_link2_name = None
        self.is_attached = False

        self.detach_sub = self.create_subscription(
            Empty,
            '/detach',
            self.detach_callback,
            qos_profile
        )
        self.get_logger().info("AttachContactLink initialized.")

        self.attach_num = 30
        self.attach_count = 0


    def contact_callback(self, msg: Contacts):
        if self.is_attached:
            # Do nothing if already attached
            return
        
        # Get the names of the models/links in contact
        for contact in msg.contacts:
            # Simultaneously retrieve the full names of collision1 and collision2
            full_name1 = contact.collision1.name  
            full_name2 = contact.collision2.name  
            
            # Split using "::" to extract model and link names
            parts1 = full_name1.split("::")
            parts2 = full_name2.split("::")
            
            if len(parts1) >= 2 and len(parts2) >= 2:
                attach_model1_name = parts1[0]
                attach_link1_name = parts1[1]
                attach_model2_name = parts2[0]
                attach_link2_name = parts2[1]
                break  

        # Ignore if the model is ground_plane
        if attach_model1_name == "ground_plane" or attach_model2_name == "ground_plane":
            return

        # Reset the count if a different model/link pair comes into contact
        if attach_model1_name != self.attach_model1_name or \
           attach_link1_name != self.attach_link1_name or \
           attach_model2_name != self.attach_model2_name or \
           attach_link2_name != self.attach_link2_name:
            # Update the previous model/link
            self.attach_model1_name = attach_model1_name
            self.attach_link1_name = attach_link1_name
            self.attach_model2_name = attach_model2_name
            self.attach_link2_name = attach_link2_name

            # Reset the count
            self.attach_count = 1
            self.get_logger().info(f"Contact count: {self.attach_count}")
            return
        
        # Attach if the same model/link pair comes into contact attach_num times
        if self.attach_count < self.attach_num:
            # Update the previous model/link
            self.attach_model1_name = attach_model1_name
            self.attach_link1_name = attach_link1_name
            self.attach_model2_name = attach_model2_name
            self.attach_link2_name = attach_link2_name

            # Increment the count
            self.attach_count += 1
            self.get_logger().info(f"Contact count: {self.attach_count}")
            return

        cmd = "gz topic -t /attach -m gz.msgs.StringMsg " \
              f"-p 'data:\"[{self.attach_model1_name}][{self.attach_link1_name}][{self.attach_model2_name}][{self.attach_link2_name}][attach]\"'"
        subprocess.run(cmd, shell=True)
        self.get_logger().info(f"Command executed: {cmd}")
        self.is_attached = True
        self.attach_count = 0


    def detach_callback(self, msg: Empty):
        if not self.is_attached:
            # Do nothing if already detached
            return

        cmd = "gz topic -t /attach -m gz.msgs.StringMsg " \
              f"-p 'data:\"[{self.attach_model1_name}][{self.attach_link1_name}][{self.attach_model2_name}][{self.attach_link2_name}][detach]\"'"
        subprocess.run(cmd, shell=True)
        self.get_logger().info(f"Command executed: {cmd}")

        self.attach_model1_name = None
        self.attach_link1_name = None
        self.attach_model2_name = None
        self.attach_link2_name = None

        self.is_attached = False

def main(args=None):
    rclpy.init(args=args)
    node = AttachContactLink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
