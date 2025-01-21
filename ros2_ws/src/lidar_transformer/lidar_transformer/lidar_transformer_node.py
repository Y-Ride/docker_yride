import rclpy
import rclpy.exceptions
import rclpy.logging
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import TransformStamped


class LidarTransformerNode(Node):
    def __init__(self):
        super().__init__('lidar_transformer_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        # Declare and load parameters
        self.target_frame_id = self.declare_parameter('target_frame_id', 'os_lidar').value
        self.lidar_topics = self.declare_parameter('lidar_topics', ['/ouster/points']).value
        self.lidar_frame_ids = self.declare_parameter('lidar_frame_ids', ['os_lidar']).value

        self.get_logger().debug(f"target frame id: {self.target_frame_id}")
        self.get_logger().debug(f"lidar topics to subscribe to: {self.lidar_topics}")
        self.get_logger().debug(f"lidar frame ids: {self.lidar_frame_ids}")

        # Load transformation matrices for lidar_frame_ids
        self.lidar_tf_dict = {}
        for lidar_frame_id in self.lidar_frame_ids:
            self.get_logger().debug(f"Getting transformation for {lidar_frame_id}")
            self.declare_parameter(lidar_frame_id, Parameter.Type.DOUBLE_ARRAY)
            try:
                # Retrieve transformation matrix for each lidar frame
                tf_list = np.array(self.get_parameter(lidar_frame_id).value)
                if tf_list.size != 16:
                    raise ValueError(f"Transformation matrix for {lidar_frame_id} is not 1x16 (4x4 flattened)")
                self.lidar_tf_dict[lidar_frame_id] = tf_list.reshape(4,4)
            except rclpy.exceptions.ParameterNotDeclaredException:
                self.get_logger().error(f"Parameter '{lidar_frame_id}' is not declared in the parameter file!")
                raise ValueError(f"Parameter '{lidar_frame_id}' is not set!")

        # Check if transformations were loaded
        if not self.lidar_tf_dict:
            self.get_logger().error('No transformations found')
            rclpy.try_shutdown()
            return

        # Subscriber and publisher
        # Create a dictionary to map subscriptions to their topics
        self.subscribers = {}
        for topic in self.lidar_topics:
            sub = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic=topic: self.lidar_callback(msg,topic),  # Lambda to pass the topic
                10
            )
            self.subscribers[sub] = topic  # Map subscription object to topic name

        # self.lidar_subscriptions = []
        # for topic in self.lidar_topics:
        #     self.get_logger().debug(f"Subscribing to topic: {topic}")
        #     subscription = self.create_subscription(
        #         PointCloud2,
        #         topic,
        #         self.lidar_callback,  # Same callback for all topics
        #         10
        #     )
        #     self.lidar_subscriptions.append(subscription)

        self.lidar_publishers = {}
        for lidar_topic in self.lidar_topics:
            publisher = self.create_publisher(
                PointCloud2,
                '/transformed_lidar'+lidar_topic,
                10
            )
            self.lidar_publishers[lidar_topic]=publisher

    def lidar_callback(self, msg: PointCloud2, topic=None):
        # # Print debug to make sure the callback is working
        # self.get_logger().debug('Received point cloud')

        # Determine which subscription triggered the callback
        # triggered_topic = self.subscribers[subscription]
        self.get_logger().debug(f"Received message from topic: {topic}")

        # Get and print what the frame_id is
        frame_id = msg.header.frame_id
        self.get_logger().debug(f'Frame ID: {frame_id}')

        # Check length of the data in msg
        self.get_logger().debug(f"Length of data: {len(msg.data)}")

        # Check if the frame_id is in the transformation dictionary
        if frame_id not in self.lidar_tf_dict.keys():
            self.get_logger().warn(f"No transformation matrix given for frame: {frame_id}")
            return
        
        try:
            # Get transformation matrix for the frame
            transformation = np.array(self.lidar_tf_dict[frame_id])
            self.get_logger().debug(f"Transformation matrix is:\n{transformation}")
            if transformation.shape != (4,4):
                raise ValueError("Transformation is not 4x4")
            
            # Transform the point cloud data
            transformed_cloud = self.transform_point_cloud(msg, transformation)
            transformed_cloud.header.frame_id = self.target_frame_id  # Set the target frame

        except Exception as e:
            self.get_logger().error(f"Error transforming point cloud: {e}")

        # Log the updated frame_id for confirmation
        self.get_logger().debug(f"Publishing with new frame_id: {transformed_cloud.header.frame_id}")

        # Publish to the corresponding transformed topic
        self.lidar_publishers[topic].publish(transformed_cloud)
    
    # # TODO: This function should keep the intensity values for the point cloud, but it hasn't been tested and might be slower
    # def transform_point_cloud(self, msg: PointCloud2, transformation_matrix) -> PointCloud2:
    #     # Read all points and retain all fields
    #     cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names=None))
    #     fields = msg.fields  # Get field definitions
    #     field_indices = {field.name: i for i, field in enumerate(fields)} # Map field names to indices

    #     if not all(f in field_indices for f in ['x', 'y', 'z']):
    #         raise ValueError("PointCloud2 message does not have x, y, z fields.")

    #     transformed_points = []
    #     for point in cloud_points:
    #         # Transform the x, y, z fields only
    #         original_point = list(point)
    #         transformed_xyz = self.apply_transform(
    #             [point[field_indices['x']], point[field_indices['y']], point[field_indices['z']]],
    #             transformation_matrix
    #         )
    #         # Update x, y, z in the original point
    #         original_point[field_indices['x']] = transformed_xyz[0]
    #         original_point[field_indices['y']] = transformed_xyz[1]
    #         original_point[field_indices['z']] = transformed_xyz[2]
    #         transformed_points.append(original_point)

    #     # Create a new PointCloud2 message with the transformed points and original fields
    #     transformed_cloud = point_cloud2.create_cloud(msg.header, fields, transformed_points)
    #     return transformed_cloud


    def transform_point_cloud(self, msg: PointCloud2, transformation_matrix) -> PointCloud2:
        # Convert PointCloud2 to numpy array for transformation
        cloud_points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_array = np.array(cloud_points)

        # Apply the 4x4 transformation matrix to the point cloud
        transformed_points = []
        for point in points_array:
            # Apply the transformation (rotation + translation)
            transformed_point = self.apply_transform(point, transformation_matrix)
            transformed_points.append(transformed_point)
        
        self.get_logger().debug(f"Length of original {len(points_array)} points")
        self.get_logger().debug(f"Length of transformed {len(transformed_points)} points")


        # Convert the transformed points back to a PointCloud2 message
        transformed_cloud = point_cloud2.create_cloud_xyz32(msg.header, transformed_points)
        return transformed_cloud

    def apply_transform(self, point, transformation_matrix):
        # Convert the point to a homogeneous coordinate (x, y, z, 1)
        point_homogeneous = np.array([point[0], point[1], point[2], 1])

        # Apply the 4x4 transformation matrix
        transformed_point = np.dot(transformation_matrix, point_homogeneous)

        # Return the transformed (x, y, z) coordinates
        return [transformed_point[0], transformed_point[1], transformed_point[2]]


def main(args=None):
    rclpy.init(args=args)
    node = LidarTransformerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

