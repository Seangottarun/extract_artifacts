import rospy
import tf
from geometry_msgs.msg import PointStamped, Point


def transform_point(point, from_frame, to_frame):
    # Initialize the tf listener
    listener = tf.TransformListener()

    # Wait for the transform to become available
    listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(4.0))

    # Create a PointStamped object with the given point
    point_stamped = PointStamped()
    point_stamped.header.frame_id = from_frame
    point_stamped.header.stamp = rospy.Time(0)
    point_stamped.point = point

    try:
        # Transform the point to the target frame
        point_transformed = listener.transformPoint(to_frame, point_stamped)
        return point_transformed.point
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Transformation error: {e}")
        return None
    
"""
EXAMPLE USAGE:

point = Point(1.0, 2.0, 3.0)
from_frame = "rgb_camera_optical_link"
to_frame = "world_graph_msf"
point_transformed = transform_point(point, from_frame, to_frame)

"""