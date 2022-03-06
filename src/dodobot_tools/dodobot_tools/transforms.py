import rospy
import tf2_ros


def lookup_transform(tf_buffer, parent_link, child_link, time_window=None, timeout=None, silent=False):
    """
    Call tf_buffer.lookup_transform. Return None if the look up fails
    """
    if time_window is None:
        time_window = rospy.Time(0)
    else:
        time_window = rospy.Time.now() - time_window

    if timeout is None:
        timeout = rospy.Duration(1.0)

    try:
        return tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        if not silent:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_link, child_link, e))
        return None
