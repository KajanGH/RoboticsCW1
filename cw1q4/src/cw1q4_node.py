#!/usr/bin/env python3

import rospy
import numpy as np
import math

# TODO: Include all the required service classes
# your code starts here -----------------------------
from cw1q4_srv.srv import quat2zyx, quat2zyxRequest, quat2zyxResponse
from cw1q4_srv.srv import quat2rodrigues, quat2rodriguesRequest, quat2rodriguesResponse
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
# your code ends here -------------------------------




def convert_quat2zyx(request):
    # TODO complete the function
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): cw1q4_srv service message, containing
        the quaternion you need to convert.

    Returns:
        quat2zyxResponse: cw1q4_srv service response, in which 
        you store the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Your code starts here ----------------------------
    """Converts quaternion to Euler ZYX representation."""
    q = request.q  # geometry_msgs/Quaternion
    
    # Extract quaternion components
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # Calculate Euler angles from quaternion
    # Formula: ZYX convention (yaw-pitch-roll)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll_x = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch_y = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)

    response = quat2zyxResponse(x=Float64(roll_x), y=Float64(pitch_y), z=Float64(yaw_z))

    # Your code ends here ------------------------------

    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):
    # TODO complete the function

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): cw1q4_srv service message, containing
        the quaternion you need to convert

    Returns:
        quat2rodriguesResponse: cw1q4_srv service response, in which 
        you store the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)

    # Your code starts here ----------------------------
    """Converts quaternion to Rodrigues vector representation."""
    q = request.q  # geometry_msgs/Quaternion
    
    # Extract quaternion components
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # Compute the angle and the axis
    angle = 2 * math.acos(qw)
    s = math.sqrt(1 - qw * qw)

    if s < 1e-6:  # Avoid division by zero; quaternion is nearly 1
        rodrigues_x, rodrigues_y, rodrigues_z = qx, qy, qz
    else:
        rodrigues_x = qx / s * angle
        rodrigues_y = qy / s * angle
        rodrigues_z = qz / s * angle

    response = quat2rodriguesResponse(
        x=Float64(rodrigues_x), y=Float64(rodrigues_y), z=Float64(rodrigues_z)
    )


    # Your code ends here ------------------------------

    assert isinstance(response, quat2rodriguesResponse)
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    #Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
