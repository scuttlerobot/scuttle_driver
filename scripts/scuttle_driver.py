#!/usr/bin/python3
import tf
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from scuttlepy import SCUTTLE

def command_velocity(msg):

    # rospy.loginfo("Linear Velocity: "+str(round(msg.linear.x, 3))+" \tAngular Velocity: "+str(round(msg.angular.z, 3)))
    scuttle.setMotion([msg.linear.x, msg.angular.z])

def update_pose(msg):

    pose = msg.pose.pose
    orientation = pose.orientation
    position = pose.position

    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    scuttle.set_heading(yaw)
    scuttle.set_global_position([position.x, position.y])

if __name__=="__main__":

    scuttle = SCUTTLE()

    rospy.init_node("scuttle_driver")
    r = rospy.Rate(50) # 50hz

    # Get enable_tf_publish enable_joint_state_publish flags
    enable_tf_publish          = rospy.get_param("/enable_tf_publish", True)
    enable_joint_state_publish = rospy.get_param("/enable_joint_state_publish", True)

    rospy.Subscriber("/cmd_vel", Twist, command_velocity)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, update_pose)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    if enable_tf_publish is True:
        odom_broadcaster = tf.TransformBroadcaster()

    try:

        while not rospy.is_shutdown():

            current_time = rospy.Time.now()

            x, y = scuttle.get_global_position()
            velocity_x, velocity_theta = scuttle.get_motion()
            th = scuttle.get_heading()

            odometry_quaternion = tf.transformations.quaternion_from_euler(0, 0, th)

            # Publish TF when enable_tf_pulish is true
            if enable_tf_publish is True:
                odom_broadcaster.sendTransform(
                (x, y, 0.),
                odometry_quaternion,
                current_time,
                "base_link",
                "odom"
                )

            odometry = Odometry()
            odometry.header.stamp = current_time
            odometry.header.frame_id = "odom"

            odometry.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odometry_quaternion))
            # Since some ROS nodes expect non-zero pose covariance,
            # set non-zero default pose covariance matrix.
            odometry.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.1, 0, 0,
                                    0, 0, 0, 0, 0.1, 0,
                                    0, 0, 0, 0, 0, 0.1]

            odometry.child_frame_id = "base_link"
            odometry.twist.twist = Twist(Vector3(0, velocity_x, 0), Vector3(0, 0, 0))

            odom_pub.publish(odometry)

            # Publish joint state when enable_joint_state_publish is true
            if enable_joint_state_publish is True:
                joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
                # pub = rospy.Publisher('joint_states', JointState, queue_size=10)
                # rospy.init_node('joint_state_publisher')
                joint_state = JointState()

                joint_state.header = Header()

                joint_state.header.stamp = rospy.Time.now()

                joint_state.name = ['l_wheel_joint',
                                   'r_wheel_joint',
                                   'r_caster_swivel_joint',
                                   'l_caster_swivel_joint',
                                   'r_caster_wheel_joint',
                                   'l_caster_wheel_joint'
                                  ]

                joint_state.position = [scuttle.left_wheel.encoder.position * ((2 * np.pi) / scuttle.left_wheel.encoder.resolution),
                                        scuttle.right_wheel.encoder.position * ((2 * np.pi) / scuttle.right_wheel.encoder.resolution),
                                       0,
                                       0,
                                       0,
                                       0
                                      ]

                joint_state.velocity = []
                joint_state.effort = []
                joint_state_publisher.publish(joint_state)

            r.sleep()

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        print('Stopping...')
        # pass

    finally:
        scuttle.stop()
