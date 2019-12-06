#include <ros/ros.h>

#include <ros/publisher.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <nav_msgs/Odometry.h>

const char node_name[] = "pose_tf_broadcaster";

class PoseBroadcasterNode
{
    public:
        PoseBroadcasterNode(ros::NodeHandle& node) :
            listener_(tf_buffer_)
        {
            std::string transform_topic = "fiducial_transforms";
            node.getParam("transform_topic", transform_topic);
            node.getParam("camera_link_name", camera_link_name_);

            subscriber_ = node.subscribe(transform_topic, 1, &PoseBroadcasterNode::OnFiducialTransformsUpdates, this);
            publisher_ = node.advertise<nav_msgs::Odometry>("/odom", 5);
        }

        double angle_trust = 10; // In degrees, how much we trust rotation estimates as a baseline
        double velocity_trust = 100; // In meters/second how accurate the velocity estimate is - since this node estimates pose, it should be fairly high

    protected:
        void OnFiducialTransformsUpdates(const fiducial_msgs::FiducialTransformArrayConstPtr& message)
        {
            bool is_current_pose_updated = false;
            double current_pose_error = 0.0;
            double current_pose_time = 0.0;
            tf2::Transform current_pose;

            for (auto transform : message->transforms)
            {
                ros::Time transform_time = ros::Time::now();
                int marker_id = transform.fiducial_id;
                std::string marker_name = "aruco_" + std::to_string(marker_id);

                tf2::Transform marker_to_camera;
                tf2::fromMsg(transform.transform, marker_to_camera);

                // The marker transform is in the wrong coordinate system, rotate it to fix it
                tf2::Quaternion coordinate_correction;
                coordinate_correction.setRPY(M_PI_2, 0, -M_PI_2);

                // Transform is initially camera to aruco, we want aruco to camera
                marker_to_camera = marker_to_camera.inverse();
                marker_to_camera.setRotation(coordinate_correction * marker_to_camera.getRotation());
                
                try
                {
                    // Pull some transforms from the stack to get our latest transform
                    tf2::Transform world_to_marker;
                    tf2::Transform camera_to_base;
                    geometry_msgs::TransformStamped stamped_transform;

                    stamped_transform = tf_buffer_.lookupTransform(marker_name, "world", ros::Time(0));
                    tf2::fromMsg(stamped_transform.transform, world_to_marker);

                    stamped_transform = tf_buffer_.lookupTransform(base_link_name_, camera_link_name_, ros::Time(0));
                    tf2::fromMsg(stamped_transform.transform, camera_to_base);
                    camera_to_base = camera_to_base.inverse();

                    // Multiply transforms to get full pose estimate (note: double check the multiplicaton order)
                    world_to_marker = camera_to_base * marker_to_camera * world_to_marker;

                    nav_msgs::Odometry odom_message;
                    odom_message.header.frame_id = "odom";
                    odom_message.child_frame_id = base_link_name_;
                    odom_message.header.stamp = ros::Time::now();

                    // Set pose position as transform origin
                    tf2::Vector3 origin = world_to_marker.getOrigin();
                    odom_message.pose.pose.position.x = origin.getX();
                    odom_message.pose.pose.position.y = origin.getY();
                    odom_message.pose.pose.position.z = origin.getZ();

                    // Set pose orientation as transform rotation
                    odom_message.pose.pose.orientation = tf2::toMsg(world_to_marker.getRotation());
                    
                    double q = transform.object_error;
                    double r = angle_trust * M_PI / 180.0;

                    odom_message.pose.covariance = boost::array<double, 36>({
                        q, 0, 0, 0, 0, 0,
                        0, q, 0, 0, 0, 0,
                        0, 0, q, 0, 0, 0,
                        0, 0, 0, r, 0, 0,
                        0, 0, 0, 0, r, 0,
                        0, 0, 0, 0, 0, r
                    });

                    // Try to estimate velocity from pose
                    double current_time = transform_time.toSec();
                    double delta_time = current_time - last_pose_time_;

                    // Create matrix representing just rotation of current pose
                    tf2::Transform rotation_matrix;
                    rotation_matrix.setIdentity();
                    rotation_matrix.setRotation(world_to_marker.getRotation());

                    // Create a matrix representing just the world space velocity of the robot
                    tf2::Vector3 world_space_velocity = (origin - last_pose_.getOrigin()) / delta_time;
                    tf2::Transform velocity_matrix;
                    velocity_matrix.setIdentity();
                    velocity_matrix.setOrigin(world_space_velocity);

                    // Rotate the velocity by the current pose's rotation
                    tf2::Vector3 local_space_velocity = (velocity_matrix * rotation_matrix).getOrigin();

                    // Find the quaternion representing the rotation between the two poses
                    tf2::Quaternion delta_rotation = world_to_marker.getRotation() * last_pose_.getRotation().inverse();

                    // Convert the delta rotation to RPY
                    tf2::Matrix3x3 delta_rotation_matrix(delta_rotation);
                    double delta_roll, delta_pitch, delta_yaw;
                    delta_rotation_matrix.getRPY(delta_roll, delta_pitch, delta_yaw);
                    tf2::Vector3 angular_velocity(delta_roll, delta_pitch, delta_yaw);
                    angular_velocity /= delta_time;

                    // Populate odom message with very noisy velocity data
                    odom_message.twist.twist.linear = tf2::toMsg(local_space_velocity);
                    odom_message.twist.twist.angular = tf2::toMsg(angular_velocity);

                    double v = velocity_trust;

                    odom_message.twist.covariance = boost::array<double, 36>({
                        v, 0, 0, 0, 0, 0,
                        0, v, 0, 0, 0, 0,
                        0, 0, v, 0, 0, 0,
                        0, 0, 0, v, 0, 0,
                        0, 0, 0, 0, v, 0,
                        0, 0, 0, 0, 0, v
                    });

                    // A bit of a hacky method to find our most recent, best estimated pose and save it for next time
                    if (!is_current_pose_updated || transform.object_error < current_pose_error)
                    {
                        current_pose = world_to_marker;
                        current_pose_error = transform.object_error;
                        current_pose_time = current_time;
                        is_current_pose_updated = true;
                    }

                    publisher_.publish(odom_message);
                }
                catch(tf2::LookupException& e)
                {
                    ROS_WARN_NAMED(node_name, "Unable to get required transforms: %s", e.what());
                }
            }

            last_pose_ = current_pose;
            last_pose_time_ = current_pose_time;
        }

    private:
        std::string camera_link_name_ = "camera_link";
        std::string base_link_name_ = "base_link";

        ros::Subscriber subscriber_;
        tf2_ros::TransformBroadcaster broadcaster_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener listener_;

        ros::Publisher publisher_;

        tf2::Transform last_pose_;
        double last_pose_time_ = 0.0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node("~");

    PoseBroadcasterNode broadcaster_node(node);

    ros::spin();

    return 0;
}
