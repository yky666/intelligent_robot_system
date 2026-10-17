#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

class ArLaserFollower
{
public:
    ArLaserFollower()
        : nh_(),
          private_nh_("~"),
          marker_seen_(false),
          marker_lost_(false),
          have_scan_(false),
          obstacle_avoid_active_(false),
          marker_range_(0.0),
          marker_bearing_(0.0),
          last_search_direction_(0.0),
          obstacle_direction_(0.0)
    {
        private_nh_.param("marker_id", marker_id_, 0);
        private_nh_.param<std::string>("marker_topic", marker_topic_, "/ar_pose_marker");
        private_nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                                       "/mobile_base/commands/velocity");

        private_nh_.param("desired_distance", desired_distance_, 0.40);
        private_nh_.param("distance_gain", distance_gain_, 0.8);
        private_nh_.param("bearing_gain", bearing_gain_, 1.8);
        private_nh_.param("max_linear_speed", max_linear_speed_, 0.35);
        private_nh_.param("max_angular_speed", max_angular_speed_, 0.25);
        private_nh_.param("marker_timeout", marker_timeout_, 0.8);
        private_nh_.param("lost_search_timeout", lost_search_timeout_, 16.0);
        private_nh_.param("search_angular_speed", search_angular_speed_, 0.20);
        private_nh_.param("scan_timeout", scan_timeout_, 0.8);
        // The current depthimage_to_laserscan reports range_min ~= 0.45 m.
        // Stop before entering that near-field blind zone.
        private_nh_.param("avoid_distance", avoid_distance_, 0.38);
        private_nh_.param("stop_distance", stop_distance_, 0.20);
        private_nh_.param("side_clearance", side_clearance_, 0.40);
        private_nh_.param("avoid_release_margin", avoid_release_margin_, 0.20);
        private_nh_.param("obstacle_turn_speed", obstacle_turn_speed_, 0.20);
        if (avoid_distance_ <= stop_distance_)
        {
            ROS_WARN("avoid_distance (%.2f) must exceed stop_distance (%.2f); "
                     "raising avoid_distance to %.2f m",
                     avoid_distance_, stop_distance_, stop_distance_ + 0.25);
            avoid_distance_ = stop_distance_ + 0.25;
        }
        // Kinect depth-to-scan field of view is about +/-29 degrees on this setup.
        // Keep the avoidance sectors inside that measured field of view.
        private_nh_.param("front_half_angle_deg", front_half_angle_deg_, 14.0);
        private_nh_.param("side_min_angle_deg", side_min_angle_deg_, 15.0);
        private_nh_.param("side_max_angle_deg", side_max_angle_deg_, 27.0);

        marker_sub_ = nh_.subscribe(marker_topic_, 1,
                                    &ArLaserFollower::markerCallback, this);
        scan_sub_ = nh_.subscribe(scan_topic_, 1,
                                  &ArLaserFollower::scanCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

        ROS_INFO("AR follower: marker_id=%d desired_distance=%.2f m",
                 marker_id_, desired_distance_);
        ROS_INFO("Subscribing to %s and %s; publishing %s",
                 marker_topic_.c_str(), scan_topic_.c_str(), cmd_vel_topic_.c_str());
    }

    void run()
    {
        ros::Rate rate(10.0);
        while (ros::ok())
        {
            ros::spinOnce();
            controlStep();
            rate.sleep();
        }
        publishStop();
    }

private:
    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }

    static double degreesToRadians(double degrees)
    {
        const double pi = 3.14159265358979323846;
        return degrees * pi / 180.0;
    }

    void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& message)
    {
        // 只接受指定 ID，避免把视野内其他机器人或标签当作跟随目标。
        for (std::size_t i = 0; i < message->markers.size(); ++i)
        {
            const ar_track_alvar_msgs::AlvarMarker& marker = message->markers[i];
            if (marker.id != marker_id_)
                continue;

            // optical frame: x 向图像右侧，z 向相机前方。
            const double x = marker.pose.pose.position.x;
            const double z = marker.pose.pose.position.z;
            const double distance = std::sqrt(x * x + z * z);
            if (!std::isfinite(distance) || distance <= 0.05)
                return;

            marker_range_ = distance;
            marker_bearing_ = std::atan2(x, z);
            // If the target was near an image edge before disappearing, search
            // first in that same turn direction after it is lost.
            if (std::fabs(marker_bearing_) > degreesToRadians(4.0))
                last_search_direction_ = marker_bearing_ > 0.0
                    ? -search_angular_speed_ : search_angular_speed_;
            marker_seen_at_ = ros::Time::now();
            marker_seen_ = true;
            marker_lost_ = false;
            return;
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message)
    {
        scan_ = *message;
        scan_seen_at_ = ros::Time::now();
        have_scan_ = true;
    }

    static bool validRange(double value, const sensor_msgs::LaserScan& scan)
    {
        return std::isfinite(value) && value >= scan.range_min && value <= scan.range_max;
    }

    double sectorMinimum(const sensor_msgs::LaserScan& scan,
                         double low_angle, double high_angle) const
    {
        double minimum = std::min(static_cast<double>(scan.range_max), 3.0);
        bool found = false;
        bool has_beam = false;
        for (std::size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const double angle = scan.angle_min + i * scan.angle_increment;
            if (angle >= low_angle && angle <= high_angle)
                has_beam = true;
            const double value = scan.ranges[i];
            if (angle >= low_angle && angle <= high_angle && validRange(value, scan))
            {
                minimum = found ? std::min(minimum, static_cast<double>(value))
                                : static_cast<double>(value);
                found = true;
            }
        }
        // If no scan ray covers this sector, its clearance is unknown, not free.
        // If rays exist but report no obstacle, use the sensor's usable range.
        if (!has_beam)
            return 0.0;
        return found ? minimum : std::min(static_cast<double>(scan.range_max), 3.0);
    }

    void laserClearances(double& front, double& left, double& right) const
    {
        const double front_half_angle = degreesToRadians(front_half_angle_deg_);
        const double side_min_angle = degreesToRadians(side_min_angle_deg_);
        const double side_max_angle = degreesToRadians(side_max_angle_deg_);
        front = sectorMinimum(scan_, -front_half_angle, front_half_angle);
        left = sectorMinimum(scan_, side_min_angle, side_max_angle);
        right = sectorMinimum(scan_, -side_max_angle, -side_min_angle);
    }

    void publishStop()
    {
        cmd_pub_.publish(geometry_msgs::Twist());
    }

    void controlStep()
    {
        const ros::Time now = ros::Time::now();
        if (!have_scan_ || (now - scan_seen_at_).toSec() > scan_timeout_)
        {
            ROS_WARN_THROTTLE(2.0, "No fresh /scan; stopping follower");
            publishStop();
            return;
        }

        const double marker_age = marker_seen_ ? (now - marker_seen_at_).toSec() : 0.0;
        if (!marker_seen_ || marker_age > marker_timeout_)
        {
            // 首次识别前不乱转；跟丢后仅原地慢转，尝试在原位置重新捕获。
            if (!marker_seen_)
            {
                publishStop();
                return;
            }
            if (!marker_lost_)
            {
                marker_lost_ = true;
                marker_lost_at_ = now;
            }
            if ((now - marker_lost_at_).toSec() > lost_search_timeout_)
            {
                ROS_WARN_THROTTLE(2.0,
                    "Target marker %d not reacquired; holding position", marker_id_);
                publishStop();
                return;
            }

            double front, left, right;
            laserClearances(front, left, right);
            geometry_msgs::Twist search;
            const double search_speed = std::min(search_angular_speed_, max_angular_speed_);
            if (last_search_direction_ > 0.0 && left >= side_clearance_)
                search.angular.z = search_speed;
            else if (last_search_direction_ < 0.0 && right >= side_clearance_)
                search.angular.z = -search_speed;
            else if (left >= side_clearance_ || right >= side_clearance_)
                search.angular.z = left > right ? search_speed : -search_speed;
            cmd_pub_.publish(search);
            ROS_WARN_THROTTLE(2.0,
                "Target marker %d lost; rotating in place to reacquire", marker_id_);
            return;
        }

        double front, left, right;
        laserClearances(front, left, right);

        // Latch the selected side while negotiating one obstacle. This avoids
        // left/right oscillation as the narrow Kinect scan changes frame to frame.
        if (!obstacle_avoid_active_ && front < avoid_distance_)
        {
            obstacle_avoid_active_ = true;
            if (left >= side_clearance_ || right >= side_clearance_)
                obstacle_direction_ = left > right ? 1.0 : -1.0;
            else
                obstacle_direction_ = 0.0;
            ROS_INFO("Obstacle avoidance started: front=%.2f left=%.2f right=%.2f direction=%.0f",
                     front, left, right, obstacle_direction_);
        }
        if (obstacle_avoid_active_ && front > avoid_distance_ + avoid_release_margin_)
        {
            obstacle_avoid_active_ = false;
            obstacle_direction_ = 0.0;
            ROS_INFO("Obstacle cleared; resuming AR following");
        }

        geometry_msgs::Twist command;
        const double distance_error = marker_range_ - desired_distance_;

        // 相机 x 正方向是图像右侧；目标在右侧时底盘应向右转，angular.z 为负。
        command.angular.z = clamp(-bearing_gain_ * marker_bearing_,
                                  -max_angular_speed_, max_angular_speed_);
        if (std::fabs(marker_bearing_) > degreesToRadians(55.0))
            command.linear.x = 0.0;
        else
            command.linear.x = clamp(distance_gain_ * distance_error,
                                     0.0, max_linear_speed_);

        // 前方近距离硬保护：不继续向障碍物前进。
        if (obstacle_avoid_active_ && obstacle_direction_ == 0.0)
        {
            // Neither visible side has enough measured clearance.
            command.linear.x = 0.0;
            command.angular.z = 0.0;
            ROS_WARN_THROTTLE(1.0,
                "Obstacle ahead but neither visible side has enough clearance; holding position");
        }
        else if (front <= stop_distance_)
        {
            command.linear.x = 0.0;
            const double selected_clearance = obstacle_direction_ > 0.0 ? left : right;
            if (selected_clearance < side_clearance_)
            {
                command.angular.z = 0.0;
                ROS_WARN_THROTTLE(1.0,
                    "Chosen bypass side is blocked (front %.2f m); holding position", front);
            }
            else
            {
                command.angular.z = obstacle_direction_ *
                    std::min(obstacle_turn_speed_, max_angular_speed_);
                ROS_WARN_THROTTLE(1.0,
                    "Obstacle close (%.2f m); turning toward open side", front);
            }
        }
        else if (obstacle_avoid_active_)
        {
            command.linear.x = std::min(command.linear.x, 0.04);
            const double selected_clearance = obstacle_direction_ > 0.0 ? left : right;
            if (selected_clearance < side_clearance_)
            {
                command.linear.x = 0.0;
                command.angular.z = 0.0;
                ROS_WARN_THROTTLE(1.0,
                    "Selected bypass side lost clearance; holding position");
            }
            else
            {
                const double bypass_bias = obstacle_direction_ * 0.20;
                command.angular.z = clamp(command.angular.z + bypass_bias,
                                          -max_angular_speed_, max_angular_speed_);
            }
        }

        cmd_pub_.publish(command);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber marker_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher cmd_pub_;

    int marker_id_;
    std::string marker_topic_;
    std::string scan_topic_;
    std::string cmd_vel_topic_;

    double desired_distance_;
    double distance_gain_;
    double bearing_gain_;
    double max_linear_speed_;
    double max_angular_speed_;
    double marker_timeout_;
    double lost_search_timeout_;
    double search_angular_speed_;
    double scan_timeout_;
    double avoid_distance_;
    double stop_distance_;
    double side_clearance_;
    double avoid_release_margin_;
    double obstacle_turn_speed_;
    double front_half_angle_deg_;
    double side_min_angle_deg_;
    double side_max_angle_deg_;

    bool marker_seen_;
    bool marker_lost_;
    bool have_scan_;
    bool obstacle_avoid_active_;
    double marker_range_;
    double marker_bearing_;
    double last_search_direction_;
    double obstacle_direction_;
    ros::Time marker_seen_at_;
    ros::Time marker_lost_at_;
    ros::Time scan_seen_at_;
    sensor_msgs::LaserScan scan_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_laser_follower");
    ArLaserFollower follower;
    follower.run();
    return 0;
}
