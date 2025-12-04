/**
 * @file ar_marker_laser_correction_node.cpp
 * @brief Node to correct AR marker orientation using laser scan data
 * 
 * This node subscribes to AR marker poses and laser scan data,
 * extracts lines from the laser scan, and corrects the marker's
 * orientation based on nearby wall/surface orientations.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <laser_line_extraction/LineSegmentList.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <algorithm>

class ARMarkerLaserCorrection
{
public:
    ARMarkerLaserCorrection() : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // Parameters
        private_nh.param<std::string>("base_frame", base_frame_, "base_link");
        private_nh.param<double>("search_radius", search_radius_, 0.2);
        private_nh.param<double>("angle_tolerance", angle_tolerance_, 45.0);
        private_nh.param<double>("min_line_length", min_line_length_, 0.05);
        private_nh.param<bool>("publish_visualization", publish_visualization_, true);
        
        // Convert angle tolerance to radians
        angle_tolerance_ = angle_tolerance_ * M_PI / 180.0;
        
        // Subscribers
        ar_marker_sub_ = nh.subscribe("ar_pose_marker", 10, 
            &ARMarkerLaserCorrection::arMarkerCallback, this);
        line_segments_sub_ = nh.subscribe("line_segments", 10,
            &ARMarkerLaserCorrection::lineSegmentsCallback, this);
        
        // Publishers
        ar_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "ar_marker_pose", 10);
        corrected_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "corrected_marker_pose", 10);
        corrected_markers_pub_ = nh.advertise<ar_track_alvar_msgs::AlvarMarkers>(
            "corrected_ar_markers", 10);
        
        if (publish_visualization_)
        {
            vis_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
                "correction_visualization", 10);
        }
        
        ROS_INFO("AR Marker Laser Correction Node initialized");
        ROS_INFO("Base frame: %s", base_frame_.c_str());
        ROS_INFO("Search radius: %.2f m, Angle tolerance: %.2f deg", 
                 search_radius_, angle_tolerance_ * 180.0 / M_PI);
    }
    
private:
    // ROS communication
    ros::Subscriber ar_marker_sub_;
    ros::Subscriber line_segments_sub_;
    ros::Publisher ar_pose_pub_;
    ros::Publisher corrected_pose_pub_;
    ros::Publisher corrected_markers_pub_;
    ros::Publisher vis_marker_pub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    std::string base_frame_;
    double search_radius_;
    double angle_tolerance_;
    double min_line_length_;
    bool publish_visualization_;
    
    // Data storage
    laser_line_extraction::LineSegmentList latest_lines_;
    bool lines_received_;
    
    struct LineSegment
    {
        double x1, y1, x2, y2;
        double angle;
        double length;
    };
    
    /**
     * @brief Callback for line segments from laser_line_extraction
     */
    void lineSegmentsCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg)
    {
        latest_lines_ = *msg;
        lines_received_ = true;
    }
    
    /**
     * @brief Callback for AR markers
     */
    void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
    {
        ar_track_alvar_msgs::AlvarMarkers corrected_markers;
        corrected_markers.header = msg->header;

        if (!lines_received_)
        {
            ROS_WARN_THROTTLE(5.0, "No line segments received yet");
            corrected_markers_pub_.publish(corrected_markers);
            return;
        }
        
        if (msg->markers.empty())
        {
            corrected_markers_pub_.publish(corrected_markers);
            return;
        }
        
        visualization_msgs::MarkerArray vis_array;
        
        for (const auto& marker : msg->markers)
        {
            // Skip if marker header frame is already base_frame
            geometry_msgs::PoseStamped marker_pose_out;
            if (marker.header.frame_id == base_frame_)
            {
                marker_pose_out.header = marker.header;
                marker_pose_out.pose = marker.pose.pose;
            }
            else
            {
                // Transform marker to base frame
                geometry_msgs::PoseStamped marker_pose_in;
                marker_pose_in.header = marker.header;
                marker_pose_in.pose = marker.pose.pose;
                
                try
                {
                    tf_buffer_.transform(marker_pose_in, marker_pose_out, base_frame_, 
                                        ros::Duration(0.1));
                }
                catch (tf2::TransformException& ex)
                {
                    ROS_WARN("Failed to transform marker: %s", ex.what());
                    continue;
                }
            }
            
            // Find nearby lines and correct orientation
            double corrected_yaw = correctOrientation(marker_pose_out, vis_array, marker.id);
            
            // Create corrected marker
            ar_track_alvar_msgs::AlvarMarker corrected_marker = marker;
            corrected_marker.header.stamp = marker_pose_out.header.stamp;
            
            // Update pose with corrected orientation
            tf::Quaternion q;
            tf::quaternionMsgToTF(marker_pose_out.pose.orientation, q);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            // Apply correction (only yaw, preserve roll and pitch)
            q.setRPY(0, corrected_yaw-M_PI/2, -M_PI);
            
            tf::quaternionTFToMsg(q, corrected_marker.pose.pose.orientation);
            
            corrected_markers.markers.push_back(corrected_marker);

            geometry_msgs::PoseStamped ar_pose;
            ar_pose.header = marker.header;
            ar_pose.pose = marker.pose.pose;
            ar_pose_pub_.publish(ar_pose);
            
            // Publish individual corrected pose
            geometry_msgs::PoseStamped corrected_pose;
            corrected_pose.header = marker.header;
            corrected_pose.pose = corrected_marker.pose.pose;
            corrected_pose_pub_.publish(corrected_pose);
        }
        
        // Publish corrected markers
        corrected_markers_pub_.publish(corrected_markers);
        
        // Publish visualization
        if (publish_visualization_ && !vis_array.markers.empty())
        {
            vis_marker_pub_.publish(vis_array);
        }
    }
    
    /**
     * @brief Correct marker orientation based on nearby laser lines
     */
    double correctOrientation(const geometry_msgs::PoseStamped& marker_pose,
                             visualization_msgs::MarkerArray& vis_array,
                             int marker_id)
    {
        // Extract current yaw 
        tf::Quaternion q;
        tf::quaternionMsgToTF(marker_pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        ROS_INFO("\n=== Correcting Marker %d ===", marker_id);
        ROS_INFO("Marker position in %s: X=%.3f, Y=%.3f, Z=%.3f",
                 base_frame_.c_str(),
                 marker_pose.pose.position.x,
                 marker_pose.pose.position.y,
                 marker_pose.pose.position.z);
        ROS_INFO("Marker yaw: %.2f deg", yaw * 180.0 / M_PI);
        
        // Transform line segments to base frame and find nearby lines
        std::vector<LineSegment> nearby_lines;
        
        ROS_DEBUG("Processing %zu line segments from frame '%s' to '%s'",
                 latest_lines_.line_segments.size(),
                 latest_lines_.header.frame_id.c_str(),
                 base_frame_.c_str());
        
        for (size_t i = 0; i < latest_lines_.line_segments.size(); ++i)
        {
            const auto& line = latest_lines_.line_segments[i];
            
            // Transform line endpoints from laser frame to base frame
            geometry_msgs::PointStamped p1_in, p2_in, p1_out, p2_out;
            p1_in.header = latest_lines_.header;
            p1_in.header.stamp = ros::Time(0); // Use latest available transform
            p1_in.point.x = line.start[0];
            p1_in.point.y = line.start[1];
            p1_in.point.z = 0;
            
            p2_in.header = latest_lines_.header;
            p2_in.header.stamp = ros::Time(0); // Use latest available transform
            p2_in.point.x = line.end[0];
            p2_in.point.y = line.end[1];
            p2_in.point.z = 0;
            
            try
            {
                tf_buffer_.transform(p1_in, p1_out, base_frame_, ros::Duration(0.1));
                tf_buffer_.transform(p2_in, p2_out, base_frame_, ros::Duration(0.1));
                
                ROS_INFO("Transform Line %zu: (%.3f, %.3f)->(%.3f, %.3f) [%s] => (%.3f, %.3f)->(%.3f, %.3f) [%s]",
                         i,
                         line.start[0], line.start[1], line.end[0], line.end[1],
                         latest_lines_.header.frame_id.c_str(),
                         p1_out.point.x, p1_out.point.y, p2_out.point.x, p2_out.point.y,  // Show X,Y coordinates
                         base_frame_.c_str());
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN_THROTTLE(2.0, "Failed to transform line %zu: %s", i, ex.what());
                continue;
            }
            
            LineSegment seg;
            // Use X and Z coordinates (assuming base_frame Y-axis points up/down)
            // This makes the horizontal plane be XY instead of XY
            seg.x1 = p1_out.point.x;
            seg.y1 = p1_out.point.y;  // Use Y coordinate for horizontal plane
            seg.x2 = p2_out.point.x;
            seg.y2 = p2_out.point.y;  // Use Y coordinate for horizontal plane
            seg.length = std::sqrt(std::pow(seg.x2 - seg.x1, 2) + 
                                   std::pow(seg.y2 - seg.y1, 2));
            seg.angle = std::atan2(seg.y2 - seg.y1, seg.x2 - seg.x1);
            
            // Check if line is near marker (use X and Z coordinates)
            double dist_to_marker = distanceToLine(marker_pose.pose.position.x,
                                                marker_pose.pose.position.y,
                                                seg);
            
            bool length_ok = seg.length > min_line_length_;
            bool dist_ok = dist_to_marker < search_radius_;
            
            ROS_INFO("  Line %zu: length=%.3fm [%s], dist=%.3fm [%s], angle=%.1f deg",
                     i, seg.length, length_ok ? "OK" : "TOO SHORT",
                     dist_to_marker, dist_ok ? "OK" : "TOO FAR",
                     seg.angle * 180.0 / M_PI);
            
            if (dist_ok && length_ok)
            {
                nearby_lines.push_back(seg);
                ROS_INFO("  -> ACCEPTED!");
            }
        }
        
        ROS_INFO("\nSummary: Found %zu nearby lines (out of %zu total) within %.2fm radius",
                 nearby_lines.size(), latest_lines_.line_segments.size(), search_radius_);
        
        if (nearby_lines.empty())
        {
            ROS_WARN("*** NO NEARBY LINES FOUND ***");
            ROS_WARN("Suggestions:");
            ROS_WARN("  1. Increase search_radius (current: %.2f m)", search_radius_);
            ROS_WARN("  2. Decrease min_line_length (current: %.2f m)", min_line_length_);
            ROS_WARN("  3. Check if marker and laser are in correct coordinate frames\n");
            return yaw; // No correction
        }
        
        // Find best matching line (closest angle to marker orientation or perpendicular)
        double best_angle = yaw;
        double min_angle_diff = M_PI;
        bool found_match = false;
        
        for (const auto& line : nearby_lines)
        {
            // Check alignment with marker (parallel) - considering 180 degree symmetry
            double angle_diff1 = std::abs(normalizeAngle(line.angle - yaw));
            double angle_diff1_180 = std::abs(normalizeAngle(line.angle - yaw + M_PI));
            angle_diff1 = std::min(angle_diff1, angle_diff1_180); // Lines have no direction
            
            // Check perpendicular alignment
            double perp_angle = normalizeAngle(line.angle + M_PI / 2.0);
            double angle_diff2 = std::abs(normalizeAngle(perp_angle - yaw));
            double angle_diff2_180 = std::abs(normalizeAngle(perp_angle - yaw + M_PI));
            angle_diff2 = std::min(angle_diff2, angle_diff2_180);
            
            double min_diff = std::min(angle_diff1, angle_diff2);
            
            ROS_DEBUG("  Line angle: %.2f deg, parallel diff: %.2f deg, perp diff: %.2f deg",
                     line.angle * 180.0 / M_PI,
                     angle_diff1 * 180.0 / M_PI,
                     angle_diff2 * 180.0 / M_PI);
            
            if (min_diff < min_angle_diff && min_diff < angle_tolerance_)
            {
                min_angle_diff = min_diff;
                found_match = true;
                // Choose the closer orientation
                if (angle_diff1 < angle_diff2)
                {
                    // Parallel: use line angle closest to marker yaw
                    double opt1 = line.angle;
                    double opt2 = normalizeAngle(line.angle + M_PI);
                    best_angle = (std::abs(normalizeAngle(opt1 - yaw)) < std::abs(normalizeAngle(opt2 - yaw))) ? opt1 : opt2;
                }
                else
                {
                    // Perpendicular: use perpendicular angle closest to marker yaw
                    double opt1 = perp_angle;
                    double opt2 = normalizeAngle(perp_angle + M_PI);
                    best_angle = (std::abs(normalizeAngle(opt1 - yaw)) < std::abs(normalizeAngle(opt2 - yaw))) ? opt1 : opt2;
                }
                
                ROS_DEBUG("  -> Best match so far! Using %s alignment, best_angle: %.2f deg",
                         (angle_diff1 < angle_diff2) ? "parallel" : "perpendicular",
                         best_angle * 180.0 / M_PI);
            }
        }
        
        if (!found_match)
        {
            ROS_WARN("No matching line found within angle tolerance (%.1f deg)", angle_tolerance_ * 180.0 / M_PI);
        }
        
        // Visualize correction
        if (publish_visualization_)
        {
            addVisualization(vis_array, marker_pose, nearby_lines, best_angle, marker_id);
        }
        
        ROS_INFO("Marker %d: Original yaw: %.2f deg, Corrected yaw: %.2f deg (correction: %.2f deg)",
                 marker_id, yaw * 180.0 / M_PI, best_angle * 180.0 / M_PI,
                 normalizeAngle(best_angle - yaw) * 180.0 / M_PI);
        
        return best_angle;
    }
    
    /**
     * @brief Calculate distance from point to line segment
     */
    double distanceToLine(double px, double py, const LineSegment& line)
    {
        double dx = line.x2 - line.x1;
        double dy = line.y2 - line.y1;
        double length_sq = dx * dx + dy * dy;
        
        if (length_sq < 1e-6)
        {
            return std::sqrt(std::pow(px - line.x1, 2) + std::pow(py - line.y1, 2));
        }
        
        double t = std::max(0.0, std::min(1.0, 
            ((px - line.x1) * dx + (py - line.y1) * dy) / length_sq));
        
        double proj_x = line.x1 + t * dx;
        double proj_y = line.y1 + t * dy;
        
        return std::sqrt(std::pow(px - proj_x, 2) + std::pow(py - proj_y, 2));
    }
    
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Add visualization markers
     */
    void addVisualization(visualization_msgs::MarkerArray& vis_array,
                         const geometry_msgs::PoseStamped& marker_pose,
                         const std::vector<LineSegment>& nearby_lines,
                         double corrected_yaw,
                         int marker_id)
    {
        // Visualize nearby lines
        for (size_t i = 0; i < nearby_lines.size(); ++i)
        {
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = base_frame_;
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "nearby_lines";
            line_marker.id = marker_id * 1000 + i;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.02;
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.8;
            
            geometry_msgs::Point p1, p2;
            p1.x = nearby_lines[i].x1;
            p1.y = nearby_lines[i].y1;
            p1.z = 0;
            p2.x = nearby_lines[i].x2;
            p2.y = nearby_lines[i].y2;
            p2.z = 0;
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            line_marker.lifetime = ros::Duration(0.5);
            
            vis_array.markers.push_back(line_marker);
        }
        
        // Visualize original orientation
        visualization_msgs::Marker original_arrow;
        original_arrow.header.frame_id = base_frame_;
        original_arrow.header.stamp = ros::Time::now();
        original_arrow.ns = "original_orientation";
        original_arrow.id = marker_id;
        original_arrow.type = visualization_msgs::Marker::ARROW;
        original_arrow.action = visualization_msgs::Marker::ADD;
        original_arrow.scale.x = 0.3;
        original_arrow.scale.y = 0.05;
        original_arrow.scale.z = 0.05;
        original_arrow.color.r = 0.0;
        original_arrow.color.g = 0.0;
        original_arrow.color.b = 1.0;
        original_arrow.color.a = 0.7;
        
        original_arrow.pose = marker_pose.pose;
        original_arrow.lifetime = ros::Duration(0.5);
        
        vis_array.markers.push_back(original_arrow);
        
        // Visualize corrected orientation
        visualization_msgs::Marker corrected_arrow;
        corrected_arrow.header.frame_id = base_frame_;
        corrected_arrow.header.stamp = ros::Time::now();
        corrected_arrow.ns = "corrected_orientation";
        corrected_arrow.id = marker_id;
        corrected_arrow.type = visualization_msgs::Marker::ARROW;
        corrected_arrow.action = visualization_msgs::Marker::ADD;
        corrected_arrow.scale.x = 0.35;
        corrected_arrow.scale.y = 0.06;
        corrected_arrow.scale.z = 0.06;
        corrected_arrow.color.r = 0.0;
        corrected_arrow.color.g = 1.0;
        corrected_arrow.color.b = 0.0;
        corrected_arrow.color.a = 0.9;
        
        corrected_arrow.pose = marker_pose.pose;
        tf::Quaternion q_corr;
        q_corr.setRPY(0, 0, corrected_yaw);
        tf::quaternionTFToMsg(q_corr, corrected_arrow.pose.orientation);
        corrected_arrow.lifetime = ros::Duration(0.5);
        
        vis_array.markers.push_back(corrected_arrow);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_marker_laser_correction");
    
    ARMarkerLaserCorrection node;
    
    ros::spin();
    
    return 0;
}
