#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include "goal_manager_msgs/GoalObject.h"
#include "goal_manager_msgs/GoalObjectList.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/tfMessage.h>
#include <map>
#include <cmath>
#include <random>
#include <algorithm>
#include "knowledge_aggregator_msgs/SO.h"
#include "knowledge_aggregator_msgs/DoAggregation.h"
#include "knowledge_aggregator_msgs/PartialObservation.h"
#include "robot_pose_publisher/RobotLocation.h"
#include "robot_pose_publisher/RobotLocations.h"

class DirtDetector
{
    //For node handling
    std::string node_name_;
    ros::NodeHandle node_;

    //For receiving data from the parameter server 
    std::string robot_specifier;
    std::string sourceFrame;

    //Topics to be Subscribed to, /robot_# is automatically added to it topics not starting with '/'
    std::string map_topic_ = "map"; //"/robot_0/map";
    std::string scan_topic_ = "scan";
    std::string robot_locations_topic = "robots_locations";
    std::string active_tasks_topic = "/active_tasks";
    std::string mod_occ_grid_topic_ = "/modified_occupancy_grid";

    std::string partial_observation_svc = "partial_observation";

    //Topic to publish at
    std::string detected_dirt_topic_ = "detected_dirt";

    //Subscribers
    ros::Subscriber map_subscriber;
    ros::Subscriber occ_grid_subscriber;
    ros::Subscriber scan_subscriber;
    ros::Subscriber active_tasks_subscriber;

    ros::Subscriber locations_subscriber;

    ros::ServiceClient doAggregation;
    ros::ServiceClient getLocations;

    //Publisher
    ros::Publisher detected_dirt_publisher;
    ros::Publisher partial_observation_publisher;

    //For publishing in a format that's recognizable by the GoalList Node
    goal_manager_msgs::GoalObject detectedDirt;

    //knowledge_aggregator::SubjectiveOpinionList so;
    std::vector<knowledge_aggregator_msgs::SO> partialObservation;

    //For getting the occupancy map
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid occ_grid;
    // For getting the laser scan
    sensor_msgs::LaserScan scan;
    // Getting the active tasks
    goal_manager_msgs::GoalObjectList active_tasks;

    geometry_msgs::Pose r_pose;

    std::vector<robot_pose_publisher::RobotLocation> robot_locations;
    std::vector<geometry_msgs::Point> obstacles;

    // lists that store robot-task distance across laser scans
    // (index of list = cell index of occ_grid)
    std::vector<std::vector<float>> occupied_cells;
    std::vector<std::vector<float>> unoccupied_cells;

    // the number of laser scans that should be aggregated
    int num_scan_aggregation;
    int laser_scan_counter; // counts number of laser scans

    //Laser Point Cloud for transforming the values gotten through the laser scan
    //into points on the map
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
    sensor_msgs::PointCloud cloud;
    sensor_msgs::LaserScan laser_scan;

    bool useSubjectiveLogic;

    std::vector<int>  laser_scan_index_list;

private:
    // For finding the dirt and publishing it
    void DetectDirt();

public:

    void locationsCallBack(const robot_pose_publisher::RobotLocations::ConstPtr &msg);
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void occGridCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void tasksCallBack(const goal_manager_msgs::GoalObjectList::ConstPtr &msg);
    float GetDirtX(std::vector<geometry_msgs::Pose> dirtCoordinate);
    float GetDirtY(std::vector<geometry_msgs::Pose> dirtCoordinate);
    DirtDetector();
    ~DirtDetector();
    void SubscribeToTopics();
    void PublishDetectedDirt();
    void PublishPartialObservation();
    void spin();
    void false_negative_execution(goal_manager_msgs::GoalObject task_object, goal_manager_msgs::GoalObject detected_dirt);

    // constant for comparing detected values with goals
    float task_pos_tolerance;
    int robot_num;
    bool false_positive;
    bool task_invisible;
    bool false_negatives_flag;
    float false_negative_prob;
    float occ_grid_tolerance;
    knowledge_aggregator_msgs::SO getSubjectiveOpinion(geometry_msgs::PoseStamped, bool isDirt);
    bool occGridWallCheck(geometry_msgs::PoseStamped);
    void generateFreeCellOpinions(float point_x, float point_y, bool free_end_point);
    knowledge_aggregator_msgs::SO createSubjectiveOpinion(float distance, int cell_num, bool isDirt);
    
    goal_manager_msgs::GoalObjectList false_negatives;
    goal_manager_msgs::GoalObjectList detected_tasks;
    float sensor_range;    
    int get_cell_index(float x, float y);
    int occ_grid_index(float x, float y);
    bool is_occupied_(float x, float y);
    bool comparing_pose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
    void findObstaclesInMap(void);
    bool isRobotInLocation(float x, float y);
    std::string getRobotId(void);
};
