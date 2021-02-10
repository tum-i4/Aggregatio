#include "dirt_detection/dirt_detector.hpp"
#include <iostream>


//Global parameter to enable/disable the dirt detection
//(disabling needed e.g. if the dirt generation should publishes directly the dirt positions and together with an active detection duplicates would be created on topic /detected_dirt)
const int GLOBAL_ENABLE_DETECTION = true;

//Global parameter for the initial trust value which is used for a detected dirt (should be between 0 and 100, where 0 is not reliable dirt and 100 is completely reliable dirt)
const int GLOBAL_TRUST_VALUE = 100;

//Global variable for ID of next detected dirt
int global_id = 1;

//The Constructor
DirtDetector::DirtDetector() : laser_sub(node_, sourceFrame, 10), laser_notifier(laser_sub, listener, "map", 100)
{

    //A private node handle for getting data from the parameter server
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();

    //Getting which robot the node is being executed for
    private_nh.getParam("robot_specifier", robot_specifier);

    //Setting the value of the source frame here which is used by 'laser_sub' to transform the data to the map frame
    sourceFrame = robot_specifier + "/base_scan";

    detected_dirt_topic_ = "/detected_dirt";
    partial_observation_svc = "/partial_observation";
    robot_locations_topic = "/robots_locations";

    // ros::service::waitForService(partial_observation_svc);

    useSubjectiveLogic = true;
    if (!node_.getParam("/use_subjective_logic", useSubjectiveLogic))
    {
        useSubjectiveLogic = false;
    }

    // get robot number
    private_nh.getParam("robot_number", robot_num);
    private_nh.getParam("false_positive", false_positive);
    private_nh.getParam("false_negative", false_negatives_flag);
    private_nh.getParam("false_negative_prob", false_negative_prob);
    private_nh.getParam("num_scan_aggregation", num_scan_aggregation);
    
    laser_scan_counter = 0;

    // get the modified occupancy map
    boost::shared_ptr<nav_msgs::OccupancyGrid const> occ_grid_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(mod_occ_grid_topic_);
    occ_grid = *occ_grid_ptr;
    occ_grid_tolerance = occ_grid.info.resolution / 2;
    task_pos_tolerance = occ_grid.info.resolution / 2;

    // initialise the two lists that store robot-task distances accress "num_scan_aggregation"
    // number of laser scans
    occupied_cells.resize(occ_grid.info.width * occ_grid.info.height);
    unoccupied_cells.resize(occ_grid.info.width * occ_grid.info.height);

    // wait for first publishing of robot position. Otherwise opinions of scan data published
    // in the middle of the map (i.e assumes robot location of 0,0)
    boost::shared_ptr<robot_pose_publisher::RobotLocations const> robot_locations_ptr = ros::topic::waitForMessage<robot_pose_publisher::RobotLocations>(robot_locations_topic);
    robot_locations = robot_locations_ptr->locations;
    
    for (int i = 0; i < robot_locations.size(); i++){
        if (robot_locations[i].robot_id.compare(robot_specifier) == 0){
            r_pose = robot_locations[i].pose;
        }
    }
}

//The Destructor
DirtDetector::~DirtDetector()
{
}

//For Subscribing to all the necessary topics
void DirtDetector::SubscribeToTopics()
{
    // For Robot location
    if (!robot_locations_topic.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), robot_locations_topic.c_str());
        locations_subscriber = node_.subscribe(robot_locations_topic, 10, &DirtDetector::locationsCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), robot_locations_topic.c_str());
    }
    
    //For Subscribing to "map"
    if (!map_topic_.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), map_topic_.c_str());
        map_subscriber = node_.subscribe(map_topic_, 1, &DirtDetector::mapCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), map_topic_.c_str());
    }
    
    //For Subscribing to "modified_occ_grid"
    //if (!mod_occ_grid_topic_.empty())
    //{
    //    ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), mod_occ_grid_topic_.c_str());
     //   occ_grid_subscriber = node_.subscribe(mod_occ_grid_topic_, 1, &DirtDetector::occGridCallBack, this);
    //}
    //else
    //{
    //    ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), mod_occ_grid_topic_.c_str());
    //}

    //For Subscribing to "scan"
    if (!scan_topic_.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), scan_topic_.c_str());
        scan_subscriber = node_.subscribe(scan_topic_, 1, &DirtDetector::scanCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), "scan_topic_");
    }

    //For Subscribing to "active_tasks"
    if (!active_tasks_topic.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), active_tasks_topic.c_str());
        active_tasks_subscriber = node_.subscribe(active_tasks_topic, 1, &DirtDetector::tasksCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), "scan_topic_");
    }

}

//For publishing to the 'detected_dirt' topic
void DirtDetector::PublishDetectedDirt()
{
    ROS_INFO("[%s]: Publisher of topic '%s'", node_name_.c_str(), detected_dirt_topic_.c_str());
    detected_dirt_publisher = node_.advertise<goal_manager_msgs::GoalObject>(detected_dirt_topic_, 100);
}

void DirtDetector::PublishPartialObservation(){
    ROS_INFO("[%s]: Publisher of topic '%s'", node_name_.c_str(), partial_observation_svc.c_str());
    partial_observation_publisher =
        node_.advertise<knowledge_aggregator_msgs::PartialObservation>(partial_observation_svc, 100);
}

void DirtDetector::locationsCallBack(const robot_pose_publisher::RobotLocations::ConstPtr &msg)
{
    robot_locations = msg->locations;

    //ROS_INFO("Robot specifier: %s", robot_specifier.c_str());
    for (int i = 0; i < robot_locations.size(); i++){
        if (robot_locations[i].robot_id.compare(robot_specifier) == 0){
            r_pose = robot_locations[i].pose;
        }
    }

    if (false_negatives_flag)
    {
	    // Calc distance from robot to each false positive, if distance exceeds laser range then
	    // remove from the list of false positives.
	    goal_manager_msgs::GoalObjectList elements_to_be_removed;
	    for (int i = 0; i < false_negatives.goal_list.size(); i++)
	    {
		goal_manager_msgs::GoalObject fn_obj = false_negatives.goal_list[i];
		float dist = std::sqrt(std::pow(r_pose.position.x - fn_obj.pose.position.x,2) + std::pow(r_pose.position.y - fn_obj.pose.position.y,2));
		
		
		if (dist > sensor_range)
		{
			// Out of sensor range so task no longer a false negative and will be removed
			elements_to_be_removed.goal_list.push_back(fn_obj);
		}
	    }
	    
	    for (int i = 0; i < elements_to_be_removed.goal_list.size(); i++)
	    {
		false_negatives.goal_list.erase(std::remove(false_negatives.goal_list.begin(), false_negatives.goal_list.end(), elements_to_be_removed.goal_list[i]), false_negatives.goal_list.end());
	    }
	    // make sure the list of elements to be removed is reset to empty.
	    elements_to_be_removed.goal_list.clear();
	    
	    // Do the same thing for the detected tasks list
	    goal_manager_msgs::GoalObjectList elements_to_be_removed_detected_tasks;
	    for (int i = 0; i < detected_tasks.goal_list.size(); i++)
	    {
		goal_manager_msgs::GoalObject dt_obj = detected_tasks.goal_list[i];
		float dist = std::sqrt(std::pow(r_pose.position.x - dt_obj.pose.position.x,2) + std::pow(r_pose.position.y - dt_obj.pose.position.y,2));
		
		
		if (dist > sensor_range)
		{
			// Out of sensor range so task no longer a false negative and will be removed
			elements_to_be_removed_detected_tasks.goal_list.push_back(dt_obj);
		}
	    }
	    
	    for (int i = 0; i < elements_to_be_removed_detected_tasks.goal_list.size(); i++)
	    {
		detected_tasks.goal_list.erase(std::remove(detected_tasks.goal_list.begin(), detected_tasks.goal_list.end(), elements_to_be_removed_detected_tasks.goal_list[i]), detected_tasks.goal_list.end());
	    }
	    // make sure the list of elements to be removed is reset to empty.
	    elements_to_be_removed_detected_tasks.goal_list.clear();
    }
}

//The call back for subsciption to 'map'
void DirtDetector::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;

    findObstaclesInMap();
}


// The call back for subscription to 'active_tasks'

void DirtDetector::tasksCallBack(const goal_manager_msgs::GoalObjectList::ConstPtr &msg)
{
	active_tasks = *msg;

	// For each element in detected dirts list check that is is inside the active_tasks list.
	// If not, then remove it, as the goal has been attained.
	// (If it were not removed and another tasks spawns in the same location one would have two tasks in the same location in the list which would lead to problems (especially as compare only based on position and not id)
		
	goal_manager_msgs::GoalObjectList elements_to_be_removed_detected_tasks;
	for (int i = 0; i < detected_tasks.goal_list.size(); i++)
	{
		goal_manager_msgs::GoalObject task_object = detected_tasks.goal_list[i];
		if (std::find(active_tasks.goal_list.begin(), active_tasks.goal_list.end(), task_object) == active_tasks.goal_list.end())
		{
			// add to list of elements to be removed.
			elements_to_be_removed_detected_tasks.goal_list.push_back(task_object);
		}
	}
	for (int i = 0; i < elements_to_be_removed_detected_tasks.goal_list.size(); i++)
	{
		detected_tasks.goal_list.erase(std::remove(detected_tasks.goal_list.begin(), detected_tasks.goal_list.end(), elements_to_be_removed_detected_tasks.goal_list[i]), detected_tasks.goal_list.end());
	}
	// make sure the list of elements to be removed is reset to empty.
	elements_to_be_removed_detected_tasks.goal_list.clear();

	// Do the same for the false negative list. (Where other robot might already have attained goal and this robot still sees it as a false negative at that place.)
	goal_manager_msgs::GoalObjectList elements_to_be_removed;
	for (int i = 0; i < false_negatives.goal_list.size(); i++)
	{
		goal_manager_msgs::GoalObject task_object = false_negatives.goal_list[i];
		if (std::find(active_tasks.goal_list.begin(), active_tasks.goal_list.end(), task_object) == active_tasks.goal_list.end())
		{
			// add to list of elements to be removed.
			elements_to_be_removed.goal_list.push_back(task_object);
		}
	}
	for (int i = 0; i < elements_to_be_removed.goal_list.size(); i++)
	{
		false_negatives.goal_list.erase(std::remove(false_negatives.goal_list.begin(), false_negatives.goal_list.end(), elements_to_be_removed.goal_list[i]), false_negatives.goal_list.end());
	}
	// make sure the list of elements to be removed is reset to empty.
	elements_to_be_removed.goal_list.clear();
}

//The call back for subsciption to 'scan'
void DirtDetector::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = *msg;

    sensor_range = scan.range_max;

    if(!listener.waitForTransform(msg->header.frame_id,
                                    "map",
                                    msg->header.stamp +
                                        ros::Duration().fromSec(
                                            msg->ranges.size()*msg->time_increment),
                                    ros::Duration(1.0))){
        ROS_INFO("[%s]: wait for transform failed", robot_specifier.c_str());
        return;
    }
    projector.transformLaserScanToPointCloud("map", *msg, cloud, listener, laser_geometry::channel_option::Distance);
    laser_scan = *msg;
    DetectDirt();
}

bool DirtDetector::comparing_pose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
	// compare the position of measured i.e detected with the ideal position. For them to described the same thing they must be within a tolerance of 0.25
	return (std::abs(pose1.position.x - pose2.position.x)<= task_pos_tolerance && std::abs(pose1.position.y - pose2.position.y)<= task_pos_tolerance);
}


int DirtDetector::get_cell_index(float x, float y){
    int cell_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int cell_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);

    int index = cell_x + cell_y * map.info.width;

    return index;
}

int DirtDetector::occ_grid_index(float x, float y){
    // cell index of row majored list modified occupancy grid.
    int cell_x = static_cast<int>((x - occ_grid.info.origin.position.x) / occ_grid.info.resolution);
    int cell_y = static_cast<int>((y - occ_grid.info.origin.position.y) / occ_grid.info.resolution);

    int index = cell_x + cell_y * occ_grid.info.width;

    return index;
}
bool DirtDetector::is_occupied_(float x, float y)
{
    int index = get_cell_index(x, y);
    return map.data[index] != 0;
}

void DirtDetector::findObstaclesInMap(void){
    float X_MIN_IN = -5.0;
    float X_MAX_IN = 5.0;
    float Y_MIN_IN = -5.0;
    float Y_MAX_IN = 5.0;
    float x_min = map.info.origin.position.x;
    float y_min = map.info.origin.position.y;
    float x_max = x_min + map.info.height * map.info.resolution;
    float y_max = y_min + map.info.width * map.info.resolution;
    float x_step = map.info.resolution;
    float y_step = map.info.resolution;
    geometry_msgs::Point point;

    // Take always the center position of the grid cells
    for (float x = x_min + x_step / 2.; x <= x_max - x_step / 2.; x+=x_step)
    {
        // Take always the center position of the grid cells
        for (float y = y_min + y_step / 2.; y <= y_max - y_step / 2.; y+=y_step)
        {
            // Check if it is inside the movement area of the robots
            if ((x >= X_MIN_IN && x <= X_MAX_IN) && (y >= Y_MIN_IN && y <= Y_MAX_IN))
            {
                // ROS_INFO("Obstacle at (%f,%f)", (float)x, (float)y);
                if (is_occupied_(x, y)){
                    point.x = x;
                    point.y = y;
                    point.z = 0.0;
                    obstacles.push_back(point);

                }
            }
        }
    }
}

bool DirtDetector::isRobotInLocation(float x, float y){
    float robot_size = 0.105*2;
    float robot_radius = robot_size / 2;
    bool isRobot = false;

    for (int i = 0; i < robot_locations.size(); i++)
    {
	// check for all robots except one taking measurements
	if (robot_locations[i].robot_id.compare(robot_specifier) != 0)
	{       
            float r_x = robot_locations[i].pose.position.x;
            float r_y = robot_locations[i].pose.position.y;
            bool not_in_x_axis = false;
            bool not_in_y_axis = false;
        
	    not_in_x_axis = x < (r_x - robot_radius - occ_grid_tolerance) || x > (r_x + robot_radius + occ_grid_tolerance);
	    not_in_y_axis = y < (r_y - robot_radius - occ_grid_tolerance) || y > (r_y + robot_radius + occ_grid_tolerance);

	    if (!(not_in_x_axis && not_in_y_axis))
	    {
                isRobot = true;
		break;
        
	    }
	}
    }

    return isRobot;
}

void DirtDetector::false_negative_execution(goal_manager_msgs::GoalObject task_object, goal_manager_msgs::GoalObject detected_dirt)
{

	// Check if task is not already detected, then it cannot become a false negative until it leaves the sensor range again.
	if (std::find(detected_tasks.goal_list.begin(), detected_tasks.goal_list.end(), task_object) == detected_tasks.goal_list.end())
	{
		// if 'task_object' is not in false negative list check if task becomes it.
		if (std::find(false_negatives.goal_list.begin(), false_negatives.goal_list.end(), task_object) == false_negatives.goal_list.end()) 
		{
			// Task becomes false negative with probability
			bool false_neg = (rand() % 100) < static_cast<int>(100 * false_negative_prob);

			
			if (false_neg == false) 
			{
				detected_tasks.goal_list.push_back(task_object);
				detected_dirt_publisher.publish(detected_dirt);
			}
			else 
			{
				false_negatives.goal_list.push_back(task_object);
				// robot does not detect the task
				task_invisible = true;
			}
		}
	}
	else
	{
		// if it is already detected publish it
		detected_dirt_publisher.publish(detected_dirt);
	}
}

void DirtDetector::DetectDirt()
{
    geometry_msgs::PoseStamped cloudPoint;
    std::vector<geometry_msgs::Pose> dirtCoordinates;


    nav_msgs::OccupancyGrid map_;
    map_ = map;


    std::vector<int> tmp_test_indexes;

    //Iterating over the coordinates received from the transformation of the scan data to the point cloud data
    for (int i = 0; i < cloud.points.size(); i++)
    {
        // Variable to check whether a dirt patch has been detected or not
        bool is_task = false;
	bool is_wall = true;
	bool is_robot = true;
	bool is_in_range = false;
	task_invisible = false;
	bool measurement_ignored = false;


        //Getting the coordinates from the point cloud
        cloudPoint.pose.position.x = cloud.points[i].x;
        cloudPoint.pose.position.y = cloud.points[i].y;
        cloudPoint.pose.position.z = cloud.points[i].z;


	// 1) Check that the measurement is within the sensor range.
	float dist = std::sqrt(std::pow(r_pose.position.x - cloudPoint.pose.position.x,2) + std::pow(r_pose.position.y - cloudPoint.pose.position.y,2));
	if (dist <= sensor_range)
	{
		is_in_range = true;
	}
	
	// 2) Check that measurement is not from a wall

        // Calculate the indeces if occupancy grid were 2D from origin.
	is_wall = occGridWallCheck(cloudPoint);
	
	// 3) Check that measurement is not the other robot
        if (!isRobotInLocation(cloudPoint.pose.position.x, cloudPoint.pose.position.y))
	{
		is_robot = false;
	}
	
	// Determine if the measurement belongs to a task
	if ( (is_wall == false) && (is_robot == false) && (is_in_range == true))
	{
		is_task = true;
	}
	
        //What to do if this is a dirt
        if (is_task == true)
        {
            //Adding the coordinates to a vector
            dirtCoordinates.push_back(cloudPoint.pose);
	    
	    //Preparing the format in which it's supposed to be published
            detectedDirt.pose = cloudPoint.pose;
            detectedDirt.id = global_id;
            global_id++;
            detectedDirt.trust_value = GLOBAL_TRUST_VALUE;

	    if (false_positive)
	    {
		    int missmatch_counter = 0;
	    // Check with which goal it corresponds too and which robot found it too see if it is valid
		    for (int i = 0; i < active_tasks.goal_list.size(); i++)
		    {
			if (comparing_pose(active_tasks.goal_list[i].pose, detectedDirt.pose))
			{
				std::vector<uint16_t> fp_list = active_tasks.goal_list[i].fp;
				
				
				// For priting the fp_list
				std::vector<uint16_t>::iterator it = std::find(fp_list.begin(), fp_list.end(), robot_num);
				int index = std::distance(fp_list.begin(), it);


				// Active task is ground truth or false positive visible to robot
				if (fp_list.size() == 0 or std::find(fp_list.begin(), fp_list.end(), robot_num) != fp_list.end())
				{
					if (false_negatives_flag)
					{
						false_negative_execution(active_tasks.goal_list[i], detectedDirt);
					}
					else 
					{
						detected_dirt_publisher.publish(detectedDirt);
					}
				}
				else 
				{
					// task is false positive of different robot, so invisible
					task_invisible = true;
				}

				// There is only one goal with which is corresponds so avoid rest of loop
				break;
			}
			else
			{
				// measurement did not match task so increment counter
				missmatch_counter = missmatch_counter + 1;
			}
		    }
		    // check if measurement does not correspong to any task within active_tasks, 
		    // then not a real task or list has not been updated yet. So IGNORE measurement.
		    if (missmatch_counter == active_tasks.goal_list.size())
		    {

			measurement_ignored = true;
		    }
	    }
	    else
	    {
		if (false_negatives_flag)
		{
		    int fn_missmatch_counter = 0;
			
		    for (int i = 0; i < active_tasks.goal_list.size(); i++)
		    {
			if (comparing_pose(active_tasks.goal_list[i].pose, detectedDirt.pose))
			{
				false_negative_execution(active_tasks.goal_list[i], detectedDirt);
				break;
			}
			else
			{
				// measurement does not match task so increment counter
				fn_missmatch_counter = fn_missmatch_counter + 1;
			}
		    }

		    // check if measurement does not correspong to any task within active_tasks, 
		    // then not a real task or list has not been updated yet. So IGNORE measurement.
		    if (fn_missmatch_counter == active_tasks.goal_list.size())
		    {

			measurement_ignored = true;
			//ROS_INFO("[Robobt: %d]: MEASUERED UNKNOWN TASK SO PRETEND EMPTY CELL!", robot_num);	
		    }
		}
		else 
		{
			
	     		detected_dirt_publisher.publish(detectedDirt);
		}
	    }


            // ROS_INFO("Robot : [%s], Object Found at at : x = %f, y = %f , \nCountFree_X [%d], CountOccupied_X [%d], \nCountFree_Y [%d], CountOccupied_Y [%d]", robot_specifier.c_str(), detectedDirt.pose.position.x, detectedDirt.pose.position.y, cellCountFree_X, cellCountOccupied_X, cellCountFree_Y, cellCountOccupied_Y);
		
	    
	    // Create a subjective opinion for the current Pose, only if not a wall
	    if (useSubjectiveLogic == true && measurement_ignored == false)
	    {	
		// calc distance between robot and task
		float distance = std::hypot((r_pose.position.x - cloudPoint.pose.position.x), (r_pose.position.y - cloudPoint.pose.position.y));
		int tmp_ind = occ_grid_index(cloudPoint.pose.position.x, cloudPoint.pose.position.y);
		if (task_invisible == false)
		{
		    // robot can detect task and what he detected is a task
		    occupied_cells[tmp_ind].push_back(distance);
		}
		else
		{
		    // robot measured task but not allowed to detect (i.e wrong false positive)
		    unoccupied_cells[tmp_ind].push_back(distance);
		}

		// add to index list so vacuous opinions cannot be published for occupied cell
		tmp_test_indexes.push_back(tmp_ind);
		laser_scan_index_list.push_back(tmp_ind);
	    }
	    else 
	    {
		//ROS_INFO("Measurement Ignored");
	    }
             
        }
    }
    
    
    if (useSubjectiveLogic == true)
    {
        // create free cell opinions for points that have endpoints
        for (int i = 0; i < cloud.points.size(); i++)
        {
            //Getting the coordinates from the point cloud
            cloudPoint.pose.position.x = cloud.points[i].x;
            cloudPoint.pose.position.y = cloud.points[i].y;
            cloudPoint.pose.position.z = cloud.points[i].z;
            
            generateFreeCellOpinions(cloudPoint.pose.position.x, cloudPoint.pose.position.y, false);
        }
    
        // create free cell opinions for laser scans without end-points
        float angle = laser_scan.angle_min;
        for (float tmp_range : laser_scan.ranges)
        {
            // i.e. no measurement just free space along this direction
            if (tmp_range > sensor_range)
            {
                geometry_msgs::PointStamped point_laser_frame;
                point_laser_frame.header.frame_id = sourceFrame;
                point_laser_frame.point.x = sensor_range * cos(angle);
                point_laser_frame.point.y = sensor_range * sin(angle);
                point_laser_frame.point.z = 0;
                
                try
                {
                	// transform point into map frame
            		geometry_msgs::PointStamped point_map_frame;
            		point_map_frame.header.frame_id = "map";
            		listener.transformPoint("map", point_laser_frame, point_map_frame);
                	
            		// generate free opinions
            		generateFreeCellOpinions(point_map_frame.point.x, point_map_frame.point.y, true);
                
                }
                catch(tf::TransformException& ex)
                {
            	    // Transform exception, move to next point
                    ROS_ERROR("Transformation error from laser to map frame:  %s", ex.what());
            	    angle = angle + laser_scan.angle_increment;
            	    continue;
                }
            }
            angle = angle + laser_scan.angle_increment;
        }
        
        // End of laser scan so increment counter
        laser_scan_counter = laser_scan_counter + 1;

    	// if number of laser scans to be aggregated is achieved, aggregate results and publish
    	if (laser_scan_counter == num_scan_aggregation)
	{
    	    knowledge_aggregator_msgs::PartialObservation opinions;
            knowledge_aggregator_msgs::SO so;
	   	 
	    // aggregate results
	    for (int k = 0; k < occupied_cells.size(); k++)
	    {
		// Occupied information superseeds unoccupied information
		if (occupied_cells[k].size() > 0)
		{
		    // calc average distance and generate occupied opinion
		    float avg_dist = std::accumulate(std::begin(occupied_cells[k]), std::end(occupied_cells[k]), 0.0) / occupied_cells[k].size();
		    so = createSubjectiveOpinion(avg_dist, k, true); // k is cell number
		    opinions.partial_observation.push_back(so);
		}
		else if (unoccupied_cells[k].size() > 0)
		{
		    // retrieve distance and generate occupied opinion
		    float dist = unoccupied_cells[k][0];
		    so = createSubjectiveOpinion(dist, k, false); // k is cell number
		    opinions.partial_observation.push_back(so);
		}
	    }
	    
	    // publishing
            if (opinions.partial_observation.size() > 0)
	    {
                partial_observation_publisher.publish(opinions);
            
	    }
	    // reset variables
	    laser_scan_counter = 0;
    	    opinions.partial_observation.clear();

	    // reset sub-lists of two lists without changing the size of the list
	    for (int l = 0; l < occupied_cells.size(); l++)
	    {
		occupied_cells[l].clear();
		unoccupied_cells[l].clear();
	    }
    	    
	    // Only allow one subjective opinion per cell for num_scan_aggregation scans.
	    // Here finished a set of scans and can now clear list preventing new unoccupied SO.
    	    laser_scan_index_list.clear();
	}
    }
}

void DirtDetector::generateFreeCellOpinions(float point_x, float point_y, bool free_end_point)
{

	// Creat the opinions for the unoccupied cells within the robot vision.
	// If the range was finit i.e. smaller than the sensor range (meaning a measurement
	// of walls, other robots or tasks) then free_end_point = false, else it should be true
    	knowledge_aggregator_msgs::PartialObservation free_opinions;
	int endpoint_index;

	// first determine the cells along the laser beam from the robot to this cloud point
	// y = mx + c, grad = x1 - x2 / y1 - y2

	//std::cout << "ROBOT POS: X, Y : " << r_pose.position.x << " " << r_pose.position.y << '\n';
	float grad = (r_pose.position.y - point_y) / (r_pose.position.x - point_x);
	float c_const = r_pose.position.y - grad * r_pose.position.x;

	// increment along the laser beam from robot to cloudPoint
	float x_increment = occ_grid.info.resolution / 5;
	if (point_x < r_pose.position.x)
	{
	    x_increment = -1 * x_increment;
	}
	
	float shift;
	if (free_end_point)
	{
	    endpoint_index = occ_grid_index(point_x, point_y);
	}
	else
	{
	    // give it an 'impossible' value
	    endpoint_index = -1;
	}

	for (float x = r_pose.position.x; std::abs(x - point_x) > std::abs(x_increment); x = x + x_increment)
	{
	    float y = grad * x + c_const;
	    int tmp_index = occ_grid_index(x, y);
	    // check that point is not inside a wall. (This time dont need to check surrounding)
	    if (tmp_index != endpoint_index && occ_grid.data[tmp_index] == 0)
	    {
	        // Within one laser scan only one opinion should be generated per free cell.
	        if (std::find(laser_scan_index_list.begin(), laser_scan_index_list.end(), tmp_index) == laser_scan_index_list.end())
	        {
	            // When not in the list add it and distance to task
		    float distance = std::hypot((r_pose.position.x - x), (r_pose.position.y - y));
	    	    unoccupied_cells[tmp_index].push_back(distance);
	        }
	    }
	}
}
bool DirtDetector::occGridWallCheck(geometry_msgs::PoseStamped cloudPoint)
{
	// This function checks if the given point is within a wall withn a given tolerance.
	// move point by +- occ_grid_tolerance in the x and y directions and check if wall
	// in new index. Only if none of them contain a wall the measurement is considered
	// not to be a wall i.e. is_wall = false.
	// This is needed as measurement are placed at the wall surface. The inherent noise in
	// the measurements my place the 'wall measurement' into a grid cell that is unoccupied
	// so it believes that it is a spawned task and not a wall.
	bool is_wall = true;
	int free_cells = 0;
	for (float delta_x = -1 * occ_grid_tolerance; delta_x <= occ_grid_tolerance; delta_x = delta_x + occ_grid_tolerance)
	{
		for (float delta_y = -1 * occ_grid_tolerance; delta_y <= occ_grid_tolerance; delta_y = delta_y + occ_grid_tolerance)
		{
			float x_shift = cloudPoint.pose.position.x + delta_x;
			float y_shift = cloudPoint.pose.position.y + delta_y;
			int index_shift = occ_grid_index(x_shift, y_shift);
			if ((occ_grid.data[index_shift] == 0))
			{
				free_cells = free_cells + 1;
			}
			else
			{
				break;
			}
		}
	}
	if (free_cells == 9)
	{
		is_wall = false;
	}
	return is_wall;

}
/*!
 * Generates a new subjective opinion for the given Pose
 */
knowledge_aggregator_msgs::SO DirtDetector::getSubjectiveOpinion(geometry_msgs::PoseStamped pose_stamped_laser_map,
		bool isDirt) {

	float x_tgt = pose_stamped_laser_map.pose.position.x;
	float y_tgt = pose_stamped_laser_map.pose.position.y;
	float x_src = r_pose.position.x;
	float y_src = r_pose.position.y;

	float unit = 1.0;
	float belief = 0.0;
	float disbelief = 0.0;
	float uncertainty = 0.0;
	float atomicity = 0.5;
	//float distance = map.info.resolution * std::hypot((x_tgt - x_src), (y_tgt - y_src));
	float distance = std::hypot((x_tgt - x_src), (y_tgt - y_src));

	float u = 0.0;
        if (distance >= scan.range_max)
            u = 0.99;
        else
            u = distance / scan.range_max;

	uncertainty = std::min(unit, u);
	if (isDirt) {
		belief = unit - uncertainty;
		disbelief = 0.0;
	} else {
		disbelief = unit - uncertainty;
		belief = 0.0;
	}

	knowledge_aggregator_msgs::SO so;
	so.pose = pose_stamped_laser_map.pose;
	so.belief = belief;
	so.disbelief = disbelief;
	so.uncertainty = uncertainty;
	so.base_rate = atomicity;

	return so;
}

knowledge_aggregator_msgs::SO DirtDetector::createSubjectiveOpinion(float distance, int cell_num, bool isDirt) 
{
	// This function generates a subjectiive opinion for a given distance "distance" between
	// the robot and the task in question. The task is located in cell cell_num.
	
	float unit = 1.0;
	float belief = 0.0;
	float disbelief = 0.0;
	float uncertainty = 0.0;
	float atomicity = 0.5;

	float u = 0.0;
        if (distance >= scan.range_max)
            u = 0.99;
        else
            u = distance / scan.range_max;

	uncertainty = std::min(unit, u);
	if (isDirt) {
		belief = unit - uncertainty;
		disbelief = 0.0;
	} else {
		disbelief = unit - uncertainty;
		belief = 0.0;
	}

	// calculate center of cell from cell number:
    	int cell_y = static_cast<int>(cell_num / occ_grid.info.width);
	int cell_x = cell_num - cell_y * occ_grid.info.width;
	float x_pos = occ_grid.info.resolution * cell_x + occ_grid.info.origin.position.x + occ_grid.info.resolution/2;
	float y_pos = occ_grid.info.resolution * cell_y + occ_grid.info.origin.position.y + occ_grid.info.resolution/2;

	knowledge_aggregator_msgs::SO so;
	so.pose.position.x = x_pos;
	so.pose.position.y = y_pos;
	so.pose.position.z = 0.0;
	so.belief = belief;
	so.disbelief = disbelief;
	so.uncertainty = uncertainty;
	so.base_rate = atomicity;

	return so;
}
//Getting the mid point of the maximum and minimum values for the obstacle along the x axis
float DirtDetector::GetDirtX(std::vector<geometry_msgs::Pose> dirtCoordinates)
{
    float min = dirtCoordinates[0].position.x;
    float max = dirtCoordinates[0].position.x;
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.x < min)
        {
            min = dirtCoordinates[i].position.x;
        }
    }
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.x > max)
        {
            max = dirtCoordinates[i].position.x;
        }
    }
    return (min + max) / 2;
}

//Getting the mid point of the maximum and minimum values for the obstacle along the y axis
float DirtDetector::GetDirtY(std::vector<geometry_msgs::Pose> dirtCoordinates)
{
    float min = dirtCoordinates[0].position.y;
    float max = dirtCoordinates[0].position.y;
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.y < min)
        {
            min = dirtCoordinates[i].position.y;
        }
    }
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.y > max)
        {
            max = dirtCoordinates[i].position.y;
        }
    }
    return (min + max) / 2;
}

void DirtDetector::spin()
{
    /*ros::Rate r(0.5);
    while (ros::ok())
    {
        DetectDirt();
        ros::spinOnce();
        r.sleep();
    }*/

    ros::spin();
}

std::string DirtDetector::getRobotId(void){
    return robot_specifier;
}

int main(int argc, char **argv)
{
    if (GLOBAL_ENABLE_DETECTION == true)
    {
        ros::init(argc, argv, "goal_detector");
        DirtDetector DirtDetector;
        DirtDetector.SubscribeToTopics();
        DirtDetector.PublishDetectedDirt();
        DirtDetector.PublishPartialObservation();

        DirtDetector.spin();
    }
    return 0;
}
