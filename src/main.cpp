#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>       // vicon_velocity/CHK_M100/CHK_M100  ( x, y, z ) 
#include <geometry_msgs/TransformStamped.h>

#include <std_srvs/SetBool.h>

#include <string>

#define STATE_VICON false
#define STATE_VO    true

geometry_msgs::PoseStamped      currVoPose;
geometry_msgs::PoseStamped      currViconPose;

geometry_msgs::PoseStamped      voOffset;
geometry_msgs::PoseStamped      viconOffset;

geometry_msgs::PoseStamped      voReference;
geometry_msgs::PoseStamped      viconReference;

geometry_msgs::PoseStamped 	outputPose;

ros::Subscriber viconSubscriber;
ros::Subscriber voSubscriber;

// TODO : service "TOGGLE KEY" -> toggled 
ros::ServiceServer toggleServiceServer;

ros::Publisher  posePublisher;

// state selection
bool modeState = STATE_VICON;
bool toggled   = false;

void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
void vo_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
bool toggle_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

void state_allocator();

void initialize_offset();
void set_offset();


int main(int argc, char **argv) {

	std::string viconTopicName,  voTopicName;
	std::string outputTopicName;

	ros::init(argc, argv, "state_changer_node");
	ros::NodeHandle nh;

	ros::param::get("~vicon_topic_name", viconTopicName);
	ros::param::get("~vo_topic_name", voTopicName);
	ros::param::get("~publish_topic_name", outputTopicName);

	ROS_INFO_STREAM("Subscribing vicon topic : " << viconTopicName << ", type : [TransformStamped]");
	ROS_INFO_STREAM("Subscribing VO    topic : " << voTopicName << ", type : [PoseStamped]\n");
	ROS_INFO_STREAM("Publishing  Pose  topic : " << outputTopicName << ", type : [PoseStamped]");

	// Subscribers
	viconSubscriber = nh.subscribe<geometry_msgs::TransformStamped> (viconTopicName, 1, vicon_callback);
	voSubscriber = nh.subscribe<geometry_msgs::PoseStamped> (voTopicName, 1, vo_callback);

	// Service
	toggleServiceServer = nh.advertiseService("state_toggle", toggle_callback);

	// Publisher
	posePublisher = nh.advertise<geometry_msgs::PoseStamped> (outputTopicName, 1);
	
	// offset initialize
	initialize_offset();

	ros::Rate rate(100);
	while(ros::ok()){
		ros::spinOnce();
		if(toggled == true){
			toggled = false;
			set_offset();
			if(modeState == STATE_VO){
				ROS_INFO_STREAM("CHANGE : [VICON] --> [ V O ], offset - x : "<<voOffset.pose.position.x<<", y : "<<voOffset.pose.position.y<<", z : "<<voOffset.pose.position.z);
			}       
			else if(modeState == STATE_VICON){
				ROS_INFO_STREAM("CHANGE : [ V O ] --> [VICON], offset - x : "<<viconOffset.pose.position.x<<", y : "<<viconOffset.pose.position.y<<", z : "<<viconOffset.pose.position.z);
			} 
		}

		state_allocator();
		rate.sleep();
	}
}

void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	currViconPose.pose.position.x = msg->transform.translation.x;
	currViconPose.pose.position.y = msg->transform.translation.y;
	currViconPose.pose.position.z = msg->transform.translation.z;
}

void vo_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	currVoPose.pose.position.x    = msg->pose.position.x;
	currVoPose.pose.position.y    = msg->pose.position.y;
	currVoPose.pose.position.z    = msg->pose.position.z;
}

void initialize_offset(){
	voOffset.pose.position.x      = 0.0;
	voOffset.pose.position.y      = 0.0;
	voOffset.pose.position.z      = 0.0;

	viconOffset.pose.position.x   = 0.0;
	viconOffset.pose.position.y   = 0.0;
	viconOffset.pose.position.z   = 0.0;
}

void state_allocator(){
	if(modeState == STATE_VICON) {
		outputPose.pose.position.x = currViconPose.pose.position.x + viconOffset.pose.position.x;
		outputPose.pose.position.y = currViconPose.pose.position.y + viconOffset.pose.position.y;
		// outputPose.pose.position.z = currViconPose.pose.position.z + viconOffset.pose.position.z;
		outputPose.pose.position.z = currViconPose.pose.position.z;
	}
	else if(modeState == STATE_VO){
		outputPose.pose.position.x = currVoPose.pose.position.x    + voOffset.pose.position.x;
		outputPose.pose.position.y = currVoPose.pose.position.y    + voOffset.pose.position.y;
		// outputPose.pose.position.z = currVoPose.pose.position.z    + voOffset.pose.position.z;
		outputPose.pose.position.z = currViconPose.pose.position.z;
	}
	
	posePublisher.publish(outputPose);
}


void set_offset(){ // when toggled
	if(modeState == STATE_VICON){ // If current = VICON (i.e. now we use VO), VO offsets are modified with current VICON position.
		viconOffset.pose.position.x = outputPose.pose.position.x - currViconPose.pose.position.x;
		viconOffset.pose.position.y = outputPose.pose.position.y - currViconPose.pose.position.y;
		viconOffset.pose.position.z = outputPose.pose.position.z - currViconPose.pose.position.z;
	}
	else if(modeState == STATE_VO) { // If current = VO (i.e. now we use VICON), VICON offsets are modified with current VO position.
		voOffset.pose.position.x = outputPose.pose.position.x - currVoPose.pose.position.x;
		voOffset.pose.position.y = outputPose.pose.position.y - currVoPose.pose.position.y;
		voOffset.pose.position.z = outputPose.pose.position.z - currVoPose.pose.position.z;
	}
}

bool toggle_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
	if(req.data == STATE_VICON){
		modeState = STATE_VICON; // go to VICON mode
		res.success = modeState;
	}

	else if (req.data == STATE_VO) {
		modeState = STATE_VO;    // go to VO    mode
		res.success = modeState;
	}
	ROS_INFO_STREAM("MODE TOGGLED by Service");
	toggled = true;
}
 

