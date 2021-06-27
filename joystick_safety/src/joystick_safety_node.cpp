#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


const int FORWARD_AXIS_INDEX = 1;
const int ARM_BUTTON_INDEX = 2;



class JoystickNode{
	// instantiate vars, publishers, subscribers
	
	ros::Subscriber desVelSub;
	ros::Publisher cmdVelPub;
	ros::Subscriber joySub;

	void desVelCallback(geometry_msgs::Twist msg){

	}

	void joyCallback(sensor_msgs::Joy msg){
		ROS_INFO_STREAM("JOYSTICK VALUE: " << msg.axes[FORWARD_AXIS_INDEX]);
		ROS_INFO_STREAM("JOYSTICK ARM  : " << msg.buttons[ARM_BUTTON_INDEX]);
	}



	public:
		explicit JoystickNode(ros::NodeHandle nh)
		{
			ROS_INFO("Starting Joystick Node");

			// subscribers
			desVelSub = nh.subscribe("des_vel", 100, &JoystickNode::desVelCallback, this);
			joySub = nh.subscribe("joy", 100, &JoystickNode::joyCallback, this);
			
			// publishers
			cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);	

			// spin
			ros::spin();
		};


};

int main(int argc, char** argv){

	ros::init(argc, argv, "joystick safety node");
	
	ros::NodeHandle nh;

	JoystickNode node(nh);

	return 0;

}

