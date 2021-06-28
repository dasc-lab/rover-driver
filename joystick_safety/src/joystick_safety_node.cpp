#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>



const int FORWARD_AXIS_INDEX = 1;
const int TURN_AXIS_INDEX = 3;
const int ARM_BUTTON_INDEX = 2;
const int CMD_MODE_BUTTON_INDEX = 0;
const float MAX_ARM_TIMEOUT = 0.5;


class JoystickNode{
	// instantiate vars, publishers, subscribers
	
	ros::Subscriber desVelSub;
	ros::Publisher cmdVelPub;
	ros::Subscriber joySub;

	bool armStatus = false;
	bool autonomousMode = false;

	float LIN_V_MAX = 1.0;
	float ANG_V_MAX = 1.0;

	ros::Time lastArmMsg;

	bool checkArmIsRecent(){

		return ros::Time::now() < lastArmMsg + ros::Duration(MAX_ARM_TIMEOUT);
		
	}

	void desVelCallback(geometry_msgs::Twist msg){

		if (!autonomousMode){return;}

		if (!armStatus){
			
			publish(0.0, 0.0);

			return;
		}

		if (!checkArmIsRecent()){
			ROS_WARN("DID NOT RECEIVE ARMING MESSAGE FROM JOYSTICK RECENTLY!!");
			armStatus = false;
			publish(0.0,0.0);
		}


		publish(msg.linear.x, msg.angular.z);

	}

	void joyCallback(sensor_msgs::Joy msg){
		ROS_INFO_STREAM("JOYSTICK VALUE: " << msg.axes[FORWARD_AXIS_INDEX]);

		// save the arm status
		armStatus = msg.buttons[ARM_BUTTON_INDEX] == 1;
		autonomousMode  = msg.buttons[CMD_MODE_BUTTON_INDEX] == 1; // if 1, assume autonomous mode, else

		// if not armed, publish 0 speed
		if (!armStatus){
			publish(0.0, 0.0);
			return;
		}

		// if armed and if not in autonomous mode, publish joystick vals
		
		if (!autonomousMode){
			float lin_vel = remap(msg.axes[FORWARD_AXIS_INDEX], LIN_V_MAX);
			float ang_vel = remap(msg.axes[TURN_AXIS_INDEX], ANG_V_MAX);

			publish(lin_vel, ang_vel);

			return;
		};

		return;

	}

	float remap(float val, float max){
		return val * max;
	}

	// publishes lin_vel ang_vel to /cmd_vel
	void publish(float lin_vel, float ang_vel){
		

		// construct msg
		geometry_msgs::Twist msg;

		msg.linear.x = lin_vel;
		msg.angular.z = ang_vel;

		// publish the msg
		cmdVelPub.publish(msg);

		return;

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

	ros::init(argc, argv, "joystick_safety_node");
	
	ros::NodeHandle nh;

	JoystickNode node(nh);

	return 0;

}

