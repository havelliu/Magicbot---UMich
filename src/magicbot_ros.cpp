extern "C"
{
	#include "rc_usefulincludes.h"
	#include "roboticscape.h"
}

#include "magicbot_ros.h"
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <math.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher motor_publisher;
ros::Publisher odom_publisher;
ros::Subscriber rc_cmd;
ros::Subscriber auto_cmd;

rc_filter_t filter1 = rc_empty_filter();
rc_filter_t filter2 = rc_empty_filter();
rc_filter_t filter3 = rc_empty_filter();
rc_filter_t filter4 = rc_empty_filter();
rc_imu_data_t imu_data;

float dutyL = 0.0;
float dutyR = 0.0;
float oldDutyL = 0.0;
float oldDutyR = 0.0;
float x_pos_robot_frame = 0.0;
float y_pos_robot_frame = 0.0;
float x_pos = 0.0;
float y_pos = 0.0;
float auto_linear_desired = 0.0;
float auto_angular_desired = 0.0;
float rc_linear_desired = 0.0;
float rc_angular_desired = 0.0;
float linear_desired = 0.0;
float angular_desired = 0.0;
float leftDistance = 0.0;
float rightDistance = 0.0;
float centerDistance = 0.0;
float leftError = 0.0;
float rightError = 0.0;
float increment = 0.0;
float angle = 0;
float P = .3;
float I = 0;
float D = 0;
float Tf = .04;
float dT = .01;
int currentEncoderLeft = 0;
int prevEncoderLeft = 0;
int currentEncoderRight = 0;
int prevEncoderRight = 0;

void magicbot_controller();
void* setpoint_manager(void* ptr);

void ros_compatible_shutdown_signal_handler(int signo)
{
	if(signo == SIGINT)
	{
		rc_set_state(EXITING);
		ROS_INFO("\nReceived SIGINT Ctrl-C.");
		ros::shutdown();
	}
	else if(signo == SIGTERM)
	{
		rc_set_state(EXITING);
		ROS_INFO("Received SIGTERM.");
		ros::shutdown();
	}
}

void rc_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	rc_linear_desired = cmd->linear.x;
	rc_angular_desired = cmd->angular.z;
	
	return;
}

void auto_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	auto_linear_desired = cmd->linear.x;
	auto_angular_desired = cmd->angular.z;

	return;
}

void RPYtoQuat(float roll, float pitch, float yaw, geometry_msgs::Quaternion& q)
{
	double cy = cos(yaw * .5);
	double sy = sin(yaw * .5);
	double cr = cos(roll * .5);
	double sr = sin(roll * .5);
	double cp = cos(pitch * .5);
	double sp = sin(pitch * .5);

	q.w = cy*cr*cp + sy*sr*sp;
	q.x = cy*sr*cp - sy*cr*sp;
	q.y = cy*cr*sp + sy*sr*cp;
	q.z = sy*cr*cp - cy*sr*sp;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "magicbot_ros_node");
	ros::start();
	ROS_INFO("File %s compiled on %s %s.", __FILE__, __DATE__, __TIME__);
	ros::NodeHandle magicbot_node;

	motor_publisher = magicbot_node.advertise<geometry_msgs::Twist>("magicbot/rc_cmd", 10);
	odom_publisher = magicbot_node.advertise<nav_msgs::Odometry>("odom", 10);
	rc_cmd = magicbot_node.subscribe("magicbot/rc_cmd", 10, rc_callback);
	auto_cmd = magicbot_node.subscribe("cmd_vel", 10, auto_callback);

	if(rc_initialize())
	{
		ROS_INFO("ERROR: Failed to initialize cape.");
		return -1;
	}
	if(rc_initialize_dsm() == -1)
	{
		ROS_INFO("DSM not initialization failed, run binding routine.");
	}
	rc_set_led(RED, 1);
	rc_set_led(GREEN, 0);
	rc_set_state(UNINITIALIZED);
	rc_enable_motors();
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Y_UP;

	if(rc_initialize_imu_dmp(&imu_data, imu_config))
	{
		ROS_INFO("ERROR: Can't talk to the IMU, shutting down.");
		rc_blink_led(RED, 5, 5);
		return -1;
	}

	signal(SIGINT, ros_compatible_shutdown_signal_handler);
	signal(SIGTERM, ros_compatible_shutdown_signal_handler);

	pthread_t setpoint_thread;
	pthread_create(&setpoint_thread, NULL, setpoint_manager, NULL);

	rc_set_imu_interrupt_func(&magicbot_controller);

	rc_set_encoder_pos(ENCODER_CHANNEL_L, 0);
	rc_set_encoder_pos(ENCODER_CHANNEL_R, 0);

	if(rc_pid_filter(&filter1, P, I, D, Tf, dT))
	{
		ROS_INFO("FAILED TO MAKE MOTOR CONTROLLER");
		return -1;
	}
	if(rc_pid_filter(&filter2, P, I, D, Tf, dT))
	{
		ROS_INFO("FAILED TO MAKE MOTOR CONTROLLER");
		return -1;
	}
	if(rc_pid_filter(&filter3, P, I, D, Tf, dT))
	{
		ROS_INFO("FAILED TO MAKE MOTOR CONTROLLER");
		return -1;
	}
	if(rc_pid_filter(&filter4, P, I, D, Tf, dT))
	{
		ROS_INFO("FAILED TO MAKE MOTOR CONTROLLER");
		return -1;
	}

	rc_enable_saturation(&filter1, -1.0, 1.0);
	rc_enable_saturation(&filter2, -1.0, 1.0);
	rc_enable_saturation(&filter3, -1.0, 1.0);
	rc_enable_saturation(&filter4, -1.0, 1.0);

	ROS_INFO("\nMagicbot Initialized\n");
	rc_set_state(RUNNING);

	ros::spin();

	ROS_INFO("Exiting!");
	rc_set_state(EXITING);
	ros::shutdown();

	rc_power_off_imu();
	rc_cleanup();
	
	return 0;
}

void* setpoint_manager(void* ptr)
{
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_set_led(RED, 0);
	rc_set_led(GREEN, 1);

	while(rc_get_state() != EXITING)
	{
		rc_usleep(1000000/SETPOINT_MANAGER_HZ);

		if(rc_get_state() != RUNNING) continue;

		geometry_msgs::Twist msg;
		
		msg.linear.x = rc_get_dsm_ch_normalized(3)*MAX_SPEED;
		msg.angular.z = rc_get_dsm_ch_normalized(4)*MAX_ANGULAR_SPEED;

		motor_publisher.publish(msg);
	}

	pthread_exit(NULL);
}

void magicbot_controller()
{
	if(rc_get_state() == EXITING)
	{
		rc_disable_motors();
		return;
	}

	prevEncoderLeft = currentEncoderLeft;
	prevEncoderRight = currentEncoderRight;
	currentEncoderLeft = rc_get_encoder_pos(ENCODER_CHANNEL_L);
	currentEncoderRight = rc_get_encoder_pos(ENCODER_CHANNEL_R);

	leftDistance = (currentEncoderLeft - prevEncoderLeft)*WHEEL_DIA*3.141592/ENC_COUNT_REV;
	rightDistance = (currentEncoderRight - prevEncoderRight)*WHEEL_DIA*3.141592/ENC_COUNT_REV;
	centerDistance = (leftDistance + rightDistance)/2;

	leftError = leftDistance/IMU_PERIOD;
	rightError = rightDistance/IMU_PERIOD;

	if(rc_get_dsm_ch_normalized(6) > 0.0)
	{
		linear_desired = rc_linear_desired;
		angular_desired = rc_angular_desired;
	}
	else
	{
		linear_desired = auto_linear_desired;
		angular_desired = auto_angular_desired;
	}

	if(fabs(angular_desired) < .5)
	{
		leftError = (linear_desired - leftError)/MAX_SPEED;
		rightError = (linear_desired - rightError)/MAX_SPEED;
	}
	else
	{
		leftError = (linear_desired/MAX_SPEED - angular_desired/MAX_ANGULAR_SPEED) - leftError/MAX_SPEED;

		rightError = (linear_desired/MAX_SPEED + angular_desired/MAX_ANGULAR_SPEED) - rightError/MAX_SPEED;
	}

	dutyL = rc_march_filter(&filter1, leftError);
	dutyR = rc_march_filter(&filter2, rightError);

	increment = (rightDistance - leftDistance)/TRACK_WIDTH;

	if(angle >= 2*3.141592)
	{
		angle = angle - 2*3.141592;
	}
	else if(angle <= -2*3.141592)
	{
		angle = angle + 2*3.141592;
	}

	angle += increment;

	x_pos_robot_frame = -centerDistance*sin(increment);
	y_pos_robot_frame = centerDistance*cos(increment);
	x_pos += x_pos_robot_frame*sin(angle) + y_pos_robot_frame*cos(angle);
	y_pos += y_pos_robot_frame*sin(angle) - x_pos_robot_frame*cos(angle);

	geometry_msgs::Quaternion q;
	RPYtoQuat(0, 0, angle, q);

	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped odom_trans;

	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "/odom";
	odom_trans.child_frame_id = "/magicbot";
	odom_trans.transform.translation.x = x_pos;
	odom_trans.transform.translation.y = y_pos;
	odom_trans.transform.rotation.x = q.x;
	odom_trans.transform.rotation.y = q.y;
	odom_trans.transform.rotation.z = q.z;
	odom_trans.transform.rotation.w = q.w;

	br.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "/map";
	odom.child_frame_id = "/odom";
	odom.pose.pose.position.x = x_pos;
	odom.pose.pose.position.y = y_pos;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = q;
	odom.twist.twist.linear.x = (leftDistance + rightDistance)/2/IMU_PERIOD;
	odom.twist.twist.angular.z = increment/IMU_PERIOD;

	odom_publisher.publish(odom);

	dutyL = linear_desired/MAX_SPEED - angular_desired/MAX_ANGULAR_SPEED;
	dutyR = linear_desired/MAX_SPEED + angular_desired/MAX_ANGULAR_SPEED;

	if(rc_set_motor(MOTOR_CHANNEL_L_F, MOTOR_POLARITY_L_F*dutyL) < 0)
	{
		ROS_INFO("FRONT LEFT MOTOR FAILED");
	}
	if(rc_set_motor(MOTOR_CHANNEL_L_B, MOTOR_POLARITY_L_B*dutyL) < 0)
	{
		ROS_INFO("BACK LEFT MOTOR FAILED");
	}
	if(rc_set_motor(MOTOR_CHANNEL_R_F, MOTOR_POLARITY_R_F*dutyR) < 0)
	{
		ROS_INFO("FRONT RIGHT MOTOR FAILED");
	}
	if(rc_set_motor(MOTOR_CHANNEL_R_B, MOTOR_POLARITY_R_B*dutyR) < 0)
	{
		ROS_INFO("BACK RIGHT MOTOR FAILED");
	}
}
