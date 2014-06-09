#ifndef ZDRIVE_H
#define ZDRIVE_H


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "dynamixel_servo/DynamixelConfigParam.h" //message to config a Dynamixel servo
#include "dynamixel_servo/DynamixelConfigPosition.h" //simplified message to just set the position of a servo.
#include "dynamixel_servo/DynamixelStatusParam.h" //message for the status of a Dynamixel servo
#include "motor_control/thrusterConfig.h"
#include "motor_control/thrusterStatus.h"
#include "z_drive/ZDriveDbg.h"
#include "uf_common/MoveToAction.h"
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <math.h>

#include <boost/progress.hpp>
#include <stdlib.h>

// Per Ros cpp guidelines you should only refer to each element in namespace std that you want. see: http://wiki.ros.org/CppStyleGuide#Namespaces-1
using std::string;


class ZDrive
{
private:
	// Information relative to the boat that is used in tuning the controller
	static const double boat_mass; //Kg
	static const double boat_inertia; //(m^2)/Kg

	// remember according to ros rep 103: x=forward, y=left,	z=up
	// Currently the root of the z_drive tf tree is centered at the base_link
	static const double z_drive_root_x_offset;
	static const double z_drive_root_y_offset;
	static const double z_drive_root_z_offset;
	// port_servo is back .7239m, left of centerline .3048m, 0m above the water
	static const double port_servo_x_offset;
	static const double port_servo_y_offset;
	static const double port_servo_z_offset;
	// starboard_servo is back .7239m, right of center line .3048m, 0m above the water
	static const double starboard_servo_x_offset;
	static const double starboard_servo_y_offset;
	static const double starboard_servo_z_offset;

	// These are the tunable gain parameters for the "pd" controller. (Note: it is not a "pid" controller)
	static const double p_gain_x;
	static const double d_gain_x;
	static const double p_gain_y;
	static const double d_gain_y;
	// this is the angle (in radian) of the boat relative to the world coordinate frame
	static const double p_gain_theta_boat;
	static const double d_gain_theta_boat;

	// these are gains for the cost function
	static const double gain_error_force_x;
	static const double gain_error_force_y;
	static const double gain_error_moment_z;
	static const double gain_thrusters_force; // all thrusters will have the same gain amount for the thrusters. This essentially is to penalize the system (in the cost function) for using more energy.
	static const double gain_deviation_equilibrum_servo_angle; // all servos want to be pointing towards equilibrium/center (to minimize the motion used by the system). Note: since all servos are the same (e.g. same specs) they have the same gain_deviation_equilibrum_servo_angle
	static const double gain_deviation_changeof_servo_angle; // we want to also minimize the overall amount in which the servos will have to turn (to minimize the motion used by the system). Note: since all servos are the same (e.g. same specs) they have the same gain_deviation_changeof_servo_angle

	// these are is the topic names to listen to for various messages.
	static const string odom_topic;
	static const string dynamixel_namespace;
	static const string dynamixel_status_topic;
	static const string dynamixel_config_topic;
	static const string thruster_status_topic;
	static const string thruster_config_topic;

	// When working to minimize the cost function, a buffer is used to get the min in a range. This is the size of that buffer/range.
	static const int cost_count_max;

	// All the partial derivative functions will use these temporary variables in their calculations. To make the code look cleaner and to not have to recalculate them with each call to the partial derivatives, they have been made members of the class.
	// remember according to ros rep 103: x=bow=forward, y=port=left
	// the required force is what we would want in an ideal world
	double force_port_required; // =requiredForceY()
	double force_bow_required; // =requiredForceX()
	double moment_z_required; // =requiredMomentZ()

	// this is the step amount that is used when finding the min of our cost function, this amount will change dynamicaly
	static const double step_size;
	static const double step_multiplier;
	double step;

	// These values will converge closer and closer to the optimal solution- hence why they are an estimate- with every iteration of calcAngleAndThrustEstimate.
	double estimated_port_servo_angle;
	double estimated_port_thruster_force;
	double estimated_starboard_servo_angle;
	double estimated_starboard_thruster_force;

	// These are locked at the beginning of a calculation of the cost optimization function
	double current_port_servo_angle;
	double current_port_thruster_force;
	double current_starboard_servo_angle;
	double current_starboard_thruster_force;

	// These are the bounds of the servos (angles are in radians and thrust is in newtons). Note: the thrusters can generate thrust in the forward or reverse direction.
	// Note: The Servo Angle limits here will comply with REP103; so 0 radians coincides with the bow, +pi/2 coincides with port, and -pi/2 coincides with starboard.
	double port_servo_angle_clock_wise_limit;
	double port_servo_angle_counter_clock_wise_limit;
	double port_thruster_reverse_limit;
	double port_thruster_foward_limit;
	double starboard_servo_angle_clock_wise_limit;
	double starboard_servo_angle_counter_clock_wise_limit;
	double starboard_thruster_reverse_limit;
	double starboard_thruster_foward_limit;

	// the servo(s) are an a belt drive. also the servos are in joint mode. in joint mode the servo can't smoothly transition from 360->0, it will go 360->359...1->0; hence the belt drive can eliminate this.
	// This function assumes that 0rad corresponds to 0 rad on the servo, so we will adjust the results we estimate and get so that it is.
	double port_servo_angle_offset;
	double starboard_servo_angle_offset;

	// These are the ID(s) of the servo(s) in the z_drive- since each servo on a dynamixel bus must have a unique id.
	// Note: The id field in a DynamixelStatusParam msg is a uint8, so the id(s) here are also uint8(s) to avoid type casting.
	uint8_t port_servo_id;
	uint8_t starboard_servo_id;

	// these are simplifications for the current odom information of the boat
	// remember according to ros rep 103: x=forward, y=left,	z=up
	double x_current;
	double x_velocity_current;
	double y_current;
	double y_velocity_current;
	double yaw_current;
	double yaw_velocity_current;
	double roll_current;
	double roll_velocity_current;
	double pitch_current;
	double pitch_velocity_current;
	// these are simplifications for the desired (what we want the boat to do) odom information of the boat
	// remember according to ros rep 103: x=forward, y=left,	z=up
	double x_desired;
	double x_velocity_desired;
	double y_desired;
	double y_velocity_desired;
	double yaw_desired;
	double yaw_velocity_desired;
	double roll_desired;
	double roll_velocity_desired;
	double pitch_desired;
	double pitch_velocity_desired;

	// these are to just temporarily store the messages that are taken in the callback(s)- for debuging purposes only.
	// Note: We will then strip out the simplified information that we care about
	nav_msgs::Odometry odom_current;
	nav_msgs::Odometry odom_desired;

	ros::NodeHandle n;
	ros::Subscriber odom_subscriber;
	ros::Subscriber thruster_status_subscriber;
	ros::Subscriber dynamixel_status_subscriber;
	ros::Publisher dynamixel_config_full_pub;
	ros::Publisher dynamixel_config_position_pub;
	ros::Publisher thruster_config_pub;
	ros::Publisher z_drive_dbg_pub;
	z_drive::ZDriveDbg dbg_msg;
	tf::TransformBroadcaster tf_brodcaster;
	tf::Transform z_drive_root_tf;
	tf::Transform port_tf;
	tf::Transform starboard_tf;
	double update_rate;
	ros::Publisher joint_pub;
	sensor_msgs::JointState joint_state;

public:
	ZDrive();
	void run();

protected:
	void currentOdomCallBack(const nav_msgs::Odometry& odom_msg);
	void desiredOdomCallBack(const  uf_common::PoseTwist desired_pose_twist);
	void dynamixelStatusCallBack(const dynamixel_servo::DynamixelStatusParam& dynamixel_status_msg);
	void thrusterStatusCallBack(const motor_control::thrusterStatus& thruster_status_msg);

	// remember x is forward, y is left right, and z is up
	// Note: in an ideal system the resultant force/moment would equal the required force/moment, but our system isn't ideal; thus we must calculate both.
	// this is the force/moment that can be provided to the system so we can move to our desired position
	double resultantForceX(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force);
	double resultantForceY(double estimated_port_servo_angle, double estimated_port_thruster_force, double estimated_starboard_servo_angle, double estimated_starboard_thruster_force);
	double resultantMomentZ(double estimated_port_servo_angle, double estimated_port_thruster_force, double estimated_starboard_servo_angle, double estimated_starboard_thruster_force);
	// this is what we want the system to do, but it may not be able to.
	double requiredForceX(double x_current, double x_desired, double x_velocity_current, double x_velocity_desired);
	double requiredForceY(double y_current, double y_desired, double y_velocity_current, double y_velocity_desired);
	double requiredMomentZ(double boat_angle_current, double boat_angle_desired, double boat_angular_velocity_current, double boat_angular_velocity_desired);
	// since the step size varies with each iteration to find the min of the cost function, this will calculate the new step_size amount
	void minimizeCostFunction();
	void calcAngleAndThrustEstimate();
	void guessInitalValues();
	double calcCost(double port_servo_angle, double port_thruster_force, double starboard_servo_angle, double starboard_thruster_force);
	void checkEstimatedValuesAgainsBounds();
	// these are the partial derivatives for the cost function
	double dCost_dPortServoAngle(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force);
	double dCost_dPortThrusterForce(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force);
	double dCost_dStarboardServoAngle(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force);
	double dCost_dStarboardThrusterForce(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force);
	// this will pull all the curent information form the class and post it in a dbg msg
	void publishDbgMsg(double user_defined_1, double user_defined_2, double user_defined_3, double user_defined_4, double user_defined_5);
};
// Information relative to the boat that is used in tuning the controler
const double ZDrive::boat_mass=36.2874; //Kg
const double ZDrive::boat_inertia=7.4623; //(m^2)/Kg

// remember according to ros rep 103: x=forward, y=left,	z=up
// Currently the root of the z_drive tf tree is centered at the base_link
const double ZDrive::z_drive_root_x_offset=0.0;
const double ZDrive::z_drive_root_y_offset=0.0;
const double ZDrive::z_drive_root_z_offset=0.0;
// port_servo is back .7239m, left of centerline .3048m, 0m above the water
const double ZDrive::port_servo_x_offset=-.7239;
const double ZDrive::port_servo_y_offset=.3048;
const double ZDrive::port_servo_z_offset=0.0;
// starboard_servo is back .7239m, right of center line .3048m, 0m above the water
const double ZDrive::starboard_servo_x_offset=-.7239;
const double ZDrive::starboard_servo_y_offset=-.3048;
const double ZDrive::starboard_servo_z_offset=0.0;

// These are the tunable gain parameters for the "pd" controller. (Note: it is not a "pid" controler)
const double ZDrive::p_gain_x=50;
const double ZDrive::d_gain_x=sqrt(4*p_gain_x*boat_mass);
const double ZDrive::p_gain_y=50;
const double ZDrive::d_gain_y=sqrt(4*p_gain_y*boat_mass);
// this is the angle (in radian) of the boat relative to the world coordinate frame
const double ZDrive::p_gain_theta_boat=70;
const double ZDrive::d_gain_theta_boat=sqrt(4*p_gain_theta_boat*boat_inertia);

// these are gains for the cost function
const double ZDrive::gain_error_force_x=1000;
const double ZDrive::gain_error_force_y=1000;
const double ZDrive::gain_error_moment_z=1000;
const double ZDrive::gain_thrusters_force=10; // all thrusters will have the same gain amount for the thrusters. This essentially is to penalize the system (in the cost function) for using more energy.
const double ZDrive::gain_deviation_equilibrum_servo_angle=0; // all servos want to be pointing towards equilibrium/center (to minimize the motion used by the system). Note: since all servos are the same (e.g. same specs) they have the same gain_deviation_equilibrum_servo_angle
const double ZDrive::gain_deviation_changeof_servo_angle=1; // we want to also minimize the overall amount in which the servos will have to turn (to minimize the motion used by the system). Note: since all servos are the same (e.g. same specs) they have the same gain_deviation_changeof_servo_angle

const string ZDrive::odom_topic="sim_odom";
const string ZDrive::dynamixel_namespace="dynamixel";
const string ZDrive::dynamixel_status_topic="dynamixel_status_post";
const string ZDrive::dynamixel_config_topic="dynamixel_config";
const string ZDrive::thruster_status_topic="thruster_status_post";
const string ZDrive::thruster_config_topic="thruster_config";

const double ZDrive::step_size=.0001;
const double ZDrive::step_multiplier=10;

const int ZDrive::cost_count_max=20;


ZDrive::ZDrive(): force_port_required(0.0), force_bow_required(0.0), moment_z_required(0.0), estimated_port_thruster_force(0.0), estimated_port_servo_angle(0.0), estimated_starboard_thruster_force(0.0), estimated_starboard_servo_angle(0.0)
{
	// While this should not be an issue on most computers, this check is added to try and ensure that doubles (like ros float64 msg types) are ieee754 compliant. See: http://goo.gl/mfC8tJ and http://goo.gl/AN8311
	if(std::numeric_limits<double>::is_iec559==false)
	{
		ROS_WARN("The variable type \"double\" in c++ should be in ieee754 format to comply with ros \"float64\" primitive type. See http://wiki.ros.org/msg");
	}

	// init values
	// Note: The Servo Angle limits here will comply with REP103; so 0 radians coincides with the bow, +pi/2 coincides with port, and -pi/2 coincides with starboard.
	port_servo_angle_clock_wise_limit=(-50*M_PI)/180; // clockwise is towards starboard
	port_servo_angle_counter_clock_wise_limit=(100*M_PI)/180;; //counterclockwise is towards port
	port_thruster_reverse_limit=-100;
	port_thruster_foward_limit=100;
	starboard_servo_angle_clock_wise_limit=(-100*M_PI)/180; // clockwise is towards starboard
	starboard_servo_angle_counter_clock_wise_limit=(50*M_PI)/180;; //counterclockwise is towards port
	starboard_thruster_reverse_limit=-100;
	starboard_thruster_foward_limit=100;
	port_servo_angle_offset=M_PI;
	starboard_servo_angle_offset=M_PI;

	// init all the values for what we will estimate
	current_port_servo_angle=0.0;
	current_port_thruster_force=0.0;
	current_starboard_servo_angle=0.0;
	current_starboard_thruster_force=0.0;
	//init all the odom messages so a normalization error isn't generated due to the fact that the estimation routine runs before a callback happens initaly.
	odom_current.pose.pose.position.x=0;
	odom_current.pose.pose.position.y=0;
	odom_current.pose.pose.position.z=0;
	odom_current.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	odom_desired.pose.pose.position.x=0;
	odom_desired.pose.pose.position.y=0;
	odom_desired.pose.pose.position.z=0;
	odom_desired.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

	/*
	// This is temporary and will be the desired odom
	odom_desired.header.stamp=ros::Time::now();
	odom_desired.header.frame_id=odom_topic;
	odom_desired.child_frame_id="base_link";
	odom_desired.pose.pose.position.x=10;
	odom_desired.pose.pose.position.y=10;
	odom_desired.pose.pose.position.z=0;
	odom_desired.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	// TWIST part of an odom message is relative to the BODY frame!
	// POSE part of the odom message is relative to the WORLD frame!
	ZDrive::x_desired=odom_desired.pose.pose.position.x;
	ZDrive::y_desired=odom_desired.pose.pose.position.y;
	// extract the relevant information based on Quaternions
	tf::Quaternion q_desired;
	tf::quaternionMsgToTF(odom_desired.pose.pose.orientation, q_desired);
	tf::Matrix3x3(q_desired).getRPY(ZDrive::roll_desired, ZDrive::pitch_desired, ZDrive::yaw_desired);
	// the velocity(s) needs to be relative to the world frame
	// first we will get the linear velocities relative to the world frame
	tf::Vector3 linear_body_velocity;
	tf::vector3MsgToTF(odom_desired.twist.twist.linear, linear_body_velocity);
	tf::Vector3 linear_world_velocity = tf::Matrix3x3(q_desired) * linear_body_velocity;
	ZDrive::x_velocity_desired=linear_world_velocity.x();
	ZDrive::y_velocity_desired=linear_world_velocity.y();
	// now the angular velocities relative to the world frame
	tf::Vector3 angular_body_velocity;
	tf::vector3MsgToTF(odom_desired.twist.twist.angular, angular_body_velocity);
	tf::Vector3 angular_world_velocity =tf::Matrix3x3(q_desired) * angular_body_velocity;
	ZDrive::roll_velocity_desired=angular_world_velocity.x();
	ZDrive::pitch_velocity_desired=angular_world_velocity.y();
	ZDrive::yaw_velocity_desired=angular_world_velocity.z();
	*/

	//create a parent node in the tree to organize the zdrive components under
	z_drive_root_tf.setOrigin(tf::Vector3(z_drive_root_x_offset, z_drive_root_y_offset, z_drive_root_z_offset));
	z_drive_root_tf.setRotation(tf::Quaternion(0,0,0));

	port_tf.setOrigin(tf::Vector3(port_servo_x_offset,port_servo_y_offset,port_servo_z_offset));
	// no rotation to start
	port_tf.setRotation(tf::Quaternion(0,0,0));

	starboard_tf.setOrigin(tf::Vector3(starboard_servo_x_offset,starboard_servo_y_offset,starboard_servo_z_offset));
	// no rotation to start
	starboard_tf.setRotation(tf::Quaternion(0,0,0));

	// set the update rate (in hz) for which the pd controller runs
	update_rate=20;

	// init the id variables for what their corresponding servo's are on the dynamixel bus
	port_servo_id=0x03;
	starboard_servo_id=0x02;

	// get the fully qualified namespace for the dynamixel node
	string dynamixel_fqns=ros::names::resolve(dynamixel_namespace,true);

	// Initialize the subscribers and their callback functions (which will just update things to what their current values are).
	odom_subscriber=n.subscribe(odom_topic,1000, &ZDrive::currentOdomCallBack,this);
	dynamixel_status_subscriber=n.subscribe(dynamixel_fqns+"/"+dynamixel_status_topic, 1000, &ZDrive::dynamixelStatusCallBack, this);
	thruster_status_subscriber=n.subscribe("thruster_status",1000,&ZDrive::thrusterStatusCallBack ,this);

	//Advertise the various publisher(s)
	dynamixel_config_full_pub=n.advertise<dynamixel_servo::DynamixelConfigParam>(dynamixel_fqns+"/"+"dynamixel_config_full",1000);
	dynamixel_config_position_pub=n.advertise<dynamixel_servo::DynamixelConfigPosition>(dynamixel_fqns+"/"+"dynamixel_config_position",1000);
	thruster_config_pub=n.advertise<motor_control::thrusterStatus>("thruster_config",1000);
	z_drive_dbg_pub=n.advertise<z_drive::ZDriveDbg>("z_drive_dbg_msg",1000);
	joint_pub=n.advertise<sensor_msgs::JointState>("z_drive_joints",10);

}
void ZDrive::dynamixelStatusCallBack(const dynamixel_servo::DynamixelStatusParam& dynamixel_status_msg)
{
	if(dynamixel_status_msg.id==port_servo_id)
	{
		// note: present_position is a float32. Hence the cast is explicitly shown. The dynamixel node is optimized for speed and memory to be near real-time, hence the float32.
		ZDrive::current_port_servo_angle=(double)dynamixel_status_msg.present_position-port_servo_angle_offset;

	}
	else if(dynamixel_status_msg.id==starboard_servo_id)
	{
		ZDrive::current_starboard_servo_angle=(double)dynamixel_status_msg.present_position-starboard_servo_angle_offset;
	}
	return;
}

void ZDrive::thrusterStatusCallBack(const motor_control::thrusterStatus& thruster_status_msg)
{
	if(thruster_status_msg.id==port_servo_id)
	{
		ZDrive::current_port_thruster_force=thruster_status_msg.thrust;
	}
	else if(thruster_status_msg.id==starboard_servo_id)
	{
		ZDrive::current_starboard_thruster_force=thruster_status_msg.thrust;
	}
	return;
}

void ZDrive::currentOdomCallBack(const nav_msgs::Odometry& odom_msg)
{
	// TWIST part of an odom message is relative to the BODY! frame!
	// POSE part of the odom message is relative to the WORLD frame!
	// odom_msg contains all the information about how our robot (base_link) currently relates to the world.
	ZDrive::odom_current=odom_msg;
	ZDrive::x_current=odom_msg.pose.pose.position.x;
	ZDrive::y_current=odom_msg.pose.pose.position.y;
	// extract the relevant information based on Quaternions
	tf::Quaternion q_temp;
	tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q_temp);
	tf::Matrix3x3(q_temp).getRPY(ZDrive::roll_current, ZDrive::pitch_current, ZDrive::yaw_current);
	// the velocity(s) needs to be relative to the world frame
	// first we will get the linear velocities relative to the world frame
	tf::Vector3 linear_body_velocity;
	tf::vector3MsgToTF(odom_msg.twist.twist.linear, linear_body_velocity);
	tf::Vector3 linear_world_velocity = tf::Matrix3x3(q_temp) * linear_body_velocity;
	ZDrive::x_velocity_current=linear_world_velocity.x();
	ZDrive::y_velocity_current=linear_world_velocity.y();
	// now the angular velocities relative to the world frame
	tf::Vector3 angular_body_velocity;
	tf::vector3MsgToTF(odom_msg.twist.twist.angular, angular_body_velocity);
	tf::Vector3 angular_world_velocity =tf::Matrix3x3(q_temp) * angular_body_velocity;
	ZDrive::roll_velocity_current=angular_world_velocity.x();
	ZDrive::pitch_velocity_current=angular_world_velocity.y();
	ZDrive::yaw_velocity_current=angular_world_velocity.z();

	return;
}

void ZDrive::desiredOdomCallBack(const  uf_common::PoseTwist desired_pose_twist)
{
	// TWIST part of an odom message is relative to the BODY! frame!
	// POSE part of the odom message is relative to the WORLD frame!
	// odom_msg contains all the information about how our robot (base_link) currently relates to the world.
	//ZDrive::odom_desired=odom_msg;
	ZDrive::x_desired=desired_pose_twist.pose.position.x;
	ZDrive::y_desired=desired_pose_twist.pose.position.y;
	// extract the relevant information based on Quaternions
	tf::Quaternion q_temp;
	tf::quaternionMsgToTF(desired_pose_twist.pose.orientation, q_temp);
	tf::Matrix3x3(q_temp).getRPY(ZDrive::roll_desired, ZDrive::pitch_desired, ZDrive::yaw_desired);
	// the velocity(s) needs to be relative to the world frame
	// first we will get the linear velocities relative to the world frame
	tf::Vector3 linear_body_velocity;
	tf::vector3MsgToTF(desired_pose_twist.twist.linear, linear_body_velocity);
	tf::Vector3 linear_world_velocity = tf::Matrix3x3(q_temp) * linear_body_velocity;
	ZDrive::x_velocity_desired=linear_world_velocity.x();
	ZDrive::y_velocity_desired=linear_world_velocity.y();
	// now the angular velocities relative to the world frame
	tf::Vector3 angular_body_velocity;
	tf::vector3MsgToTF(desired_pose_twist.twist.angular, angular_body_velocity);
	tf::Vector3 angular_world_velocity =tf::Matrix3x3(q_temp) * angular_body_velocity;
	ZDrive::roll_velocity_desired=angular_world_velocity.x();
	ZDrive::pitch_velocity_desired=angular_world_velocity.y();
	ZDrive::yaw_velocity_desired=angular_world_velocity.z();
	return;
}

void ZDrive::publishDbgMsg(double user_defined_1=0, double user_defined_2=0, double user_defined_3=0, double user_defined_4=0, double user_defined_5=0)
{
	dbg_msg.cost_value=calcCost(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	dbg_msg.current_port_servo_angle=ZDrive::current_port_servo_angle;
	dbg_msg.current_port_thruster_force=ZDrive::current_port_thruster_force;
	dbg_msg.current_starboard_servo_angle=ZDrive::current_starboard_servo_angle;
	dbg_msg.current_starboard_thruster_force=ZDrive::current_starboard_thruster_force;
	dbg_msg.estimated_port_servo_angle=ZDrive::estimated_port_servo_angle;
	dbg_msg.estimated_port_thruster_force=ZDrive::estimated_port_thruster_force;
	dbg_msg.estimated_starboard_servo_angle=ZDrive::estimated_starboard_servo_angle;
	dbg_msg.estimated_starboard_thruster_force=ZDrive::estimated_starboard_thruster_force;
	dbg_msg.required_force_x=ZDrive::force_bow_required;
	dbg_msg.required_force_y=ZDrive::force_port_required;
	dbg_msg.required_moment_z=ZDrive::moment_z_required;
	dbg_msg.resultant_force_x=resultantForceX(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	dbg_msg.resultant_force_y=resultantForceY(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	dbg_msg.resultant_moment_z=resultantMomentZ(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	dbg_msg.step=ZDrive::step;
	dbg_msg.x_current=ZDrive::x_current;
	dbg_msg.x_desired=ZDrive::x_desired;
	dbg_msg.x_velocity_current=ZDrive::x_velocity_current;
	dbg_msg.x_velocity_desired=ZDrive::x_velocity_desired;
	dbg_msg.y_current=ZDrive::y_current;
	dbg_msg.y_desired=ZDrive::y_desired;
	dbg_msg.y_velocity_current=ZDrive::y_velocity_current;
	dbg_msg.y_velocity_desired=ZDrive::y_velocity_desired;
	dbg_msg.yaw_current=ZDrive::yaw_current;
	dbg_msg.yaw_desired=ZDrive::yaw_desired;
	dbg_msg.user_defined_1=user_defined_1;
	dbg_msg.user_defined_2=user_defined_2;
	dbg_msg.user_defined_3=user_defined_3;
	dbg_msg.user_defined_4=user_defined_4;
	dbg_msg.user_defined_5=user_defined_5;
	z_drive_dbg_pub.publish(dbg_msg);
}

void ZDrive::run()
{

	// odom_current.pose.pose.position for points x,y,z in the world frame
	// odom_current.pose.pose.orientation for Quaternion which need to be turned into rpy for rotation about an axis in the world frame
	// odom_current.twist.twist.linear for vx,vy,vz in the base frame
	// odom_current.twist.twist.angular for angular velocity in the base frame

	// we have to wait for the nodes to connect before we post the inital config message
	while(dynamixel_config_full_pub.getNumSubscribers()==0)
	{
		//ROS_INFO("Waiting for ZDrive node and Dynamixel node to connect");
	}

	// fill in an init message
	dynamixel_servo::DynamixelConfigParam dynamixel_init_config_msg;
	dynamixel_init_config_msg.goal_position=(float)(M_PI);
	dynamixel_init_config_msg.moving_speed=0x0084; // 15rpm
	dynamixel_init_config_msg.torque_limit=0x03FF;
	dynamixel_init_config_msg.goal_acceleration=(8.5826772*M_PI/180)*0xFD;

	// config the port servo initaly first
	dynamixel_init_config_msg.id=ZDrive::port_servo_id;
	dynamixel_config_full_pub.publish(dynamixel_init_config_msg);
	// config the starboard servo initaly first
	dynamixel_init_config_msg.id=ZDrive::starboard_servo_id;
	dynamixel_config_full_pub.publish(dynamixel_init_config_msg);

	actionlib::SimpleActionServer<uf_common::MoveToAction> actionserver(n, "moveto", false);

	dynamixel_servo::DynamixelConfigPosition dynamixel_position_msg;
	motor_control::thrusterConfig thruster_config_msg;
	ros::Rate loop_rate(update_rate);
	while(ros::ok())
	{
		// First process callbacks to get the most recent odom and desired position information
		ros::spinOnce();

		if(actionserver.isNewGoalAvailable())
		{
			boost::shared_ptr<const uf_common::MoveToGoal> goal = actionserver.acceptNewGoal();
			desiredOdomCallBack(goal->posetwist);
		}

		// now run the control algorithm(s) to extrapolate the information needed for the z_drive
		minimizeCostFunction();

		// publish a dynamixel_config_position message for the port servo based on the controller's (possibly) new estimates.
		// Note: the dynamixel_server is written such that it quickly disregards messages where there is no change required in the servo's position.
		dynamixel_position_msg.id=ZDrive::port_servo_id;
		dynamixel_position_msg.goal_position=(float)(ZDrive::estimated_port_servo_angle+ZDrive::port_servo_angle_offset);
		ROS_INFO("Port servo position in deg: %f\t\t Thrust in newtons: %f", dynamixel_position_msg.goal_position*180/M_PI,ZDrive::estimated_port_thruster_force);
		dynamixel_config_position_pub.publish(dynamixel_position_msg);
		thruster_config_msg.id=ZDrive::port_servo_id;
		thruster_config_msg.thrust=ZDrive::estimated_port_thruster_force;
		thruster_config_pub.publish(thruster_config_msg);

		// publish a dynamixel_config_position message for the starboard servo based on the controller's (possibly) new estimates.
		dynamixel_position_msg.id=ZDrive::starboard_servo_id;
		dynamixel_position_msg.goal_position=(float)(ZDrive::estimated_starboard_servo_angle+ZDrive::starboard_servo_angle_offset);
		ROS_INFO("Starboard servo position in deg: %f\t\t Thrust in newtons: %f", dynamixel_position_msg.goal_position*180/M_PI, ZDrive::estimated_starboard_thruster_force);
		dynamixel_config_position_pub.publish(dynamixel_position_msg);
		thruster_config_msg.id=ZDrive::starboard_servo_id;
		thruster_config_msg.thrust=ZDrive::estimated_starboard_thruster_force;
		thruster_config_pub.publish(thruster_config_msg);

		// update the joint_state things
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(2);
		joint_state.position.resize(2);
		joint_state.name[0] ="port_servo";
		joint_state.position[0]=1.0;
		joint_state.name[1] ="starboard_servo";
		joint_state.position[1]=1.0;
		// publish the joint state
		joint_pub.publish(joint_state);

		// brodcast the various transforms. Note: base_link is the rotational center of the robot
		tf_brodcaster.sendTransform(tf::StampedTransform(z_drive_root_tf, ros::Time::now(), "base_link", ros::this_node::getName()));
		tf_brodcaster.sendTransform(tf::StampedTransform(port_tf, ros::Time::now(), ros::this_node::getName(), "port"));
		tf_brodcaster.sendTransform(tf::StampedTransform(starboard_tf, ros::Time::now(), ros::this_node::getName(), "starboard"));
		loop_rate.sleep();
	}
}

// Note: in an ideal system the resultant force/moment would equal the required force/moment, but our system isn't ideal; thus we must calculate both.
//---------------------- this is the force/moment that can be provided to the system for a given configuration of the z_drive ---------------
double ZDrive::resultantForceX(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force)
{
	return port_thruster_force*cos(port_servo_angle) + starboard_thruster_force*cos(starboard_servo_angle);
}
double ZDrive::resultantForceY(double port_servo_angle, double port_thruster_force, double starboard_servo_angle, double starboard_thruster_force)
{
	return port_thruster_force*sin(port_servo_angle) + starboard_thruster_force*sin(starboard_servo_angle);
}
double ZDrive::resultantMomentZ(double port_servo_angle, double port_thruster_force, double starboard_servo_angle, double starboard_thruster_force)
{
	return port_servo_x_offset*port_thruster_force*sin(port_servo_angle) - port_servo_y_offset*port_thruster_force*cos(port_servo_angle) + starboard_servo_x_offset*starboard_thruster_force*sin(starboard_servo_angle) - starboard_servo_y_offset*starboard_thruster_force*cos(starboard_servo_angle);
}

//---------------------- this is what we want the system to do, but it may not be able to. ----------------
double ZDrive::requiredForceX(double x_current, double x_desired, double x_velocity_current, double x_velocity_desired)
{
	return p_gain_x*(x_current-x_desired)+d_gain_x*(x_velocity_current-x_velocity_desired);
}
double ZDrive::requiredForceY(double y_current, double y_desired, double y_velocity_current, double y_velocity_desired)
{
	return p_gain_y*(y_current-y_desired)+d_gain_y*(y_velocity_current-y_velocity_desired);
}
double ZDrive::requiredMomentZ(double boat_angle_current, double boat_angle_desired, double boat_angular_velocity_current, double boat_angular_velocity_desired)
{
	// this is the angle (in radian) of the boat relative to the world coordinate frame
	return p_gain_theta_boat*(boat_angle_current-boat_angle_desired)+d_gain_theta_boat*(boat_angular_velocity_current-boat_angular_velocity_desired);
}

//---------------------- these are the partial derivatives related to a given configuration of the z_drive system (with respect to the costValue/"cost function")  --------------------------
double ZDrive::dCost_dPortServoAngle(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force)
{
	// remember according to ros rep 103: x=bow=forward, y=port=left
	// We will first assume that we are pointing north at 0,0- we will transform it relative to the actual odom information shortly.
	return gain_deviation_equilibrum_servo_angle/2 - (gain_deviation_changeof_servo_angle*(2*ZDrive::current_port_servo_angle - 2*port_servo_angle))/2 - gain_error_moment_z*(port_thruster_force*port_servo_x_offset*cos(port_servo_angle) + port_thruster_force*port_servo_y_offset*sin(port_servo_angle))*(ZDrive::moment_z_required + port_thruster_force*port_servo_y_offset*cos(port_servo_angle) + starboard_thruster_force*starboard_servo_y_offset*cos(starboard_servo_angle) - port_thruster_force*port_servo_x_offset*sin(port_servo_angle) - starboard_thruster_force*starboard_servo_x_offset*sin(starboard_servo_angle)) - gain_error_force_x*port_thruster_force*sin(port_servo_angle)*(port_thruster_force*cos(port_servo_angle) - ZDrive::force_bow_required + starboard_thruster_force*cos(starboard_servo_angle)) + gain_error_force_y*port_thruster_force*cos(port_servo_angle)*(port_thruster_force*sin(port_servo_angle) - ZDrive::force_port_required + starboard_thruster_force*sin(starboard_servo_angle));
}
double ZDrive::dCost_dPortThrusterForce(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force)
{
	// remember according to ros rep 103: x=bow=forward, y=port=left
	// We will first assume that we are pointing north at 0,0- we will transform it relative to the actual odom information shortly.
	return gain_thrusters_force*port_thruster_force + gain_error_moment_z*(port_servo_y_offset*cos(port_servo_angle) - port_servo_x_offset*sin(port_servo_angle))*(ZDrive::moment_z_required + port_thruster_force*port_servo_y_offset*cos(port_servo_angle) + starboard_thruster_force*starboard_servo_y_offset*cos(starboard_servo_angle) - port_thruster_force*port_servo_x_offset*sin(port_servo_angle) - starboard_thruster_force*starboard_servo_x_offset*sin(starboard_servo_angle)) + gain_error_force_x*cos(port_servo_angle)*(port_thruster_force*cos(port_servo_angle) - ZDrive::force_bow_required + starboard_thruster_force*cos(starboard_servo_angle)) + gain_error_force_y*sin(port_servo_angle)*(port_thruster_force*sin(port_servo_angle) - ZDrive::force_port_required + starboard_thruster_force*sin(starboard_servo_angle));
}
double ZDrive::dCost_dStarboardServoAngle(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force)
{
	// remember according to ros rep 103: x=bow=forward, y=port=left
	// We will first assume that we are pointing north at 0,0- we will transform it relative to the actual odom information shortly.
	return gain_deviation_equilibrum_servo_angle/2 - (gain_deviation_changeof_servo_angle*(2*ZDrive::current_starboard_servo_angle - 2*starboard_servo_angle))/2 - gain_error_moment_z*(starboard_thruster_force*starboard_servo_x_offset*cos(starboard_servo_angle) + starboard_thruster_force*starboard_servo_y_offset*sin(starboard_servo_angle))*(ZDrive::moment_z_required + port_thruster_force*port_servo_y_offset*cos(port_servo_angle) + starboard_thruster_force*starboard_servo_y_offset*cos(starboard_servo_angle) - port_thruster_force*port_servo_x_offset*sin(port_servo_angle) - starboard_thruster_force*starboard_servo_x_offset*sin(starboard_servo_angle)) - gain_error_force_x*starboard_thruster_force*sin(starboard_servo_angle)*(port_thruster_force*cos(port_servo_angle) - ZDrive::force_bow_required + starboard_thruster_force*cos(starboard_servo_angle)) + gain_error_force_y*starboard_thruster_force*cos(starboard_servo_angle)*(port_thruster_force*sin(port_servo_angle) - ZDrive::force_port_required + starboard_thruster_force*sin(starboard_servo_angle));
}
double ZDrive::dCost_dStarboardThrusterForce(double port_servo_angle,double port_thruster_force,double starboard_servo_angle,double starboard_thruster_force)
{
	// remember according to ros rep 103: x=bow=forward, y=port=left
	// We will first assume that we are pointing north at 0,0- we will transform it relative to the actual odom information shortly.
	return gain_thrusters_force*starboard_thruster_force + gain_error_moment_z*(starboard_servo_y_offset*cos(starboard_servo_angle) - starboard_servo_x_offset*sin(starboard_servo_angle))*(ZDrive::moment_z_required + port_thruster_force*port_servo_y_offset*cos(port_servo_angle) + starboard_thruster_force*starboard_servo_y_offset*cos(starboard_servo_angle) - port_thruster_force*port_servo_x_offset*sin(port_servo_angle) - starboard_thruster_force*starboard_servo_x_offset*sin(starboard_servo_angle)) + gain_error_force_x*cos(starboard_servo_angle)*(port_thruster_force*cos(port_servo_angle) - ZDrive::force_bow_required + starboard_thruster_force*cos(starboard_servo_angle)) + gain_error_force_y*sin(starboard_servo_angle)*(port_thruster_force*sin(port_servo_angle) - ZDrive::force_port_required + starboard_thruster_force*sin(starboard_servo_angle));
}

//---------------------- calcCost function calculates the given "cost" that we have assigned to the system for a given configuration of the z_drive. --------------
double ZDrive::calcCost(double port_servo_angle, double port_thruster_force, double starboard_servo_angle, double starboard_thruster_force)
{
	// Note: Doubles are stored in ieee754 format (thus having a sign, exponent, and fraction section), so binary division and multiplication can't be applied here- for optimization.
	return
			gain_error_force_x/2 * pow((resultantForceX(port_servo_angle, port_thruster_force, starboard_servo_angle, starboard_thruster_force)-ZDrive::force_bow_required),2) +
			gain_error_force_y/2 * pow((resultantForceY(port_servo_angle, port_thruster_force, starboard_servo_angle, starboard_thruster_force)-ZDrive::force_port_required),2) +
			gain_error_moment_z/2 * pow((resultantMomentZ(port_servo_angle, port_thruster_force, starboard_servo_angle, starboard_thruster_force)-ZDrive::moment_z_required),2) +
			gain_thrusters_force/2 * pow(port_thruster_force,2) +
			gain_thrusters_force/2 * pow(starboard_thruster_force,2) +
			gain_deviation_equilibrum_servo_angle/2 * port_servo_angle +
			gain_deviation_equilibrum_servo_angle/2 * starboard_servo_angle +
			gain_deviation_changeof_servo_angle/2 * pow((port_servo_angle - ZDrive::current_port_servo_angle),2) +
			gain_deviation_changeof_servo_angle/2 * pow((starboard_servo_angle - ZDrive::current_starboard_servo_angle),2);

}

void ZDrive::guessInitalValues()
{
	// this is for future use to allow someone to create a good initial guess on what the first servo angle should be
	ZDrive::estimated_port_servo_angle=ZDrive::current_port_servo_angle;
	ZDrive::estimated_port_thruster_force=ZDrive::current_port_thruster_force;
	ZDrive::estimated_starboard_servo_angle=ZDrive::current_starboard_servo_angle;
	ZDrive::estimated_starboard_thruster_force=ZDrive::current_starboard_thruster_force;
}

void ZDrive::checkEstimatedValuesAgainsBounds()
{
	// if any bound exceeds the accepted limits, reset it to the corresponding limit
	if(ZDrive::estimated_port_servo_angle<ZDrive::port_servo_angle_clock_wise_limit)
	{
		ZDrive::estimated_port_servo_angle=ZDrive::port_servo_angle_clock_wise_limit;
	}
	else if(ZDrive::estimated_port_servo_angle>ZDrive::port_servo_angle_counter_clock_wise_limit)
	{
		ZDrive::estimated_port_servo_angle=ZDrive::port_servo_angle_counter_clock_wise_limit;
	}
	if(ZDrive::estimated_port_thruster_force<ZDrive::port_thruster_reverse_limit)
	{
		ZDrive::estimated_port_thruster_force=ZDrive::port_thruster_reverse_limit;
	}
	else if(ZDrive::estimated_port_thruster_force>ZDrive::port_thruster_foward_limit)
	{
		ZDrive::estimated_port_thruster_force=ZDrive::port_thruster_foward_limit;
	}

	if (ZDrive::estimated_starboard_servo_angle < ZDrive::starboard_servo_angle_clock_wise_limit)
	{
		ZDrive::estimated_starboard_servo_angle = ZDrive::starboard_servo_angle_clock_wise_limit;
	}
	else if (ZDrive::estimated_starboard_servo_angle > ZDrive::starboard_servo_angle_counter_clock_wise_limit)
	{
		ZDrive::estimated_starboard_servo_angle = ZDrive::starboard_servo_angle_counter_clock_wise_limit;
	}
	if (ZDrive::estimated_starboard_thruster_force < ZDrive::starboard_thruster_reverse_limit)
	{
		ZDrive::estimated_starboard_thruster_force = ZDrive::starboard_thruster_reverse_limit;
	}
	else if (ZDrive::estimated_starboard_thruster_force > ZDrive::starboard_thruster_foward_limit)
	{
		ZDrive::estimated_starboard_thruster_force = ZDrive::starboard_thruster_foward_limit;
	}
}

// After this function runs the final solution that is estimated is closer and closer to the optimal solution
// Note: The initial call to this function assumes that a guess has been made for the respective trust and angle measurements; this function was designed to converge towards the optimal solution with each iteration.
void ZDrive::calcAngleAndThrustEstimate()
{

	double partial_derivative_port_angle_result=dCost_dPortServoAngle(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	double partial_derivative_port_thrust_result=dCost_dPortThrusterForce(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	double partial_derivative_starboard_angle_result=dCost_dStarboardServoAngle(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	double partial_derivative_starboard_thrust_result=dCost_dStarboardThrusterForce(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);

	// init the cost variables. so innately best_cost_value=the current config of the system, and new_cost_value will be another hypothesized config that might be more optimal.
	// the cost cost_values/cost_function can be very oscillatory, so the goal is to get the best results even if it is oscillating, hence the following variables
	double best_cost_value=calcCost(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
	double new_cost_value=DBL_MAX; // initalize it to the maximum allowed value since initaly it isnt better than anything
	double best_step=0;
	double best_port_servo_angle=ZDrive::estimated_port_servo_angle;
	double best_port_thruster_force=ZDrive::estimated_port_thruster_force;
	double best_starboard_servo_angle=ZDrive::estimated_starboard_servo_angle;
	double best_starboard_thruster_force=ZDrive::estimated_starboard_thruster_force;
	int cost_count=0;// this is essentialy how many times have we seen results that are worse than previously calculated values


	// this is in a for loop to ensure there isn't an endless loop while finding the best value for the step size- that we will return.
	// so the goal is to run the for loop until we find the first cost value that isn't decreasing in a given buffer range
	for (double i=0; i<=.01; i+=ZDrive::step_size )
	{
		ZDrive::step=i;
		ZDrive::estimated_port_servo_angle=ZDrive::estimated_port_servo_angle-(ZDrive::step*partial_derivative_port_angle_result);
		ZDrive::estimated_port_thruster_force=ZDrive::estimated_port_thruster_force-(ZDrive::step*partial_derivative_port_thrust_result);
		ZDrive::estimated_starboard_servo_angle=ZDrive::estimated_starboard_servo_angle-(ZDrive::step*partial_derivative_starboard_angle_result);
		ZDrive::estimated_starboard_thruster_force=ZDrive::estimated_starboard_thruster_force-(ZDrive::step*partial_derivative_starboard_thrust_result);
		new_cost_value=calcCost(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);

		if(new_cost_value<=best_cost_value)
		{
			best_cost_value=new_cost_value;
			best_step=ZDrive::step;
			best_port_servo_angle=ZDrive::estimated_port_servo_angle;
			best_port_thruster_force=ZDrive::estimated_port_thruster_force;
			best_starboard_servo_angle=ZDrive::estimated_starboard_servo_angle;
			best_starboard_thruster_force=ZDrive::estimated_starboard_thruster_force;
			cost_count=0;
		}
		else
		{
			cost_count++;
		}

		if(cost_count>=ZDrive::cost_count_max)
		{
			cost_count=0; //reinitalize it for the next loop to just use it
			break; //exit the for loop
		}
	}

	// Again we will bound the loop so it isn't an infinate loop
	// Now work backwards from where we got the best_cost_value by a VERY SMALL amount (ZDrive::step_multiplier times smaller) to try to find the min value thats less.
	for (double i=best_step; i>=0; i-=(ZDrive::step_size/ZDrive::step_multiplier) )
	{
		ZDrive::step=i;
		ZDrive::estimated_port_servo_angle=ZDrive::estimated_port_servo_angle-(ZDrive::step*partial_derivative_port_angle_result);
		ZDrive::estimated_port_thruster_force=ZDrive::estimated_port_thruster_force-(ZDrive::step*partial_derivative_port_thrust_result);
		ZDrive::estimated_starboard_servo_angle=ZDrive::estimated_starboard_servo_angle-(ZDrive::step*partial_derivative_starboard_angle_result);
		ZDrive::estimated_starboard_thruster_force=ZDrive::estimated_starboard_thruster_force-(ZDrive::step*partial_derivative_starboard_thrust_result);
		new_cost_value=calcCost(ZDrive::estimated_port_servo_angle, ZDrive::estimated_port_thruster_force, ZDrive::estimated_starboard_servo_angle, ZDrive::estimated_starboard_thruster_force);
		if(new_cost_value<=best_cost_value)
		{
			best_cost_value=new_cost_value;
			best_step=ZDrive::step;
			best_port_servo_angle=ZDrive::estimated_port_servo_angle;
			best_port_thruster_force=ZDrive::estimated_port_thruster_force;
			best_starboard_servo_angle=ZDrive::estimated_starboard_servo_angle;
			best_starboard_thruster_force=ZDrive::estimated_starboard_thruster_force;
			cost_count=0;
		}
		else
		{
			cost_count++;
		}

		if(cost_count>=ZDrive::cost_count_max)
		{
			cost_count=0; //reinitalize it for the next loop to just use it
			break; //exit the for loop
		}
	}

	// Now again set it back to what that best solution was that we found
	ZDrive::step=best_step;
	ZDrive::estimated_port_servo_angle=best_port_servo_angle;
	ZDrive::estimated_port_thruster_force=best_port_thruster_force;
	ZDrive::estimated_starboard_servo_angle=best_starboard_servo_angle;
	ZDrive::estimated_starboard_thruster_force=best_starboard_thruster_force;

	// lastly verify that the estimates we calculated are within bounds. Note: checkEstimatedValuesAgainsBounds will fix to their corresponding limit if they are not.
	checkEstimatedValuesAgainsBounds();
}


void ZDrive::minimizeCostFunction()
{

	// we must first get the required forces before we can create an initial guess for the solution to minimize the cost function. (remember we may not be able to archive this in the end, hence the resultant force)
	ZDrive::force_bow_required=requiredForceX(ZDrive::x_current, ZDrive::x_desired, ZDrive::x_velocity_current, ZDrive::x_velocity_desired);
	ZDrive::force_port_required=requiredForceY(ZDrive::y_current, ZDrive::y_desired, ZDrive::y_velocity_current, ZDrive::y_velocity_desired);
	ZDrive::moment_z_required=requiredMomentZ(ZDrive::yaw_current, ZDrive::yaw_desired, ZDrive::yaw_velocity_current, ZDrive::yaw_velocity_desired);
	// Intelligently guess and initialize values for the respective thrusts and angles. This must be done before we can calculate estimates.
	guessInitalValues();

	for(int i=0;i<20;i++)
	{
		// each time we calculate an estimate again we get closer and closer to the optimal solution for the desired position
		calcAngleAndThrustEstimate();
		publishDbgMsg((double)i);
	}

}

#endif