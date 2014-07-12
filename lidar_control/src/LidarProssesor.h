#ifndef LIDAR_PROSSESOR_H_
#define LIDAR_PROSSESOR_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include "tf/transform_listener.h"
#include <algorithm>		//for remove_if
#include <cmath>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h";
#include "geometry_msgs/Vector3Stamped.h"

#define PI 3.14159265358979323846

struct f_2dpt 
{
	float x, y;
};

struct simple2dVector
{
public:
	float angle_, mag_;
	
	simple2dVector(float mag, float angle):
		mag_(mag), angle_(angle)
	{}
	
	simple2dVector():
			mag_(0.0f), angle_(0.0f)
		{}
	
	float GetX() const {return mag_ * cos(angle_);}
	float GetY() const {return mag_ * sin(angle_);}
	
	simple2dVector& operator+=(const simple2dVector& vec)
	{		
		float x = this->GetX() + vec.GetX();
		float y = this->GetY() + vec.GetY();
		
		this->mag_ = sqrt(pow(x, 2) + pow(y, 2));
		this->angle_ = atan(y/x);			//Returns angle between -PI/2 to PI/2
		
		return *this;
	}

};

using namespace std;

class LidarProssesor
{
	/*
	 * Private vars
	 */
	//Laser scan subscriber
	ros::Subscriber scan_sub;
	
	//Output publisher
	ros::Publisher out_pub;
	
	//Debug publisher
	ros::Publisher debug_pub;
	ros::Publisher debug_pub2;

	//Transform
	tf::TransformListener lidar_tf;

	//State
	bool prossesing_, first_;

	//Current pts
	vector<float> ranges_;

	//Angles
	float min_angle_, max_angle_, angle_inc_;
	//Ranges
	//static float min_range_, max_range_;
	float min_range_, max_range_;

	//Configuration params		
	//	max_point_range_weight
	//		This value controls how much weight is given to points at the minimum range
	//		The scalling is minimum range to maximum range linearly coresponds to max_point_range weight to 0
	double max_point_range_weight_;
	//	angulat_point_weight
	//		This value controls how quickly the weighting reduces from the points nominal angle
	//		The nominal weight is reduced by angular_decay_ per angle_inc_
	//double angular_decay_;
	// Potential field
	//		1081 is the size of the range message from laser
	//int potential_field[1081];
	
	//maping params
	float range_to_weight_slope_;
	float range_to_weight_y_intercept_ ;

	/*
	 * Private functions
	 */
private:
	void GetNewScan(const sensor_msgs::LaserScan& scan);

	void FindBestVector();
	
	//static bool isOutOfRange(const float& range);
	bool isOutOfRange(const float& range);
	
	f_2dpt GeneratePoint(float magnitude, float angle);
	
	vector<f_2dpt> GeneratePoints();
	
	float RangeToWeight(float range);
	
	/*
	 * 	Public functions
	 */
public:
	void Run();
	
	void Setup();
	
	LidarProssesor();

};

//Static var initilization
//float LidarProssesor::min_range_ = 0;
//float LidarProssesor::max_range_ = 0;

LidarProssesor::LidarProssesor():
		prossesing_(false)
	{}

void LidarProssesor::GetNewScan(const sensor_msgs::LaserScan& scan)
{
	if(!prossesing_)
	{
		if(first_ == true)	//If its the first msg get the params from the laser
		{
			first_ = false;
			min_angle_ = scan.angle_min;
			max_angle_ = scan.angle_max;
			angle_inc_ = scan.angle_increment;
			min_range_ = scan.range_min;
			max_range_ = scan.range_max;
			//range_to_weight_slope_ = -max_point_range_weight_ / (max_range_ - min_range_);		//Linear
			//range_to_weight_y_intercept_ = -range_to_weight_slope_ * max_range_;
			range_to_weight_slope_ = -(PI/2)/(max_range_ - min_range_);								//Tangental
			range_to_weight_y_intercept_ = -range_to_weight_slope_ * max_range_;
			ROS_INFO("min angle: %f\tmax angle %f\tangle inc. %f\tmin_range %f\tmax range %f", min_angle_, 
					max_angle_, angle_inc_, min_range_, max_range_);
		}
		//ROS_INFO("Scan vector");
		ranges_ = scan.ranges;
		//ROS_INFO("After scan vector");
		prossesing_ = true;
	}
}

void LidarProssesor::Run()
{
	ros::Rate loop_freq(10);
	
	while(ros::ok())
	{
		if(prossesing_)
		{
			FindBestVector();
			//ROS_INFO("After Find best vector");
			prossesing_ = false;
		}
		
		loop_freq.sleep();
		ros::spinOnce();
	}
}

void LidarProssesor::Setup()
{
	/*
	 * Initilize ros
	 */
	ros::NodeHandle n;

	//Initilize subscribers
	scan_sub = n.subscribe("/lidar/scan", 1, &LidarProssesor::GetNewScan, this);		//Queue size selected so that we will drop waiting messages

	//initilize publishers
	out_pub = n.advertise<geometry_msgs::PoseStamped>("/lidar/minimum_pts", 10);
	debug_pub = n.advertise<geometry_msgs::PoseArray>("lidar/debug",10);
	debug_pub2 = n.advertise<geometry_msgs::PoseArray>("lidar/debug2",10);
	
	//Get params
	n.param<double>("/lidar/processing/max_point_range_weight", max_point_range_weight_, 100.0);
	//n.param<double>("/lidar/processing/angular_point_weight", angular_decay_, 50);
}

f_2dpt LidarProssesor::GeneratePoint(float magnitude, float angle)
{
	f_2dpt pt;
	pt.x = magnitude * cos(angle);
	pt.y = magnitude * sin(angle);
	return pt;
}

bool LidarProssesor::isOutOfRange(const float& range)
{
	return (range <= min_range_ || range >= max_range_ - 10);
}

/*
 *  Ignores Z values
 */
vector<f_2dpt> LidarProssesor::GeneratePoints()
{
	float current_angle = min_angle_;
		vector <f_2dpt> pts;
		//Remove outliers
		//ranges_.erase(remove_if(ranges_.begin(), ranges_.end(), LidarProssesor::isOutOfRange));		Won't work because I need to know the angles
		for(int iii = 0; iii < ranges_.size(); ++iii)
		{
			if(!isOutOfRange(ranges_[iii]))			//If in range make a point
			{
				//Generate points
				pts.push_back(GeneratePoint(ranges_[iii], current_angle));
			}
			else
			{
				//ummm... do nothing???
			}
			
			//Update angle
			current_angle += angle_inc_;
		}
}

float LidarProssesor::RangeToWeight(float range)
{
	//Linear maping of range to weight
	//return range_to_weight_slope_ * range + range_to_weight_y_intercept_;
	//Tangental mapping of weight, gives infinite at minimum and 0 at maximum
	return tan(range_to_weight_slope_ * range + range_to_weight_y_intercept_);
	
}

void LidarProssesor::FindBestVector()
{
	/*
	 * Outdated algorithim!!!!!
	 *
	//Generate a psedo-potential feild in the 1 dimensional space (angular radians)
	float current_angle = min_angle_;
	float* potential_field = new float[ranges_.size()]/*{0}/*Not sure if this works~~~~~~;				//This should initilize array to 0
	
	//init
	int center = ranges_.size() / 2;
	for(int iii = 0; iii < ranges_.size(); ++iii)
	{
		//potential_field[iii] = abs(center - iii) * 0.1 * max_point_range_weight_;
		potential_field[iii] = 0;
	}
	
	//Remove outliers
	//ranges_.erase(remove_if(ranges_.begin(), ranges_.end(), LidarProssesor::isOutOfRange));		Won't work because I need to know the angles
	int num_in_range_pts = 0;
	for(int iii = 0; iii < ranges_.size(); ++iii)
	{		
		if(!isOutOfRange(ranges_[iii]))			//If in range make add to the vector fields
		{
			++num_in_range_pts;
			
			float nominal_weight = RangeToWeight(ranges_[iii]);
			
			//ROS_INFO("Nominal weight is %f", nominal_weight);
			
			//Generate weights
			for(int jjj = 0; jjj < ranges_.size(); ++jjj)
			{
				float new_weight = nominal_weight - (abs(jjj - iii) *  angular_decay_);
				if(new_weight <= 0)
				{
					//ROS_INFO("Add nothing!");
					continue;
				}
				//ROS_INFO("New_weight %f", new_weight);
				potential_field[jjj] += new_weight;
			}
		}
		
		//Update angle
		current_angle += angle_inc_;
	}

	//ROS_INFO("Number of points in range = %i", num_in_range_pts);
	
	//Find the minimums
	//Defualt to forward
	int half_index = ranges_.size() / 2;
	int index_of_min = half_index;
	int min = potential_field[index_of_min];
	for (int iii = 1; iii < ranges_.size(); ++iii)
	{
		float weight = potential_field[iii];
		if(min > weight)
		{
			//ROS_INFO("Old min: %i, New Min: %i", min, potential_field[iii]);
			index_of_min = iii;
			min = ranges_[iii];
		}
		else if(min == weight)
		{
			if(abs(iii - half_index) < abs(index_of_min - half_index))
			{
				index_of_min = iii;
				min = ranges_[iii];
			}
		}
	}
	
	ROS_INFO("Index of min: %i", index_of_min);
	
	//Publish the result
	geometry_msgs::PoseStamped result;
	result.pose.position.x = 0;
	result.pose.position.y = 0;
	result.pose.position.z = 0;
	result.pose.orientation.x = 0;
	result.pose.orientation.y = 0;
	result.pose.orientation.z = 1;
	result.pose.orientation.w = cos(min_angle_ + index_of_min * angle_inc_);
	result.header.frame_id = "lidar";
	result.header.stamp = ros::Time::now();
		
	out_pub.publish(result);
	
	delete[] potential_field;
	potential_field = NULL;
	*/
	
	
	/*
	 *  New algorithim
	 *  	Each point contributes a vector perpindicular to the line formed between the boat and the point
	 *  	The vector has a magnitude linearly corespoinding to the distance between the mesured point and the lidar
	 *  	The sign of the vector is chosen such that the vector lies in the first and fourth quadriant (positive x)
	 *  	Each vector is sumed to find the final value
	 *    Explination:
	 *    	This algorithim is similar to a potential field methode in 1 Dimension
	 *    	by mirrioring points across the y-axis the optimal direction to avoid that pointis then always perpendicular 
	 *    	to the line formed between the boat and the point. By choosing the vector in the positive x direction we ensure
	 *    	the boat will go forward, otherwise the boat would always choose to reverse (Because we can't see behind us so it
	 *    	assumes empty space [which isn't a horrible assumption since we likely came from that direction]). These directions
	 *    	are wiegthed by how close the point is, that way points that are far away will tend to be ignored, and as a less 
	 *    	weighted point is approched it gains weight and pushes the optimal direction away from it (ideally close up pts
	 *    	would have infinite weight)
	 *  	
	 *    Notes:
	 *    	* The prossesing is done in the lidar frame of reference and then converted into the base_link frame of reference
	 *    	* A low pass filter(averageing function) could be implemeted to reduce the number of calculated pts
	 */
	
	//Define a vector for the solution
	simple2dVector solution;
	geometry_msgs::PoseArray poses;
	geometry_msgs::PoseArray points;
	poses.header.frame_id = "lidar";
	points.header.frame_id = "lidar";
	
	//Loop through every point
	for(int iii = 0; iii < ranges_.size(); ++iii)
	{
		float range = ranges_[iii];
		if(!isOutOfRange(range))
		{
			float angle = min_angle_ + angle_inc_ * iii;
			
			geometry_msgs::Pose result;
			result.position.x = 0;
			result.position.y = 0;
			result.position.z = 0;
			result.orientation.x = 0;
			result.orientation.y = 0;
			result.orientation.z = sin(angle/2);
			result.orientation.w = cos(angle/2);
			points.poses.push_back(result);
			
			//Get the perpendicular line
			if(angle > 0.0f)
			{
				angle -= PI / 2;
			}
			else
			{
				angle += PI / 2;
			}
			//Perpendicular to the line between the lidar  and the point and in quadriant I or II
			//Magnitude is specified by weighting
			solution += simple2dVector(RangeToWeight(ranges_[iii]), angle);
			//geometry_msgs::Pose result;
			result.position.x = 0;
			result.position.y = 0;
			result.position.z = 0;
			result.orientation.x = 0;
			result.orientation.y = 0;
			result.orientation.z = sin(angle/2);
			result.orientation.w = cos(angle/2);
			poses.poses.push_back(result);
		}
	}
	
	ROS_INFO("Solution: %f<%f", solution.mag_, solution.angle_*180/PI);
	
	//Publish the result
	geometry_msgs::PoseStamped result;
	result.pose.position.x = 0;
	result.pose.position.y = 0;
	result.pose.position.z = 0;
	result.pose.orientation.x = 0;
	result.pose.orientation.y = 0;
	result.pose.orientation.z = sin(solution.angle_/2);
	result.pose.orientation.w = cos(solution.angle_/2);
	result.header.frame_id = "lidar";
	result.header.stamp = ros::Time::now();
		
	out_pub.publish(result);
	debug_pub.publish(poses);
	debug_pub2.publish(points);
	
}

#endif
