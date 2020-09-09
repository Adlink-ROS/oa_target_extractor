// Copyright (c) 2018 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// cpplint: c system headers
//#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include <object_analytics_msgs/msg/object_in_box3_d.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// cpplint: c++ system headers
//#include <algorithm>
//#include <csignal>
//#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <string>
#include <utility>
#include <math.h>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

#define CUTOFF_FREQ 1.5

class OATargteExtractor : public rclcpp::Node
{
public:
	OATargteExtractor()
	: Node("oa_target_extracker"), 
	_tf_broadcaster(this),
	_tgt_tf_broad(this),
	_ros_clock(RCL_ROS_TIME)
	{
		_localization_sub = this->create_subscription<object_analytics_msgs::msg::ObjectsInBoxes3D>(
		"/object_analytics/localization", std::bind(&OATargteExtractor::topic_callback, this, std::placeholders::_1));
		
		_tracking_sub = this->create_subscription<object_analytics_msgs::msg::TrackedObjects>(
		"/object_analytics/tracking", std::bind(&OATargteExtractor::cb_tracking, this, std::placeholders::_1));
		
		_pcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		"/camera/aligned_depth_to_color/color/points", std::bind(&OATargteExtractor::cb_pcloud, this, std::placeholders::_1));
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
		qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		qos_profile.depth = 1;
		_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("human_poses", qos_profile);
		_tfpoint = create_publisher<geometry_msgs::msg::Point>("point");
		_marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("crowds_marker");
		_tgt_marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("target_marker");
	  
		_timer = this->create_wall_timer(50ms, std::bind(&OATargteExtractor::_timercallback, this));
		
		_cam_frame_id = "camera_color_optical_frame";
		_pclcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);
		_last_lp_time = _ros_clock.now();
		_last_tgt_time = _last_lp_time;
		_last_worldtf_time = _last_lp_time;
		_tracking_alive = false;
		_target_alive = false;
		_world_tf_alive = false;
		_new_pcloud = false;
		_new_tgt = false;
		
		RCLCPP_INFO(this->get_logger(), "Tracking initialized!");
	}

private:
	tf2_ros::TransformBroadcaster _tf_broadcaster, _tgt_tf_broad;
	std::shared_ptr<tf2_ros::Buffer> _tfBuffer;
	std::shared_ptr<tf2_ros::TransformListener> _tf_listener; 
	
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr _tfpoint;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _publisher;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _tgt_marker_pub;
	rclcpp::Subscription<object_analytics_msgs::msg::ObjectsInBoxes3D>::SharedPtr _localization_sub;
	rclcpp::Subscription<object_analytics_msgs::msg::TrackedObjects>::SharedPtr _tracking_sub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pcloud_sub;
	
	rclcpp::TimerBase::SharedPtr _timer;
	
	rclcpp::Clock _ros_clock;
	rclcpp::Time _last_lp_time;
	int _current_id;
	bool _target_alive;
	bool _tracking_alive;
	bool _world_tf_alive;
	rclcpp::Time _last_tgt_time, _last_worldtf_time;
	
	//std::shared_ptr<sensor_msgs::msg::PointCloud2> _last_pcloud;
	std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> _pclcloud;
	bool _new_pcloud;
	
	geometry_msgs::msg::TransformStamped _tgt_tf, _last_tgt_tf, _vehicle_tgt_tf, _world_to_cam_tf;
	geometry_msgs::msg::PoseArray _tgt_pose;
	std::string _cam_frame_id;
	bool _new_tgt;
  
	// Markers
	visualization_msgs::msg::Marker person_marker;
	visualization_msgs::msg::MarkerArray _crowds_mk;
	visualization_msgs::msg::MarkerArray _tgt_mk;
	
	/*************************
	 *     Subscirber
	 * 
	 *************************/
	void topic_callback(const object_analytics_msgs::msg::ObjectsInBoxes3D::SharedPtr obj)
	{
		std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
		geometry_msgs::msg::Point tf_point;
		int i = 0;
		for (auto box = obj->objects_in_boxes.begin(); box!=obj->objects_in_boxes.end(); box++)
		{
			// Say something
			geometry_msgs::msg::TransformStamped tf_transform;
			geometry_msgs::msg::Point tfpoint;

			tf_transform.header.stamp = obj->header.stamp;
			tf_transform.header.frame_id = obj->header.frame_id;
			tfpoint.x = (box->min.x + box->max.x)/2;
			tfpoint.y = (box->min.y + box->max.y)/2;
			tfpoint.z = (box->min.z + box->max.z)/2;
			
			tf_transform.child_frame_id = box->object.object_name + std::to_string(i);
			tf_transform.transform.translation.x = tfpoint.x;
			tf_transform.transform.translation.y = tfpoint.y;
			tf_transform.transform.translation.z = tfpoint.z;
			tf_transform.transform.rotation.x = 0;
			tf_transform.transform.rotation.y = 0;
			tf_transform.transform.rotation.z = 0;
			tf_transform.transform.rotation.w = 1;
			tf_point = tfpoint;
			tf_transforms.push_back(tf_transform);
			i++;
			
			// for human marker publisher
			if( (!_world_tf_alive)) {	continue; 	}
			std_msgs::msg::Header mk_header = obj->header;
			mk_header.frame_id = "/world";
			
			tf2::Stamped<tf2::Transform> world2cam_tf2, cam2tgt_tf2;
			tf2::fromMsg(_world_to_cam_tf, world2cam_tf2);
			tf2::fromMsg(tf_transform, cam2tgt_tf2);
			tf2::Vector3 ttt = (world2cam_tf2*cam2tgt_tf2).getOrigin();
			tfpoint.x = ttt.x();
			tfpoint.y = ttt.y();
			tfpoint.z = ttt.z();
			
			int mk_id = 0;
			_crowds_mk.markers.clear();
			if(box->object.object_name == "person")
			{
				addHumanMarker(_crowds_mk, mk_header, tfpoint, mk_id, 1);
			}
				    
		}
		_tf_broadcaster.sendTransform(tf_transforms);
		_marker_pub->publish(_crowds_mk);
		_tfpoint->publish(tf_point);
	}
	
	void cb_pcloud(const sensor_msgs::msg::PointCloud2::SharedPtr pcloud)
	{
		pcl::fromROSMsg<pcl::PointXYZRGB>( (*pcloud), (*_pclcloud) );
		_new_pcloud = true;
	}
	
	void cb_tracking(const object_analytics_msgs::msg::TrackedObjects::SharedPtr trk_obj)
	{
		if(!_world_tf_alive) {	return; }
		if(!_target_alive)
		{
			// select one obj as target
			for( auto obj = trk_obj->tracked_objects.begin(); obj!=trk_obj->tracked_objects.end(); obj++)
			{
				if(obj->object.object_name == "person" && obj->object.probability > 0.7)
				{
					_current_id = obj->id;
					_target_alive = true;
					RCLCPP_INFO(this->get_logger(), "Found target #%d", _current_id);
					break;
				}
			}
		}
		
		if(!_target_alive)
		{
			return;
		}else
		{
			// find target id
			for( auto obj = trk_obj->tracked_objects.begin(); obj!=trk_obj->tracked_objects.end(); obj++)
			{
				if(obj->id == _current_id)
				{
					if(!_new_pcloud || obj->object.probability < 0.55) {
						return;
					}
					_new_pcloud = false;
					
					int pix_center_x = obj->roi.x_offset + ((float)obj->roi.width/2);
					int pix_center_y = obj->roi.y_offset + ((float)obj->roi.height/2);
					int pix_center_ind = pix_center_y * _pclcloud->width + pix_center_x;
					
					if((unsigned int)pix_center_ind >= _pclcloud->size()) {
						return;
					}
					
					//get coordinates
					float _tgt_x = _pclcloud->points[pix_center_ind].x;	// get pointcloud_vehicle_tgt_tf distance
					float _tgt_y = _pclcloud->points[pix_center_ind].y;
					float _tgt_z = _pclcloud->points[pix_center_ind].z;
					
					if(!std::isfinite(_tgt_z))
					{
						//RCLCPP_INFO(this->get_logger(), "IS INFINITE");	
						return;
					}
					
					geometry_msgs::msg::TransformStamped target;
					target.header.stamp = trk_obj->header.stamp;
					target.header.frame_id = trk_obj->header.frame_id;
					target.transform.translation.x = _tgt_x;
					target.transform.translation.y = _tgt_y;
					target.transform.translation.z = _tgt_z;
					target.transform.rotation.x = 0;
					target.transform.rotation.y = 0;
					target.transform.rotation.z = 0;
					target.transform.rotation.w = 1;
					
					tf2::Stamped<tf2::Transform> world2cam_tf2, cam2tgt_tf2;
					tf2::fromMsg(_world_to_cam_tf, world2cam_tf2);
					tf2::fromMsg(target, cam2tgt_tf2);
					tf2::Vector3 target_world_coord = (world2cam_tf2*cam2tgt_tf2).getOrigin();
										
					//Check within target zone
					if( target_world_coord.y() < - 4.50)
					{
						//RCLCPP_INFO(this->get_logger(), "Too far");
						return;
					}
					
					_last_tgt_time = _ros_clock.now();
					_new_tgt = true;
					_tracking_alive = true;
					
					_tgt_tf = target;
					return;
				}
			}
		}
		
	}	
	
	/*************************
	 *     Publisher/timer
	 * 
	 *************************/
	void _timercallback()
	{
		rclcpp::Time now = _ros_clock.now();
		rclcpp::Duration dur = now - _last_lp_time;
		
		// initial hold before we have tf
		if( (!_world_tf_alive) && (now - _last_worldtf_time).seconds() < 1.0 ) 
		{
			return;
		}
		
		try{
			fprintf(stderr, "***!!! CT: cam_frame_id=%s\n", _cam_frame_id.c_str());
			_world_to_cam_tf = _tfBuffer->lookupTransform("world", _cam_frame_id, tf2::TimePointZero);
			_world_tf_alive = true;
		}
		catch (tf2::TransformException &ex) {
			if(!_world_tf_alive) {
				RCLCPP_WARN(this->get_logger(), "%s", ex.what());
				RCLCPP_WARN(this->get_logger(), "Will Wait 2 seconds and try again...");
				_last_worldtf_time = now;
			}else {
				RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
			}
			return;
		}
		
		if(!_new_tgt && (now-_last_tgt_time).seconds() > 1.0)
		{
			if(_target_alive) {
				RCLCPP_INFO(this->get_logger(), "Loose target");
			}
			_target_alive = false;	// seeing no new targets, searching for new one
			_last_tgt_time = now;
			return;
		}
		if(!_tracking_alive) {
			return;		// prevent publishing anything before we've found a target
		}

		// Low pass, 1st order
		double alpha, alpha_compl;
		double cutoff_freq_rad;
		cutoff_freq_rad = 2 * M_PI*CUTOFF_FREQ * dur.seconds();
		alpha = cutoff_freq_rad / (cutoff_freq_rad+1);
		alpha_compl = 1 / (cutoff_freq_rad+1);
		
		//=== 3D tracking target ===
		_last_tgt_tf.transform.translation.x = alpha*_tgt_tf.transform.translation.x + alpha_compl*_last_tgt_tf.transform.translation.x;
		_last_tgt_tf.transform.translation.y = alpha*_tgt_tf.transform.translation.y + alpha_compl*_last_tgt_tf.transform.translation.y;
		_last_tgt_tf.transform.translation.z = alpha*_tgt_tf.transform.translation.z + alpha_compl*_last_tgt_tf.transform.translation.z;
		_last_tgt_tf.transform.rotation = _tgt_tf.transform.rotation;
		
		_last_tgt_tf.header = _tgt_tf.header;
		_last_tgt_tf.header.stamp = now;
		
		_last_tgt_tf.child_frame_id = std::string("tracking_target");
		
		//DEBUG RCLCPP_INFO(this->get_logger(), "/%s->/%s; X Y Z: %f, %f, %f", _last_tgt_tf.header.frame_id.c_str(), _last_tgt_tf.child_frame_id.c_str(),_last_tgt_tf.transform.translation.x, _last_tgt_tf.transform.translation.y, _last_tgt_tf.transform.translation.z );
		
		_tgt_tf_broad.sendTransform(_last_tgt_tf);
		_new_tgt = false;
		_last_lp_time = now;
		
		//=== Vehicle tracking target ===
		geometry_msgs::msg::TransformStamped tf_world_to_tgt;
		try{
			tf_world_to_tgt = _tfBuffer->lookupTransform("world", _last_tgt_tf.child_frame_id, tf2::TimePointZero);
		}
		catch (tf2::TransformException &ex) {
			RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
			return;
		}

		_vehicle_tgt_tf.header = tf_world_to_tgt.header;
		_vehicle_tgt_tf.header.stamp = now;
		_vehicle_tgt_tf.child_frame_id = std::string("deliver_target");
		_vehicle_tgt_tf.transform.translation.x = clamp(tf_world_to_tgt.transform.translation.x, 4.00, 5.8); 
		_vehicle_tgt_tf.transform.translation.y = -2.7;
		_vehicle_tgt_tf.transform.translation.z = 0;
		tf2::Quaternion vehicle_heading;
		vehicle_heading.setEuler(0, 0, 90 * -M_PI / 180);
		_vehicle_tgt_tf.transform.rotation = tf2::toMsg(vehicle_heading);
		_tgt_tf_broad.sendTransform(_vehicle_tgt_tf);
		
		geometry_msgs::msg::Pose vehicle_tgt_pose;
		tf2::Stamped<tf2::Transform> _vehicle_tgt_tf2;
		tf2::fromMsg(_vehicle_tgt_tf, _vehicle_tgt_tf2);
		tf2::toMsg(_vehicle_tgt_tf2, vehicle_tgt_pose);
		
		_tgt_pose.header = _vehicle_tgt_tf.header;
		_tgt_pose.poses.clear();
		_tgt_pose.poses.push_back( vehicle_tgt_pose );
		_publisher->publish(_tgt_pose);
		
		
		// for target marker publisher
		geometry_msgs::msg::Point tfpoint;	
		std_msgs::msg::Header mk_header = _tgt_pose.header;
		int mk_id = 0;
		_tgt_mk.markers.clear();
	
		tfpoint.x = tf_world_to_tgt.transform.translation.x;
		tfpoint.y = tf_world_to_tgt.transform.translation.y;
		tfpoint.z = tf_world_to_tgt.transform.translation.z;		
		addHumanMarker(_tgt_mk, mk_header, tfpoint, mk_id, 2);
		
		tfpoint.x = _tgt_pose.poses[0].position.x;
		tfpoint.y = _tgt_pose.poses[0].position.y;
		tfpoint.z = _tgt_pose.poses[0].position.z;	
		addTargetMarker(_tgt_mk, mk_header, tfpoint, mk_id,2);
		_tgt_marker_pub->publish(_tgt_mk);	
	}
	
	
	void addHumanMarker(
			visualization_msgs::msg::MarkerArray & marker_array,
			std_msgs::msg::Header header,
			geometry_msgs::msg::Point tfpoint,
			int & marker_id, int type											) 
	{
		visualization_msgs::msg::Marker body_mk, head_mk;

		// publish rviz markers
		//static ros::Time last_seen;
		body_mk.header = header;
		body_mk.ns = "People_detected";
		head_mk.header = header;
		head_mk.ns = "People_detected";
		
		//body_mk.color.a = (ros::Duration(3) - (now - last_seen)).toSec()/ros::Duration(3).toSec() + 0.1;

		// Cylinder for body 
		body_mk.id = marker_id;
		body_mk.pose.position.x = tfpoint.x;
		body_mk.pose.position.y = tfpoint.y;
		body_mk.pose.position.z = 0.8;
		body_mk.type = visualization_msgs::msg::Marker::CYLINDER;
		if(type ==2 ) {
			body_mk.scale.x = 0.25;
			body_mk.scale.y = 0.25;
			body_mk.scale.z = 1.2;
			body_mk.color.r = 1.0;
			body_mk.color.g = 0;
			body_mk.color.b = 1.0;
		}else {
			body_mk.scale.x = 0.2;
			body_mk.scale.y = 0.2;
			body_mk.scale.z = 1.2;
			body_mk.color.r = 0.8;
			body_mk.color.g = 0.8;
			body_mk.color.b = 0.8;
		}
		body_mk.color.a = 1.0;
		marker_array.markers.emplace_back(body_mk);
		marker_id++;
		//std::cout << "pub" << std::endl;

		// Sphere for head shape                        
		head_mk.type = visualization_msgs::msg::Marker::SPHERE;
		head_mk.pose.position.x = tfpoint.x;
		head_mk.pose.position.y = tfpoint.y;
		head_mk.pose.position.z = 1.5;
	
		if(type == 2 ) {
			head_mk.scale.x = 0.25;
			head_mk.scale.y = 0.25;
			head_mk.scale.z = 0.25;
			head_mk.color.r = 1.0;
			head_mk.color.g = 0;
			head_mk.color.b = 1.0;
		}else {
			head_mk.scale.x = 0.2;
			head_mk.scale.y = 0.2;
			head_mk.scale.z = 0.2;
			head_mk.color.r = 0.8;
			head_mk.color.g = 0.8;
			head_mk.color.b = 0.8;
		}
		head_mk.color.a = 1.0;
		head_mk.id = marker_id;
		marker_array.markers.emplace_back(head_mk);
		marker_id ++;                
	}
	void addTargetMarker(
			visualization_msgs::msg::MarkerArray & marker_array,
			std_msgs::msg::Header header,
			geometry_msgs::msg::Point tfpoint,
			int & marker_id, int type											) 
	{
		visualization_msgs::msg::Marker body_mk, head_mk;

		// publish rviz markers
		//static ros::Time last_seen;
		body_mk.header = header;
		body_mk.ns = "People_detected";
		head_mk.header = header;
		head_mk.ns = "People_detected";
		
		//body_mk.color.a = (ros::Duration(3) - (now - last_seen)).toSec()/ros::Duration(3).toSec() + 0.1;

		// Sphere                   
		head_mk.type = visualization_msgs::msg::Marker::SPHERE;
		head_mk.pose.position.x = tfpoint.x;
		head_mk.pose.position.y = tfpoint.y;
		head_mk.pose.position.z = 0.6;
		head_mk.scale.x = 0.4;
		head_mk.scale.y = 0.4;
		head_mk.scale.z = 0.4;
		if(type == 2 ) {			
			head_mk.color.r = 1.0;
			head_mk.color.g = 0;
			head_mk.color.b = 0;
		}else {
			head_mk.color.r = 0.8;
			head_mk.color.g = 0.8;
			head_mk.color.b = 0.8;
		}
		head_mk.color.a = 1.0;
		head_mk.id = marker_id;
		marker_array.markers.emplace_back(head_mk);
		marker_id ++;                
	}
	
	float clamp(float n, float lower, float upper) {
		return std::max(lower, std::min(n, upper));
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OATargteExtractor>());
  rclcpp::shutdown();
  return 0;
}
