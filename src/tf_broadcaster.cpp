#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "p3dx_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster bc;

	while(n.ok()){
		bc.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.175,0.0,0.15)),
			ros::Time::now(), "base_link", "laser"));
		
		bc.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.15,0.0,0.06)),
			ros::Time::now(), "base_link", "sonar_frame"));
		r.sleep();
	}

}
