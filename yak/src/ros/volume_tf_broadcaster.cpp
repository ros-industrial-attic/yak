#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "volume_tf_broadcaster");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);

  while (nh.ok()) {
    transform.setOrigin((tf::Vector3(0.0, 0.0, 0.0)));

    transform.setRotation(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(1.5708)));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "volume_frame"));

    rate.sleep();
  }
  return 0;
}
