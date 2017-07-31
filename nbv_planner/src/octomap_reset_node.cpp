#include <octomap_reset_node.hpp>

OctomapResetter::OctomapResetter(ros::NodeHandle &nh) {
  volume_update_subscriber_ = nh.subscribe("/volume_marker/feedback", 1, &OctomapResetter::UpdateCallback, this);
  volume_reset_client_ = nh.serviceClient<std_srvs::Empty>("/octomap_server/reset");
}

void OctomapResetter::UpdateCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  std_srvs::Empty srv;
  volume_reset_client_.call(srv);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "volume_tf_broadcaster");
  ros::NodeHandle nh;

  OctomapResetter resetter(nh);

  while (true) {
    ros::spinOnce();
  }
  return 0;
}
