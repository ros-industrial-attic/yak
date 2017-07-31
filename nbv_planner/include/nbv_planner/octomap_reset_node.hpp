#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <std_srvs/Empty.h>

class OctomapResetter {
public:
  OctomapResetter(ros::NodeHandle& nh);

  void UpdateCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  ros::Subscriber volume_update_subscriber_;
  ros::ServiceClient volume_reset_client_;

private:

};
