#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "chilitagsdetector.hpp"


using namespace std;

/** @file

    @brief ROS nodelet for Chilitags marker detection.

*/

class ChilitagsNodelet: public nodelet::Nodelet
{
public:
  ChilitagsNodelet()
  {}

  ~ChilitagsNodelet()
  {
    NODELET_INFO("Stopping chilitags detection");
  }

private:
  virtual void onInit();
  boost::shared_ptr<ChilitagsDetector> chilitags_;
};

void ChilitagsNodelet::onInit()
{
    //ROS initialization
    ros::NodeHandle rosNode(getNodeHandle());
    ros::NodeHandle _private_node(getPrivateNodeHandle());

    // load parameters
    string configFilename;
    _private_node.param<string>("markers_configuration", configFilename, "");
    string camera_frame;
    _private_node.param<string>("camera_frame_id", camera_frame, "camera");
    double defaultTagSize, gain;
    _private_node.param<double>("default_marker_size", defaultTagSize, 50.);
    _private_node.param<double>("gain", gain, 0.9);
    int persistence;
    _private_node.param<int>("persistence", persistence, 5);
    bool omitOtherTags;
    _private_node.param<bool>("omit_other_tags", omitOtherTags, false);

    if (configFilename.empty() && omitOtherTags == true) {
        NODELET_ERROR_STREAM("If a marker configuration file is not passed as a parameter,\n" <<
                             "omitOtherTags must not be set to true or no tags will be detected.");
    }

    // initialize the detector by subscribing to the camera video stream
    chilitags_.reset(new ChilitagsDetector(rosNode, camera_frame, configFilename, omitOtherTags,
                                           defaultTagSize, gain, persistence));
    ROS_INFO("ros_markers nodelet is ready. Marker locations will be published on TF when detected.");
    ROS_INFO_STREAM("Camera frame is:" << camera_frame );

}

// Register this plugin with pluginlib.  Names must match nodelet_chilitags.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(ros_markers, detector,
                        ChilitagsNodelet, nodelet::Nodelet);
