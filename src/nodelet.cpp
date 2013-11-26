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
    double squareSize, gain;
    _private_node.param<double>("default_marker_size", squareSize, 50.);
    _private_node.param<double>("gain", gain, 0.9);

    string camera_frame;
    _private_node.param<string>("camera_frame_id", camera_frame, "camera");

    if (configFilename.empty() && squareSize == 0.) {
        NODELET_ERROR_STREAM("Either a marker configuration file or a default marker size\n" <<
                         "must be passed as parameter (both are also ok). Quitting now.");
    }

    // initialize the detector by subscribing to the camera video stream
    chilitags_.reset(new ChilitagsDetector(rosNode, camera_frame, configFilename, squareSize, gain));
    ROS_INFO("ros_markers nodelet is ready. Marker locations will be published on TF when detected.");
    NODELET_INFO("Tracking %d markers associated to %d objects.", 
            chilitags_->nbTrackedMarkers(), 
            chilitags_->nbTrackedObjects());

}

// Register this plugin with pluginlib.  Names must match nodelet_chilitags.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(ros_markers, detector,
                        ChilitagsNodelet, nodelet::Nodelet);
