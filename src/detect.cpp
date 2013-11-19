#include <string>
#include <ros/ros.h>
#include "chilitagsdetector.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "ros_markers");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    // load parameters
    string configFilename;
    _private_node.param<string>("markers_configuration", configFilename, "");
    double squareSize, gain;
    _private_node.param<double>("default_marker_size", squareSize, 50.);
    _private_node.param<double>("gain", gain, 0.9);

    string camera_frame;
    _private_node.param<string>("camera_frame_id", camera_frame, "camera");

    if (configFilename.empty() && squareSize == 0.) {
        ROS_ERROR_STREAM("Either a marker configuration file or a default marker size\n" <<
                         "must be passed as parameter (both are also ok).");
        return(1);
    }

    // initialize the detector by subscribing to the camera video stream
    ChilitagsDetector detector(rosNode, camera_frame, configFilename, squareSize, gain);
    ROS_INFO("ros_markers is ready. Marker locations will be published on TF when detected.");
    ROS_INFO("Tracking %d markers associated to %d objects.", detector.nbTrackedMarkers(), detector.nbTrackedObjects());

    ros::spin();

    return 0;
}

