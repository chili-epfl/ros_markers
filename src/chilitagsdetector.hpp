#include <string>
#include <set>

// chilitags
#include <chilitags/DetectChilitags.hpp>
#include <chilitags/Objects.hpp>

// opencv2
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#ifdef WITH_KNOWLEDGE
#include <liboro/oro.h>
#include <liboro/socket_connector.h>
#endif

class ChilitagsDetector
{
public:
    ChilitagsDetector(ros::NodeHandle& rosNode, 
                      const std::string& camera_frame, 
                      const std::string& configFilename, 
                      double squareSize, 
                      double gain = 0.9);

    int nbTrackedObjects() const {return objects.nbTrackedObjects();}
    int nbTrackedMarkers() const {return objects.nbTrackedMarkers();}

private:

#ifdef WITH_KNOWLEDGE
    oro::SocketConnector connector;
    oro::Ontology* kb;
#endif

    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string camera_frame;

    image_geometry::PinholeCameraModel cameramodel;
    cv::Mat cameraMatrix, distCoeffs;
    bool firstUncalibratedImage;

    cv::Mat inputImage;
    chilitags::DetectChilitags detector;
    chilitags::Objects objects;
    std::set<std::string> objectsSeen;

    void setROSTransform(cv::Matx44d trans, tf::Transform& transform);

    void findMarkers(const sensor_msgs::ImageConstPtr& msg, 
                     const sensor_msgs::CameraInfoConstPtr& camerainfo);
};

