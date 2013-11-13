#include <string>

// chilitags
#include <DetectChilitags.hpp>
#include <Objects.hpp>

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

using namespace std;
using namespace cv;

class ChilitagsDetector
{
public:
    ChilitagsDetector(ros::NodeHandle& rosNode, const string& camera_frame, const string& configFilename, double squareSize, double gain = 0.9) :
            rosNode(rosNode),
            it(rosNode),
            camera_frame(camera_frame),
            detector(&inputImage),
            firstUncalibratedImage(true),
#ifdef WITH_KNOWLEDGE
            connector("localhost", "6969"),
#endif
            objects(cv::noArray(), cv::noArray(), configFilename, squareSize, gain)
    {
#ifdef WITH_KNOWLEDGE
        try {
            kb = oro::Ontology::createWithConnector(connector);
        } catch (oro::OntologyServerException ose) {
            ROS_ERROR_STREAM("Could not connect to the knowledge base: " << ose.what());
            exit(1);
        }
#endif
        sub = it.subscribeCamera("image", 1, &ChilitagsDetector::findMarkers, this);
    }


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
    string camera_frame;

    image_geometry::PinholeCameraModel cameramodel;
    Mat cameraMatrix, distCoeffs;
    bool firstUncalibratedImage;

    Mat inputImage;
    chilitags::DetectChilitags detector;
    chilitags::Objects objects;
    set<string> objectsSeen;

    void setROSTransform(Matx44d trans, tf::Transform& transform)
    {
        transform.setOrigin( tf::Vector3( trans(0,3) / 1000,
                                        trans(1,3) / 1000,
                                        trans(2,3) / 1000) );

        tf::Quaternion qrot;
        tf::Matrix3x3 mrot(
            trans(0,0), trans(0,1), trans(0,2),
            trans(1,0), trans(1,1), trans(1,2),
            trans(2,0), trans(2,1), trans(2,2));
        mrot.getRotation(qrot);
        transform.setRotation(qrot);
    }

    void findMarkers(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& camerainfo)
    {
        // updating the camera model is cheap if not modified
        cameramodel.fromCameraInfo(camerainfo);
        // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
        // K[0] == 0.0 <=> uncalibrated).
        if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
            if(firstUncalibratedImage) {
                ROS_ERROR("Camera publishes uncalibrated images. Can not detect markers.");
                ROS_WARN("Detection will start over again when camera info is available.");
            }
            firstUncalibratedImage = false;
            return;
        }
        firstUncalibratedImage = true;
        // TODO: can we avoid to re-set the calibration matrices for every frame? ie,
        // how to know that the camera info has changed?
        objects.resetCalibration(cameramodel.intrinsicMatrix(), 
                                    cameramodel.distortionCoeffs());

        // hopefully no copy here:
        //  - assignement operator of cv::Mat does not copy the data
        //  - toCvShare does no copy if the default (source) encoding is used.
        inputImage = cv_bridge::toCvShare(msg)->image; 

         /********************************************************************
         *                      Markers detection                           *
         ********************************************************************/
        detector.update();

        auto foundObjects = objects.all();
        ROS_DEBUG_STREAM(foundObjects.size() << " objects found.");

        /****************************************************************
        *                Publish TF transforms                          *
        *****************************************************************/

#ifdef WITH_KNOWLEDGE
        auto previouslySeen(objectsSeen);
#endif
        objectsSeen.clear();

        for (auto& kv : foundObjects) {

            objectsSeen.insert(kv.first);
            setROSTransform(kv.second, 
                            transform);

            br.sendTransform(
                    tf::StampedTransform(transform, 
                                         ros::Time::now(), 
                                         camera_frame, 
                                         kv.first));
        }

#ifdef WITH_KNOWLEDGE
        set<oro::Statement> stmts;
        for(const auto& obj : objectsSeen) {
            stmts.insert(obj + " isVisible true");
            stmts.insert(obj + " rdf:type FiducialMarker");
        }
        if (!stmts.empty()) kb->add(stmts);

        stmts.clear();
        for (const auto& pobj : previouslySeen) {
            if (objectsSeen.find(pobj) == objectsSeen.end()) {
                stmts.insert(pobj + " isVisible true");
            }
        }
        if (!stmts.empty()) kb->remove(stmts);
#endif


    }
};

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

    ros::spin();

    return 0;
}

