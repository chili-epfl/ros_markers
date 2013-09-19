// This file serves as an illustration of how to use cv::solvePnP to estimate the 6D pose of a marker.
// It requires the chilitags library for 2D tag recognition.


// This header contains the detection part
#include <DetectChilitags.hpp>
// This header provides an easy way to use the tag detection information
#include <Chilitag.hpp>

#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cstdio>
#include <iostream>
#include <vector>
#include <map>

#include "estimator.h"
#include "objectconfig.h"

// maximum of objects or tags that can be tracked. Must be =<1024
#define MAX_OBJECTS 1024

using namespace std;
using namespace cv;

//  *** For the GUI ***
const static cv::Scalar scColor(255, 0, 255);
// These constants will be given to OpenCv for drawing with
// sub-pixel accuracy with fixed point precision coordinates
static const int scShift = 16;
static const float scPrecision = 1<<scShift;
// *********************

class BaseMarkerBroadcaster {

public:
    BaseMarkerBroadcaster(InputArray cameraMatrix, InputArray distCoeffs, float gain = 0.9):
        cameraMatrix(cameraMatrix),
        distCoeffs(distCoeffs)
    {
        // create the estimators with specified gain.
        for (int i = 0; i < MAX_OBJECTS; ++i) {
            estimatedTranslations.insert(pair<int, Estimator<Mat>>(i, Estimator<Mat>(gain)));
            estimatedRotations.insert(pair<int, Estimator<Mat>>(i, Estimator<Mat>(gain)));
        }


    }

    void publish() {

        for (auto& kv : names) {

            setROSTransform(estimatedRotations[kv.first](), 
                            estimatedTranslations[kv.first](), 
                            transform);

            br.sendTransform(tf::StampedTransform(
                            transform, 
                            ros::Time::now(), 
                            "camera", 
                            kv.second));
        }


    }

    void setOutputImage(cv::Mat& image) {
        outputImage = &image;
    }

protected:
    InputArray cameraMatrix;
    InputArray distCoeffs;

    // store, for each tag ID, an estimator
    map<int, Estimator<Mat>> estimatedTranslations;
    map<int, Estimator<Mat>> estimatedRotations;

    map<int, string> names;

    cv::Mat* outputImage;

private:
    tf::TransformBroadcaster br;
    tf::Transform transform;

    void setROSTransform(const cv::Mat& rvec, const cv::Mat& tvec, 
                         tf::Transform& transform)
    {
        transform.setOrigin( tf::Vector3( tvec.at<double>(0) / 1000,
                                        tvec.at<double>(1) / 1000,
                                        tvec.at<double>(2) / 1000) );

        // the process to get a ROS quaternion out of a OpenCV rotation vector is
        // not optimal: 
        // rotation vector -> OpenCV rotation matrix -> ROS matrix -> ROS quaternion...
        tf::Quaternion qrot;
        Mat cvmrot;
        Rodrigues(rvec, cvmrot);
        tf::Matrix3x3 mrot(
            cvmrot.at<double>(0), cvmrot.at<double>(1), cvmrot.at<double>(2),
            cvmrot.at<double>(3), cvmrot.at<double>(4), cvmrot.at<double>(5),
            cvmrot.at<double>(6), cvmrot.at<double>(7), cvmrot.at<double>(8));
        mrot.getRotation(qrot);
        transform.setRotation(qrot);
    }

};

class MarkerBroadcaster : public BaseMarkerBroadcaster {

public:
    MarkerBroadcaster(InputArray cameraMatrix, InputArray distCoeffs, float markerSize, float gain = 0.9, bool gui = false): BaseMarkerBroadcaster(cameraMatrix, distCoeffs, gain) {

        hasGUI = gui;
        markerCorners.push_back(Point3f(0.,0.,0.));
        markerCorners.push_back(Point3f(markerSize,0.,0.));
        markerCorners.push_back(Point3f(markerSize,markerSize,0.));
        markerCorners.push_back(Point3f(0.,markerSize,0.));
    }

    void process() {

        // We iterate over the 1024 possible tags (from #0 to #1023)
        for (int tTagId = 0; tTagId < 1024; ++tTagId) {

            chilitags::Chilitag tTag(tTagId);

            if (tTag.isPresent()) {

                names[tTagId] = string("marker_") + to_string(tTagId);

                chilitags::Quad tCorners = tTag.getCorners();

                imagePoints.clear();
                for (auto p : tCorners.mCorners) {
                    imagePoints.push_back(p);
                }

                /****************************************************************
                *             Pose estimation for each seen marker              *
                *****************************************************************/
                
                // Rotation & translation vectors, computed by cv::solvePnP
                Mat rvec, tvec;

                // Find the 3D pose of our marker
                solvePnP(markerCorners, imagePoints, 
                        cameraMatrix, distCoeffs,
                        rvec, tvec, false,
                        cv::ITERATIVE);

                estimatedTranslations[tTagId] << tvec;
                estimatedRotations[tTagId] << rvec;

                if (hasGUI) draw(imagePoints);

            }

        }
    }

    void draw(const vector<Point2f>& tCorners) {

        // We draw this quadrilateral
        for (size_t i = 0; i < 4; ++i) {
            cv::line(
                *outputImage,
                scPrecision*tCorners[i],
                scPrecision*tCorners[(i+1)%4],
                scColor, 1, CV_AA, scShift);
        }
    }

private:
    bool hasGUI;
    vector<Point3f> markerCorners;
    vector<Point2f> imagePoints;

};

static bool readCameraMatrix(const string& filename,
                             Mat& cameraMatrix, Mat& distCoeffs,
                             Size& calibratedImageSize )
{
    cout << "Reading camera calibration from " << filename << "..." << endl;
    FileStorage fs(filename, FileStorage::READ);
    fs["image_width"] >> calibratedImageSize.width;
    fs["image_height"] >> calibratedImageSize.height;
    fs["distortion_coefficients"] >> distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;

    if( distCoeffs.type() != CV_64F )
        distCoeffs = Mat_<double>(distCoeffs);
    if( cameraMatrix.type() != CV_64F )
        cameraMatrix = Mat_<double>(cameraMatrix);

    return true;
}

int main(int argc, char* argv[])
{
    const char* help = "Usage: detect [-g|--gui] [--gain <filter gain, default 0.9>] -s <default marker size> [-c <marker config (YAML)>] -i <camera calibration (YAML)>\n";
 
    bool hasGUI = false;
    bool filtered = true;

    // gain = 1 -> no filtering (only measurements)
    float gain = 0.9;

    char* intrinsicsFilename = 0;
    char* markersConfigFilename = 0;
    double squareSize = 0.;

    for( int i = 1; i < argc; i++ )
    {
        if( strcmp(argv[i], "-g") == 0 ||
            strcmp(argv[i], "--gui") == 0)
        {
            cout << "Running with GUI" << endl;
            hasGUI = true;
        }
        else if( strcmp(argv[i], "-c") == 0 )
            markersConfigFilename = argv[++i];
        else if( strcmp(argv[i], "-i") == 0 )
            intrinsicsFilename = argv[++i];
        else if( strcmp(argv[i], "--gain") == 0 )
        {
            if(sscanf(argv[++i], "%f", &gain) != 1 || gain < 0 || gain > 1)
            {
                printf("Incorrect --gain parameter (must be 0.0 < gain < 1.0)\n");
                puts(help);
                return 0;
            }
            else
            {
                cout << "Estimator gain set to " << gain << endl;
            }
        }
        else if( strcmp(argv[i], "-s") == 0 )
        {
            if(sscanf(argv[++i], "%lf", &squareSize) != 1 || squareSize <= 0)
            {
                printf("Incorrect -s parameter (must be a positive real number)\n");
                puts(help);
                return 0;
            }
        }
    }
    if (intrinsicsFilename == 0 || squareSize == 0)
    {
        puts(help);
        return 0;
    }

    ObjectConfig objects(markersConfigFilename);

    //ROS initialization
    ros::init(argc, argv, "chilitags_tf_broadcaster");
    ros::NodeHandle rosNode;

    // For (basic) profiling
    int64 startTickCount;
    double tagRecoDuration, poseEstimationDuration;

    // Format floats
    cout.precision(1);
    cout.setf(ios::fixed, ios::floatfield);

    int tCameraIndex = 0;

    Mat cameraMatrix, distCoeffs;
    Size calibratedImageSize;
    readCameraMatrix(intrinsicsFilename, cameraMatrix, distCoeffs, calibratedImageSize );


    // The source of input images
    cv::VideoCapture tCapture(tCameraIndex);
    if (!tCapture.isOpened())
    {
        std::cerr << "Unable to initialise video capture." << std::endl;
        return 1;
    }
    tCapture.set(CV_CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
    tCapture.set(CV_CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);

    if (hasGUI) cv::namedWindow("TagPoseEstimation");


    // The tag detection happens in the DetectChilitags class.
    // All it needs is a pointer to a OpenCv Image, i.e. a cv::Mat *
    // and a call to its update() method every time the image is updated.
    cv::Mat tInputImage;
    cv::Mat tOutputImage;
    chilitags::DetectChilitags tDetectChilitags(&tInputImage);

    MarkerBroadcaster markerBroadcaster(cameraMatrix, distCoeffs, 27, gain, hasGUI);

    while (rosNode.ok() ) {

        // need to leave some time to OpenCV to display smthg
        if (hasGUI) cv::waitKey(10);

        // Capture a new image.
        tCapture.read(tInputImage);

        if (hasGUI) {
            // We dont want to draw directly on the input image, so we clone it
            tOutputImage = tInputImage.clone();
        }


        /********************************************************************
         *                      Markers detection                           *
         ********************************************************************/
        startTickCount = cv::getTickCount();
        tDetectChilitags.update();
        tagRecoDuration = (1000 * (double) (cv::getTickCount()-startTickCount)) / cv::getTickFrequency();

        startTickCount = cv::getTickCount();

        markerBroadcaster.process();
        markerBroadcaster.publish();




        poseEstimationDuration = (1000 * (double) (cv::getTickCount()-startTickCount)) / cv::getTickFrequency();

        cout << "\x1b[K" 
            << "Total time: " << poseEstimationDuration + tagRecoDuration << "ms "
            << "(tag detection: " << tagRecoDuration << "ms, "
            << ", pose estim. + ROS export: " << poseEstimationDuration << "ms" << endl << "\x1b[1F";

        if (hasGUI) cv::imshow("TagPoseEstimation", tOutputImage);
    }

    cout << endl;

    if (hasGUI) cv::destroyWindow("TagPoseEstimation");
    tCapture.release();

    return 0;
}

