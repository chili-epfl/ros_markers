#include <string>
#include <iostream>

// chilitags
#include <DetectChilitags.hpp>
#include <Objects.hpp>

// opencv2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> // for video capture

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;

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


int main(int argc, char* argv[])
{
    const char* help = "Usage: detect [--gain <filter gain, default 0.9>] [-c <markers configuration (YAML)> | -s <default marker size>] -i <camera calibration (YAML)>\n";
 
    // gain = 1 -> no filtering (only measurements)
    float gain = 0.9;

    char* intrinsicsFilename = 0;
    string configFilename;
    double squareSize = 0.;

    for( int i = 1; i < argc; i++ )
    {
        if( strcmp(argv[i], "-c") == 0 )
            configFilename = string(argv[++i]);
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
    if (intrinsicsFilename == 0 || (configFilename.empty() && squareSize == 0))
    {
        puts(help);
        return 0;
    }

    //ROS initialization
    ros::init(argc, argv, "chilitags_tf_broadcaster");
    ros::NodeHandle rosNode;
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // For (basic) profiling
    int64 startTickCount;
    double tagRecoDuration, poseEstimationDuration;

    // Format floats
    cout.precision(1);
    cout.setf(ios::fixed, ios::floatfield);

    int tCameraIndex = 0;

    // Read camera calibration
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

    // The tag detection happens in the DetectChilitags class.
    // All it needs is a pointer to a OpenCv Image, i.e. a cv::Mat *
    // and a call to its update() method every time the image is updated.
    cv::Mat tInputImage;
    chilitags::DetectChilitags tDetectChilitags(&tInputImage);

    // the class chilitags::Objects takes care of 3D localization of the markers
    // and of objects (counpound markers)
    chilitags::Objects objects(cameraMatrix, distCoeffs,
                               configFilename,
                               squareSize,
                               gain);

    while (rosNode.ok() ) {

        // Capture a new image.
        tCapture.read(tInputImage);

        /********************************************************************
         *                      Markers detection                           *
         ********************************************************************/
        startTickCount = cv::getTickCount();
        tDetectChilitags.update();
        tagRecoDuration = (1000 * (double) (cv::getTickCount()-startTickCount)) / cv::getTickFrequency();

        startTickCount = cv::getTickCount();

        auto foundObjects = objects.all();
        
        cout << "\x1b[K" << foundObjects.size() << " objects found." << endl;
        /****************************************************************
        *                Publish TF transforms                          *
        *****************************************************************/
        for (auto& kv : foundObjects) {

            setROSTransform(kv.second, 
                            transform);

            br.sendTransform(
                    tf::StampedTransform(transform, 
                                         ros::Time::now(), 
                                         "camera", 
                                         kv.first));
        }

        poseEstimationDuration = (1000 * (double) (cv::getTickCount()-startTickCount)) / cv::getTickFrequency();

        cout << "\x1b[K" 
            << "Total time: " << setw(5) << poseEstimationDuration + tagRecoDuration << "ms "
            << "(tag detection: " << setw(5) << tagRecoDuration << "ms, "
            << ", pose estim. + ROS export: " << setw(5) << poseEstimationDuration << "ms)" << endl << "\x1b[2F";
    }

    cout << endl << endl;
    tCapture.release();

    return 0;
}

