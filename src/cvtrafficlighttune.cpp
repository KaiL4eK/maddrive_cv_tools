#include <iostream>
using namespace std;

#include <ros/ros.h>
#include <ros/console.h>
using namespace ros;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include <maddrive_cv_tools/cvtrafficlightconfig.h>

void onMouse(int evt, int x, int y, int flags, void* param);
void update( void );

const int   window_width            = 320;
const int   window_height           = 240;

Point       roi_rect_ul;
Point       roi_rect_lr;

int         Hmin = 0;
int         Hmax = 180;

int         Smin = 0;
int         Smax = 255;

int         Vmin = 0;
int         Vmax = 255;

Mat         original_image;
Mat         roi_image;
Mat         filtered_image;

const string original_window_name  = "tune";
const string thresh_window_name    = "thresh";
const string filtered_window_name  = "filter";

int main(int argc, char **argv)
{
    ros::init( argc, argv, "traffic_light_tuner" );

    namedWindow(original_window_name);
    namedWindow(thresh_window_name);
    namedWindow(filtered_window_name);

    setMouseCallback(original_window_name, onMouse, NULL);

    moveWindow( original_window_name, 0, 0 );
    moveWindow( thresh_window_name, window_width, 0 );
    moveWindow( filtered_window_name, window_width * 2, 0 );

    createTrackbar("Hmin", filtered_window_name, &Hmin, 180, NULL);
    createTrackbar("Hmax", filtered_window_name, &Hmax, 180, NULL);
    createTrackbar("Smin", filtered_window_name, &Smin, 255, NULL);
    createTrackbar("Smax", filtered_window_name, &Smax, 255, NULL);
    createTrackbar("Vmin", filtered_window_name, &Vmin, 255, NULL);
    createTrackbar("Vmax", filtered_window_name, &Vmax, 255, NULL);

    NodeHandle nh("~");
    string image_filepath;

    nh.getParam("image", image_filepath);

    while ( 1 )
    {
        original_image = imread(image_filepath, CV_LOAD_IMAGE_COLOR);
        if( !original_image.data )
        {
            ROS_ERROR_STREAM ( "Could not open or find the image: " << image_filepath );
            return -1;
        }

        resize( original_image, original_image, Size(window_width, window_height) );

        update();

//        cout << "Key: " << (waitKey(0) & 0xFF) << endl;
        char key = waitKey(30);
        if ( key == 10 || key == 27 )
            break;
    }

    return 0;
}

void onMouse(int evt, int x, int y, int flags, void* param)
{
    if(evt == CV_EVENT_LBUTTONDOWN) {
        cout << "Clicked L " << x << " / " << y << endl;
        roi_rect_ul.x = x;
        roi_rect_ul.y = y;
    } else if (evt == CV_EVENT_RBUTTONDOWN) {
        cout << "Clicked R " << x << " / " << y << endl;
        roi_rect_lr.x = x;
        roi_rect_lr.y = y;
    } else
        return;

    CvTrafficLightConfig::getInstance().setROIRectangle(
                roi_rect_ul.x * 1. / window_width,
                roi_rect_ul.y * 1. / window_height,
                roi_rect_lr.x * 1. / window_width,
                roi_rect_lr.y * 1. / window_height
             );
}

void update( void )
{
    Mat draw_original_image = original_image.clone();

    vector<double> &roi = CvTrafficLightConfig::getInstance().getROIRectangle();
    roi_rect_ul = Point( roi[0] * window_width, roi[1] * window_height );
    roi_rect_lr = Point( roi[2] * window_width, roi[3] * window_height );

    Rect    roi_rect( roi_rect_ul, roi_rect_lr );

    rectangle( draw_original_image, roi_rect, Scalar(255, 0, 0) );
    circle( draw_original_image, roi_rect_ul, 3, Scalar(0, 255, 0), -1 );
    circle( draw_original_image, roi_rect_lr, 3, Scalar(0, 0, 255), -1 );
    imshow( original_window_name, draw_original_image );

    if ( roi_rect.area() )
    {
        roi_image = original_image(roi_rect);
        resize( roi_image, roi_image, Size(window_width, window_height) );
        imshow( thresh_window_name, roi_image );
    }

    cvtColor( roi_image, filtered_image, CV_BGR2HSV );
    inRange( filtered_image, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), filtered_image );
    imshow( filtered_window_name, filtered_image );
}
