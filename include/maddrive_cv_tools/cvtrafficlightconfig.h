#ifndef CVTRAFFICLIGHTCONFIG_H
#define CVTRAFFICLIGHTCONFIG_H

#include <ros/ros.h>

using namespace std;

class CvTrafficLightConfig
{
private:
    CvTrafficLightConfig();
    ~CvTrafficLightConfig() {  }

    CvTrafficLightConfig(CvTrafficLightConfig const&);
    CvTrafficLightConfig& operator= (CvTrafficLightConfig const&);

    ros::NodeHandle         parameter_node_handle;

    vector<double>  roi_rectangle;
    string          roi_param_name;

    vector<int>     colorspace_min;
    string          colorspace_min_param_name;

    vector<int>     colorspace_max;
    string          colorspace_max_param_name;

public:
    static CvTrafficLightConfig& getInstance()
    {
        static CvTrafficLightConfig s;
        return s;
    }

    void setROIRectangle(double ul_x, double ul_y, double lr_x, double lr_y)
    {
        roi_rectangle.at(0) = ul_x;
        roi_rectangle.at(1) = ul_y;
        roi_rectangle.at(2) = lr_x;
        roi_rectangle.at(3) = lr_y;

        parameter_node_handle.setParam(roi_param_name, roi_rectangle);
    }

    vector<double> &getROIRectangle( void )
    {
        return roi_rectangle;
    }
};

#endif // CVTRAFFICLIGHTCONFIG_H
