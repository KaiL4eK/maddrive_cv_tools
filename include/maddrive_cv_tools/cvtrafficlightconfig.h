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

    // Keeped as [ulx, uly, lrx, lry]
    vector<double>  roi_rectangle;
    string          roi_param_name;

    // Keeped as [hmin, smin, vmin, hmax, smax, vmax]
    vector<int>     colorspace_limits;
    string          colorspace_limits_param_name;


public:
    static CvTrafficLightConfig& getInstance()
    {
        static CvTrafficLightConfig s;
        return s;
    }

    void setROIRectangle(double ul_x, double ul_y, double lr_x, double lr_y);
    vector<double> &getROIRectangle( void );

    void setColorspaceLimits(int h_min, int s_min, int v_min, int h_max, int s_max, int v_max);
    vector<int> &getColorspaceLimits( void );

    static void save_parameters_to_file( void ) { system("rosrun maddrive_cv_tools save_traffic_light_params.sh"); }
    static void load_parameters_from_file( void ) { system("rosrun maddrive_cv_tools load_traffic_light_params.sh"); }
};

#endif // CVTRAFFICLIGHTCONFIG_H
