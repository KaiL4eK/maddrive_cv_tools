#include <maddrive_cv_tools/cvtrafficlightconfig.h>

CvTrafficLightConfig::CvTrafficLightConfig()
{
    parameter_node_handle = ros::NodeHandle( "/md_cv_traffic_light" );

    roi_rectangle       = vector<double> (4, 0.0);
    colorspace_limits   = vector<int> (6, 0);
    colorspace_limits.at(3) = 180;
    colorspace_limits.at(4) = 255;
    colorspace_limits.at(5) = 255;

    roi_param_name                  = "roi";
    colorspace_limits_param_name    = "colorspace_limits";

    parameter_node_handle.getParam(roi_param_name, roi_rectangle);
    parameter_node_handle.getParam(colorspace_limits_param_name, colorspace_limits);
}

void CvTrafficLightConfig::setROIRectangle(double ul_x, double ul_y, double lr_x, double lr_y)
{
    roi_rectangle.at(0) = ul_x;
    roi_rectangle.at(1) = ul_y;
    roi_rectangle.at(2) = lr_x;
    roi_rectangle.at(3) = lr_y;

    parameter_node_handle.setParam(roi_param_name, roi_rectangle);
}

vector<double> &CvTrafficLightConfig::getROIRectangle( void )
{
    return roi_rectangle;
}

void CvTrafficLightConfig::setColorspaceLimits(
        int h_min, int s_min, int v_min, int h_max, int s_max, int v_max)
{
    colorspace_limits.at(0) = h_min;
    colorspace_limits.at(1) = s_min;
    colorspace_limits.at(2) = v_min;
    colorspace_limits.at(3) = h_max;
    colorspace_limits.at(4) = s_max;
    colorspace_limits.at(5) = v_max;

    parameter_node_handle.setParam(colorspace_limits_param_name, colorspace_limits);
}

vector<int> &CvTrafficLightConfig::getColorspaceLimits( void )
{
    return colorspace_limits;
}
