#include <maddrive_cv_tools/cvtrafficlightconfig.h>

CvTrafficLightConfig::CvTrafficLightConfig()
{
    parameter_node_handle = ros::NodeHandle( "md_cv_traffic_light" );

    roi_rectangle   = vector<double> (4, 0.0);
    colorspace_min  = vector<int> (3, 0);
    colorspace_max  = vector<int> (3, 0);

    roi_param_name              = "roi";
    colorspace_max_param_name   = "colorspace_max";
    colorspace_min_param_name   = "colorspace_min";

    parameter_node_handle.getParam(roi_param_name, roi_rectangle);
    parameter_node_handle.getParam(colorspace_min_param_name, colorspace_min);
    parameter_node_handle.getParam(colorspace_max_param_name, colorspace_max);
}
