//
// Created by hjl on 2021/10/22.
//

#include <traversability_analysis/terrain_map_empty.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "terrain_analysis_node");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    traversability_analysis::TerrainMapEmpty  traversability_analysis(nh, nh_private); 
    ros::spin();
    return 0;
}