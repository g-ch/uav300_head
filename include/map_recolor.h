#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void mapRecolor(pcl::PointCloud<pcl::PointXYZ>::Ptr original_map, pcl::PointCloud<pcl::PointXYZRGB>::Ptr recolored_map, float ground_height=0.4, float max_height=3.0){
    for(int index=0; index < original_map->width; index++){
        pcl::PointXYZRGB point_this;
        point_this.x = original_map->points[index].x;
        point_this.y = original_map->points[index].y;
        point_this.z = original_map->points[index].z;

        if(point_this.z < ground_height){
            point_this.r = 230;
            point_this.b = 120;
            point_this.g = 30;
        } else if(point_this.z < max_height){
            int value = floor(point_this.z / max_height * 240); // Mapping 0~1.0 to 0~240
            value = value > 240 ? 240 : value;
            int section_devide_num = 120;
            int section = value / section_devide_num;
            float float_key = (value % section_devide_num) / (float)section_devide_num * 255;
            int key = floor(float_key);
            int nkey = 255 - key;

            switch(section){
                case 0:
                    point_this.b = key;
                    point_this.g = 255;
                    point_this.r = 0;
                    break;
                case 1:
                    point_this.b = 255;
                    point_this.g = nkey;
                    point_this.r = 0;
                    break;
                default:
                    point_this.r = 0;
                    point_this.b = 240;
                    point_this.g = 30;
            }

        } else{
            point_this.r = 0;
            point_this.b = 240;
            point_this.g = 30;
        }
        recolored_map->points.push_back(point_this);
    }
}