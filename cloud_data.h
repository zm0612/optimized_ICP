//
// Created by meng on 2020/12/10.
//

#ifndef TRL_SLAM_CLOUD_DATA_H
#define TRL_SLAM_CLOUD_DATA_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CloudData {
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

public:
    CloudData() : cloud_ptr(new CLOUD()) {
    }

public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};

#endif //TRL_SLAM_CLOUD_DATA_H
