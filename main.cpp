//
// Created by Zhang Zhimeng on 22-8-26.
//

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

#include "optimized_ICP_GN.h"

using namespace std;

int main() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_opti_transformed_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_svd_transformed_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::io::loadPCDFile("../data/room_scan2.pcd", *cloud_target_ptr);
    pcl::io::loadPCDFile("../data/room_scan1.pcd", *cloud_source_ptr);

    Eigen::Matrix4f T_predict, T_final;
    T_predict.setIdentity();

    T_predict << 0.765, 0.643, -0.027, -1.472,
            -0.644, 0.765, -0.023, 1.366,
            0.006, 0.035, 0.999, -0.125,
            0.0, 0.0, 0.0, 1.0;

    std::cout << "Wait, matching..." << std::endl;

    // =======================   optimized icp   =======================
    OptimizedICPGN icp_opti;
    icp_opti.SetTargetCloud(cloud_target_ptr);
    icp_opti.SetTransformationEpsilon(1e-4);
    icp_opti.SetMaxIterations(30);
    icp_opti.SetMaxCorrespondDistance(0.5);
    icp_opti.Match(cloud_source_ptr, T_predict, cloud_source_opti_transformed_ptr, T_final);
    std::cout << "============== Optimized ICP =================" << std::endl;
    std::cout << "T final:\n" << T_final << std::endl;
    std::cout << "fitness score: " << icp_opti.GetFitnessScore() << std::endl;
    // =======================   optimized icp   =======================


    // =======================   svd icp   =======================
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_svd;
    icp_svd.setInputTarget(cloud_target_ptr);
    icp_svd.setInputSource(cloud_source_ptr);
    icp_svd.setMaxCorrespondenceDistance(0.5);
    icp_svd.setMaximumIterations(30);
    icp_svd.setEuclideanFitnessEpsilon(1e-4);
    icp_svd.setTransformationEpsilon(1e-4);
    icp_svd.align(*cloud_source_svd_transformed_ptr, T_predict);
    std::cout << "\n============== SVD ICP =================" << std::endl;
    std::cout << "T final: \n" << icp_svd.getFinalTransformation() << std::endl;
    std::cout << "fitness score: " << icp_svd.getFitnessScore() << std::endl;
    // =======================   svd icp   =======================

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->initCameraParameters();

    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Optimized ICP", 10, 10, "optimized icp", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_opti_color(
            cloud_source_opti_transformed_ptr,
            255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud_source_opti_transformed_ptr, source_opti_color, "source opti cloud",
                                          v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_color_0(cloud_target_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZI>(cloud_target_ptr, target_color_0, "target cloud1", v2);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
    viewer->addText("SVD ICP", 10, 10, "svd icp", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_color_1(cloud_target_ptr, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZI>(cloud_target_ptr, target_color_1, "target cloud2", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_svd_color(cloud_source_svd_transformed_ptr,
                                                                                      0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud_source_svd_transformed_ptr, source_svd_color, "source svd cloud",
                                          v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source opti cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source svd cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target cloud2");
    viewer->addCoordinateSystem(1.0);

    viewer->setCameraPosition(0, 0, 20, 0, 10, 10, v1);
    viewer->setCameraPosition(0, 0, 20, 0, 10, 10, v2);

    viewer->spin();

    return 0;
}