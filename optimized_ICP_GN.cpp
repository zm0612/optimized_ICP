//
// Created by meng on 2021/3/25.
//
#include "optimized_ICP_GN.h"
#include "common.h"

OptimizedICPGN::OptimizedICPGN()
        : kdtree_flann_ptr_(new pcl::KdTreeFLANN<pcl::PointXYZI>) {
}

bool OptimizedICPGN::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) {
    target_cloud_ptr_ = target_cloud_ptr;
    kdtree_flann_ptr_->setInputCloud(target_cloud_ptr);//构建kdtree用于全局最近邻搜索
}

bool OptimizedICPGN::Match(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr,
                           const Eigen::Matrix4f &predict_pose,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr &transformed_source_cloud_ptr,
                           Eigen::Matrix4f &result_pose) {
    has_converge_ = false;
    source_cloud_ptr_ = source_cloud_ptr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f T = predict_pose;

    //Gauss-Newton's method solve ICP.
    for (unsigned int i = 0; i < max_iterations_; ++i) {
        pcl::transformPointCloud(*source_cloud_ptr, *transformed_cloud, T);
        Eigen::Matrix<float, 6, 6> Hessian = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> B = Eigen::Matrix<float, 6, 1>::Zero();

        for (unsigned int j = 0; j < transformed_cloud->size(); ++j) {
            const pcl::PointXYZI &origin_point = source_cloud_ptr->points[j];

            //删除距离为无穷点
            if (!pcl::isFinite(origin_point)) {
                continue;
            }

            const pcl::PointXYZI &transformed_point = transformed_cloud->at(j);
            std::vector<float> resultant_distances;
            std::vector<int> indices;
            //在目标点云中搜索距离当前点最近的一个点
            kdtree_flann_ptr_->nearestKSearch(transformed_point, 1, indices, resultant_distances);

            //舍弃那些最近点,但是距离大于最大对应点对距离
            if (resultant_distances.front() > max_correspond_distance_) {
                continue;
            }

            Eigen::Vector3f nearest_point = Eigen::Vector3f(target_cloud_ptr_->at(indices.front()).x,
                                                            target_cloud_ptr_->at(indices.front()).y,
                                                            target_cloud_ptr_->at(indices.front()).z);

            Eigen::Vector3f point_eigen(transformed_point.x, transformed_point.y, transformed_point.z);
            Eigen::Vector3f origin_point_eigen(origin_point.x, origin_point.y, origin_point.z);
            Eigen::Vector3f error = point_eigen - nearest_point;

            Eigen::Matrix<float, 3, 6> Jacobian = Eigen::Matrix<float, 3, 6>::Zero();
            //构建雅克比矩阵
            Jacobian.leftCols(3) = Eigen::Matrix3f::Identity();
            Jacobian.rightCols(3) = -T.block<3, 3>(0, 0) * Hat(origin_point_eigen);

            //构建海森矩阵
            Hessian += Jacobian.transpose() * Jacobian;
            B += -Jacobian.transpose() * error;
        }

        if (Hessian.determinant() == 0) {
            continue;
        }

        Eigen::Matrix<float, 6, 1> delta_x = Hessian.inverse() * B;

        T.block<3, 1>(0, 3) = T.block<3, 1>(0, 3) + delta_x.head(3);
        T.block<3, 3>(0, 0) *= SO3Exp(delta_x.tail(3)).matrix();

        if (delta_x.norm() < transformation_epsilon_) {
            has_converge_ = true;
            break;
        }

        // debug
//        std::cout << "i= " << i << "  norm delta x= " << delta_x.norm() << std::endl;
    }

    final_transformation_ = T;
    result_pose = T;
    pcl::transformPointCloud(*source_cloud_ptr, *transformed_source_cloud_ptr, result_pose);

    return true;
}

float OptimizedICPGN::GetFitnessScore(float max_range) const {
    float fitness_score = 0.0f;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*source_cloud_ptr_, *transformed_cloud_ptr, final_transformation_);

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    int nr = 0;

    for (unsigned int i = 0; i < transformed_cloud_ptr->size(); ++i) {
        kdtree_flann_ptr_->nearestKSearch(transformed_cloud_ptr->points[i], 1, nn_indices, nn_dists);

        if (nn_dists.front() <= max_range) {
            fitness_score += nn_dists.front();
            nr++;
        }
    }

    if (nr > 0)
        return fitness_score / static_cast<float>(nr);
    else
        return (std::numeric_limits<float>::max());
}

bool OptimizedICPGN::HasConverged() const {
    return has_converge_;
}

void OptimizedICPGN::SetMaxIterations(unsigned int iter) {
    max_iterations_ = iter;
}

void OptimizedICPGN::SetMaxCorrespondDistance(float max_correspond_distance) {
    max_correspond_distance_ = max_correspond_distance;
}

void OptimizedICPGN::SetTransformationEpsilon(float transformation_epsilon) {
    transformation_epsilon_ = transformation_epsilon;
}