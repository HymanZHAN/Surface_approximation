#pragma once

#ifndef TWO_PATCHES_RECOGNITION_H
#define TWO_PATCHES_RECOGNITION_H


#include "read_parameters.h"
#include "border_definition.h"
#include "segmentation.h"
#include "io.h"
#include "single_patch_recognition.h"





int TwoPatchesPartition(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, int threshold_inliers,
                     	int *patch_count, Eigen::MatrixXf **patch_data, pcl::PointCloud<pcl::PointXYZ>::Ptr **sourceClouds);




#endif