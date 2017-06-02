#pragma once

#ifndef MESH_H
#define MESH_H

#include "read_parameters.h"
#include <pcl/features/normal_3d.h>


bool ReadSTLFile(const char *cfilename, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_filtered, 
	             pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals, 
	             pcl::PointCloud<pcl::PointXYZ>::Ptr *original_point_cloud_boundary);
//extern pcl::PointCloud<pcl::Normal>::Ptr all_cloud_normals (new pcl::PointCloud<pcl::Normal>);

#endif