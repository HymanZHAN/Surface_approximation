#pragma once

#ifndef IO_H
#define IO_H

#include <fstream>
#include <string>
#include <vector>
#include <windows.h>
#include <math.h>
#include <iostream> 
#include <string>
#include <fstream>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "alpha_shape_polygons.h"
#include "border_definition.h"
#include "cloud_visualizer.h"
#include "io.h"
#include "read_parameters.h"



extern std::string PATH_HEAD;

std::string exe_Path();

bool fillClouds(std::string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals1);

void outputCloudOnExcel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud);

void outputCloudOnPTS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud);

void outputCloudOnExcel(pcl::ModelCoefficients::Ptr coefficients, std::string name_cloud, std::string type);

void outputCloudAndNormalOnTXT(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, std::string name_cloud);

void outputCloudAndNormalOnTXT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, std::string name_cloud);

std::string exportCloudAsPTS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud);

std::string outputCloudOnTXT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name_cloud, int count);



//void outputCloudOnTXT_PtNumber(Polygon_2 polygon);

//void outputCloudOnTXT_PtNumber(std::vector<Alpha_shape_2::Point> alpha_shape_points);


#endif