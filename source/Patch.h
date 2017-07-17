#pragma once

#include <vector>
#include <string>

#include "alpha_shape_polygons.h"
#include "border_definition.h"
#include "geometry_tools.h"
#include "io.h"
#include "segmentation.h"
#include "single_patch_recognition.h"
#include "multi_patches_recognition.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
enum Shape { plane, cylinder, cone };

class Patch
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_input_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_inlier_normals;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_remainder_normals;

	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
	std::vector<int> serial_number_boundary;
	Shape model;
	
public:
	int FixHoleAndFragmentation();
	void CheckBoundary();

};

Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list);

void FlattenInlierPointsBasedOnModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Shape model,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier, pcl::ModelCoefficients::Ptr coefficients);

void FillHole(Polygon_2 hole, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier, pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder, pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals);

void GiveBackOtherPiecesToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier, pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder, pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, Polygon_2 polygon_max_area);