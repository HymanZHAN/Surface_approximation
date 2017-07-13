#pragma once

#include <vector>
#include <string>

#include "alpha_shape_polygons.h"
#include "border_definition.h"
#include "geometry_tools.h"
#include "io.h"
#include "segmentation.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
enum class Shape { plane, cylinder, cone };

class Patch
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder;
	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
	std::vector<int> serial_number_boundary;
	Shape model;
	
public:
	Patch::Patch();
	Patch FixHoleAndFragmentation();
	void CheckBoundary();

};

Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list);