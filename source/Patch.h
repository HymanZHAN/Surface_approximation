#pragma once

#include <vector>
#include <string>

#include "alpha_shape_polygons.h"
#include "border_definition.h"
#include "geometry_tools.h"
#include "io.h"
#include "segmentation.h"

class Patch
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder;
	pcl::PointIndices::Ptr inliers;
	std::vector<int> serial_number_boundary;
	enum shape { plane, cylinder, cone };
	pcl::ModelCoefficients::Ptr coefficients;
public:


};