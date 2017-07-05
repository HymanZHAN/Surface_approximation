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
	pcl::ModelCoefficients::Ptr coefficients;
	std::vector<int> serial_number_boundary;
	enum Shape { plane, cylinder, cone };
	
public:
	int FixHoleAndFragmentation();
	void CheckBoundary();

};

int Patch::FixHoleAndFragmentation()
{
	Polygon_list polygon_list = getAlphaShape(cloud_inlier);
	if (polygon_list.size() == 1)
	{
		return 0;
	}
	std::sort(polygon_list.begin(), polygon_list.end(),
		[](const Polygon_2& lhs, const Polygon_2& rhs) { return lhs.area() > rhs.area(); });
	Polygon_2 polygon_max_area = polygon_list.back();
	polygon_list.pop_back();
	Polygon_2 polygon_max_area_2 = polygon_list.back();
	polygon_list.pop_back();


}


void Patch::CheckBoundary()
{

}

