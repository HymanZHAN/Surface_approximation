#include "patch.h"

Patch::Patch(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointIndices::Ptr inliers_parent, Shape model):cloud_input(input_cloud)
{
	// Use input_cloud to run RANSAC	
	// Obtain the initial inliers and corresponding properties

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	switch (model)
	{
	case Shape::plane:
		setSegmentationParametersForPlane(seg);

	case Shape::cylinder:
		setSegmentationParametersForPlane(seg);

	case Shape::cone:
		setSegmentationParametersForPlane(seg);
	}
	
	seg.setInputCloud(input_cloud);
	seg.segment(*Patch::inliers_current, *Patch::coefficients);// both "inliers" and "coefficients" are output
	cloud_inlier = extractCloud(input_cloud, inliers_current);
	cloud_remainder = extractCloud(input_cloud, inliers_current, true);

	inliers_current->indices.

	for (auto it = cloud_inlier->begin(); it != cloud_inlier->end(); ++it)
	{
		indices_map.insert(std::pair<int, int>(distance(cloud_inlier->begin(), it), ));
	};


}

Patch Patch::FixHoleAndFragmentation()
{
	if (model == Shape::plane)
	{

	}
	Polygon_list polygon_list = getAlphaShape(cloud_inlier);
	if (polygon_list.size() == 1)
	{
		return 0;
	}
	std::sort(polygon_list.begin(), polygon_list.end(),
		[](const Polygon_2& lhs, const Polygon_2& rhs) { return lhs.area() > rhs.area(); });
	Polygon_2 polygon_max_area = polygon_list.back();
	polygon_list.pop_back();
	Polygon_2 hole = CheckHoleInside(polygon_max_area, &polygon_list);
	GiveBackOtherPiecesToCloud(cloud_input, cloud_inlier, polygon_list);
	if (hole.size() == 0)
	{
		return 0;
	}
	else
	{
		FillHole(cloud_input, hole, cloud_inlier);
	}
	

}


Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list)
{
	float x, y;
	for (Polygon_list::iterator it = (*polygon_list).begin(); it != (*polygon_list).end(); ++it)
	{
		if ( CGAL::bounded_side_2(polygon_max_area.vertices_begin(), polygon_max_area.vertices_end(),
			Point((*it).vertex(0), (*it).vertex(1)), K()) != CGAL::ON_UNBOUNDED_SIDE)
		{

		}
	}
	
}

void Patch::CheckBoundary()
{

}

