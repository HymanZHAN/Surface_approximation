#include "patch.h"
#include "alpha_shape_polygons.h"

int Patch::FixHoleAndFragmentation()
{
	if (model == plane)
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

