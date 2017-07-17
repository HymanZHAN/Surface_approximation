#include "patch.h"
#include "alpha_shape_polygons.h"

int Patch::FixHoleAndFragmentation()
{
	//flatten
	pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	FlattenInlierPointsBasedOnModel(&flattened_cloud, Patch::model, Patch::cloud_inlier, Patch::coefficients);

	Polygon_list polygon_list = getAlphaShape(flattened_cloud);

	if (polygon_list.size() == 1)  // there is one piece without hole inside
	{
		return 0;
	}

	//sort polygons based on area
	std::sort(polygon_list.begin(), polygon_list.end(),
		[](const Polygon_2& lhs, const Polygon_2& rhs) { return lhs.area() > rhs.area(); });
	Polygon_2 polygon_max_area = polygon_list.back();
	polygon_list.pop_back();

	//check if there is a hole inside the polygon with maximun area
	Polygon_2 hole = CheckHoleInside(polygon_max_area, &polygon_list);
    
	//when polygon_list.size()==0 at this moment, there is only one piece with one hole inside
	//when polygon_list.size()> 0, there are other pieces which need to be given back to cloud_input
	if (polygon_list.size() > 0) 
	{
		GiveBackOtherPiecesToCloud(&(this->cloud_inlier), &(this->cloud_inlier_normals), &(this->cloud_remainder), 
			                       &(this->cloud_remainder_normals), flattened_cloud, polygon_max_area);
	}
	
	if (hole.size() == 0)
	{
		return 0;
	}
	else
	{
		FillHole(hole, flattened_cloud, &(this->cloud_inlier), &(this->cloud_inlier_normals), &(this->cloud_remainder), &(this->cloud_remainder_normals));
	}
	
	return 0;
}


void FlattenInlierPointsBasedOnModel(pcl::PointCloud<pcl::PointXYZ>::Ptr *flattened_cloud, Shape model,
	                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inlier,pcl::ModelCoefficients::Ptr coefficients)
{
	if (model == plane)
	{

		*flattened_cloud = transformPlanarPatchPoints(cloud_inlier, coefficients->values);
	}
	else
	{
		if (model == cylinder)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cyl_patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			transformed_cyl_patch_cloud = transformCylindricalPatchPoints(cloud_inlier, coefficients->values);
			*flattened_cloud = flattenCylindricalPatch(transformed_cyl_patch_cloud, coefficients->values);
		}
		else
		{
			if (model == cone)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cone_patch_cloud = transformConicalPatchPoints(cloud_inlier, coefficients->values);
				*flattened_cloud = FlattenCloud(transformed_cone_patch_cloud, coefficients->values);
			}
		}
	}
}

Polygon_2 CheckHoleInside(Polygon_2 polygon_max_area, Polygon_list *polygon_list)
{
	for (Polygon_list::iterator it = (*polygon_list).begin(); it != (*polygon_list).end(); ++it)
	{
		if (CGAL::bounded_side_2(polygon_max_area.vertices_begin(), polygon_max_area.vertices_end(),
			Point((*((*it).vertices_begin())).x(), (*((*it).vertices_begin())).y()), K()) != CGAL::ON_UNBOUNDED_SIDE)
		{
			(*polygon_list).remove(*it);
			return *it;

		}
	}
	Polygon_2 hole;
	return hole;
}

void GiveBackOtherPiecesToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier, pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals,
	                            pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder, pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals,
	                            pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, Polygon_2 polygon_max_area)
{
	
	pcl::PointIndices real_indices = GetIndicesOfPointsInsideAndOnPolygon(flattened_cloud, polygon_max_area);
	
	
	//update cloud data
	pcl::PointIndices::Ptr real_inliers(new pcl::PointIndices(real_indices));

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr temp_cloud_normals(new pcl::PointCloud<pcl::Normal>);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(*cloud_inlier);
	extract.setIndices(real_inliers);
	extract.setNegative(true);
	extract.filter(*temp_cloud);
	extract.setNegative(false);
	extract.filter(**cloud_inlier);
	

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	extract_normals.setInputCloud(*cloud_inlier_normals);
	extract_normals.setIndices(real_inliers);
	extract_normals.setNegative(true);
	extract_normals.filter(*temp_cloud_normals);
	extract_normals.setNegative(false);
	extract_normals.filter(**cloud_inlier_normals);
	
	(**cloud_remainder) += (*temp_cloud);
	(**cloud_remainder_normals) += (*temp_cloud_normals);

}

void FillHole(Polygon_2 hole, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_inlier,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_inlier_normals, pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_remainder,
	pcl::PointCloud<pcl::Normal>::Ptr *cloud_remainder_normals)
{
	pcl::PointIndices indices_hole = GetIndicesOfPointsOnPolygon(flattened_cloud, hole);

	//get cloud of hole
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hole(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_hole = GetCloudBasedOnIndices(indices_hole, *cloud_inlier);

	//get the plane comprised by the hole throught RANSAC method
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers_plane_hole(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coefficients_plane_hole(new pcl::ModelCoefficients());
	setSegmentationParametersForPlane(seg);
	seg.setInputCloud(cloud_hole);
	seg.segment(*inliers_plane_hole, *coefficients_plane_hole);

	//Project cloud_remainder on the plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder_projected(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_remainder_projected = GetProjectedCloud(plane, coefficients_plane_hole, *cloud_remainder);

	//move the points inside the polygon from cloud_remainder to cloud_inlier
	pcl::PointIndices indices_points_inside_hole = GetIndicesOfPointsInsidePolygon(cloud_remainder_projected, hole);
	MovePartsFromCloudToCloud(indices_points_inside_hole, &cloud_remainder, &cloud_remainder_normals, &cloud_inlier, &cloud_inlier_normals);

}



pcl::PointIndices GetIndicesOfPointsInsideAndOnPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon)
{
	pcl::PointIndices indices;
	int size_flattened_cloud = cloud->points.size();
	for (int i = 0; i < size_flattened_cloud; i++)
	{
		if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(),
			Point(cloud->at(i).x, cloud->at(i).z), K()) != CGAL::ON_UNBOUNDED_SIDE)
		{
			indices.indices.push_back(i);
		}
	}
	return indices;
}

pcl::PointIndices GetIndicesOfPointsOnPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon)
{
	pcl::PointIndices indices;
	int size_flattened_cloud = cloud->points.size();
	for (int i = 0; i < size_flattened_cloud; i++)
	{
		if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(),
			Point(cloud->at(i).x, cloud->at(i).z), K()) == CGAL::ON_BOUNDARY)
		{
			indices.indices.push_back(i);
		}
	}
	return indices;
}

pcl::PointIndices GetIndicesOfPointsInsidePolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Polygon_2 polygon)
{
	pcl::PointIndices indices;
	int size_flattened_cloud = cloud->points.size();
	for (int i = 0; i < size_flattened_cloud; i++)
	{
		if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(),
			Point(cloud->at(i).x, cloud->at(i).z), K()) == CGAL::ON_BOUNDED_SIDE)
		{
			indices.indices.push_back(i);
		}
	}
	return indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudBasedOnIndices(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices(indices));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_indices(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(original_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_indices);
	return cloud_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GetProjectedCloud(Shape shape, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	if (shape == plane)
	{
		proj.setModelType(pcl::SACMODEL_PLANE);
	}
	else
	{
		if (shape == cylinder)
		{
			proj.setModelType(pcl::SACMODEL_CYLINDER);
		}
		else
		{
			proj.setModelType(pcl::SACMODEL_CONE);
		}
	}
	
	proj.setInputCloud(original_cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	return cloud_projected;
}

void MovePartsFromCloudToCloud(pcl::PointIndices indices, pcl::PointCloud<pcl::PointXYZ>::Ptr **cloud_from,pcl::PointCloud<pcl::Normal>::Ptr **normal_from, 
	                           pcl::PointCloud<pcl::PointXYZ>::Ptr **cloud_to, pcl::PointCloud<pcl::Normal>::Ptr **normal_to)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_parts(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining_in_cloud_from(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices(indices));

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(**cloud_from);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_parts);
	extract.setNegative(true);
	extract.filter(*cloud_remaining_in_cloud_from);
	(**cloud_from).swap(cloud_remaining_in_cloud_from);

	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::Normal>::Ptr normal_parts(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_remaining_in_normal_from(new pcl::PointCloud<pcl::Normal>);
	extract_normals.setInputCloud(**normal_from);
	extract_normals.setIndices(inliers);
	extract_normals.setNegative(false);
	extract_normals.filter(*normal_parts);
	extract_normals.setNegative(true);
	extract_normals.filter(*normal_remaining_in_normal_from);
	(**normal_from).swap(normal_remaining_in_normal_from);

	(***cloud_to) += (*cloud_parts);
	(***normal_to) += (*normal_parts);

}



void Patch::CheckBoundary()
{

}

