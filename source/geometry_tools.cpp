#include "geometry_tools.h"


Eigen::Matrix3f buildTransformMatrixFromAxis(Eigen::Vector3f x_axis, Eigen::Vector3f y_axis, Eigen::Vector3f z_axis)
{
	using namespace Eigen;

	Matrix3f transform_matrix;
	transform_matrix << x_axis, y_axis, z_axis;
	//std::cout << "Transformation matrix: "  << std::endl;
	//std::cout << transform_matrix << std::endl;

	return transform_matrix;
}

Eigen::Vector3f transformPointByMatrix(Eigen::Matrix3f transform_matrix, Eigen::Vector3f point_coord)
{
	return transform_matrix*point_coord;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud, Eigen::Matrix3f transform_matrix)
{
	const int num_nodes = patch_cloud->points.size();
	Eigen::Vector3f point, transformed_point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_patch_cloud (new pcl::PointCloud<pcl::PointXYZ>);


	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(patch_cloud->at(i).getArray3fMap());
		transformed_point = transform_matrix*point;
		// OR transformed_point = transformPointByMatrix(transform_matrix, point);
		transformed_patch_cloud->push_back (pcl::PointXYZ (transformed_point[0], transformed_point[1], transformed_point[2]));
	}

	return transformed_patch_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudByMatrix_cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud,Eigen::Matrix3f transform_matrix, std::vector<float> cyl_param)
{
	const int num_nodes = patch_cloud->points.size();
	Eigen::Vector3f point, transformed_point, point_on_axis(cyl_param[0], cyl_param[1], cyl_param[2]), translation_vector;
	translation_vector = transform_matrix.inverse()*point_on_axis;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_patch_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::size_t i = 0; i < num_nodes; ++i)
	{
		point = Eigen::Vector3f(patch_cloud->at(i).getArray3fMap());
		transformed_point = transform_matrix.inverse()*point-translation_vector;
		transformed_patch_cloud->push_back (pcl::PointXYZ (transformed_point[0], transformed_point[1], transformed_point[2]));
	}
	return transformed_patch_cloud;
}

template <typename Point> void list_coordinates(Point const& p) 
{ 
    using boost::geometry::get; 
    
    std::cout << "x = " << get<0>(p) << " y = " << get<1>(p) << std::endl; 

} 

void TransformCloseToCoordinateSystem(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud, pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals)
{
	//pcl::ModelCoefficients::Ptr plane;
	//ThreePointsToPlane((cloud->at(0))., cloud->at(1), cloud->at(2), plane);
	// Ground plane estimation:
	//Eigen::VectorXf ground_coeffs;
	//ground_coeffs.resize(4);
	//std::vector<int> clicked_points_indices;
	//for (int i = 0; i < 3; i++)
	//{
	//	clicked_points_indices.push_back(i);
	//}
	//pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(*cloud);
	//model_plane.computeModelCoefficients(clicked_points_indices, ground_coeffs);
	//std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;
	pcl::ModelCoefficients::Ptr transform_coefficients(new pcl::ModelCoefficients());
	transform_coefficients->values.push_back((*cloud)->at(0).x - 1000);
	transform_coefficients->values.push_back((*cloud)->at(0).y - 1000);
	transform_coefficients->values.push_back((*cloud)->at(0).z - 1000);
	transform_coefficients->values.push_back(10);
	transform_coefficients->values.push_back(20);
	transform_coefficients->values.push_back(30);
	transform_coefficients->values.push_back(0.8);
	//for (int i = 0; i < 4; i++)
	//{
	//	transform_coefficients->values.push_back(ground_coeffs(i));
	//}


	//transform coordinates of point cloud by the method of transforming cone, 
	//appex of which is the first point in point cloud, central axis of which is normal of the plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_cloud = transformConicalPatchPoints(*cloud, transform_coefficients->values);
	(*cloud).swap(transformed_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_normals(new pcl::PointCloud<pcl::PointXYZ>);
	int size_normals = (*cloud_normals)->size();
	for (int i = 0; i < size_normals; i++)
	{
		transformed_cloud_normals->push_back(pcl::PointXYZ((*cloud_normals)->at(i).normal_x, (*cloud_normals)->at(i).normal_y, (*cloud_normals)->at(i).normal_z));
	}
	transformed_cloud_normals = transformConicalPatchPoints(transformed_cloud_normals, transform_coefficients->values);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_new(new pcl::PointCloud<pcl::Normal>);
	for (int i = 0; i < size_normals; i++)
	{
		cloud_normals_new->push_back(pcl::Normal(transformed_cloud_normals->at(i).x, transformed_cloud_normals->at(i).y, transformed_cloud_normals->at(i).z));
	}
	(*cloud_normals).swap(cloud_normals_new);


	//pcl::PointCloud<pcl::Normal>::Ptr transformed_cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//transformed_cloud_normals = transformPlanarPatchPoints(cloud_normal, plane_coefficients->values);
	//(*cloud_normal).swap(*transformed_cloud_normals);
}

void TransformCloseToCoordinateSystem(pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud)
{
	//float x0 = (*cloud)->at(0).x / 2, y0 = (*cloud)->at(0).y / 2, z0 = (*cloud)->at(0).z / 2;
	float x0 = (*cloud)->at(0).x, y0 = (*cloud)->at(0).y, z0 = (*cloud)->at(0).z;
	int size = (*cloud)->size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < size; i++)
	{
		cloud_new->push_back(pcl::PointXYZ((*cloud)->at(i).x - x0, (*cloud)->at(i).y - y0, (*cloud)->at(i).z - z0));
	}
	(*cloud).swap(cloud_new);
}



