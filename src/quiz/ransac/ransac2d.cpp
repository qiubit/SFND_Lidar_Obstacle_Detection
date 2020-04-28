/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <stdlib.h>
#include <algorithm>
#include <random>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	int bestInliers = 0;
	std::unordered_set<int> inliersCurrent;
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	const auto pointsTotal = cloud->size();
	if (pointsTotal < 2) {
		std::cout << "Point cloud is too small to contain a line" << std::endl;
	}
	std::vector<size_t> pointIndices;
	for (size_t i = 0; i < cloud->size(); ++i) {
		pointIndices.push_back(i);
	}

	// For max iterations
	for (int i = 0; i < maxIterations; ++i) {
		inliersCurrent.clear();

		const int planePtsNum = 3;
		std::vector<pcl::PointXYZ> sampledLinePts;
		std::random_shuffle(pointIndices.begin(), pointIndices.end());
		
		pcl::PointXYZ p1 = cloud->at(pointIndices[0]);
		pcl::PointXYZ p2 = cloud->at(pointIndices[1]);

		float A = p1.y - p2.y;
		float B = p2.x - p1.x;
		float C = p1.x*p2.y - p2.x*p1.y;

		float distanceDenom = sqrtf(A*A + B*B);
		for (size_t idx : pointIndices) {
			pcl::PointXYZ pCur = cloud->at(idx);
			float d = fabsf(A*pCur.x + B*pCur.y + C) / distanceDenom;
			// std::cout << "d: " << d << ", distanceTol: " << distanceTol << std::endl;
			if (d < distanceTol) {
				inliersCurrent.insert((int) idx);
			}
		}

		if (inliersCurrent.size() > bestInliers) {
			bestInliers = inliersCurrent.size();
			inliersResult = inliersCurrent;
		}
	}
	
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	int bestInliers = 0;
	std::unordered_set<int> inliersCurrent;
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	const auto pointsTotal = cloud->size();
	if (pointsTotal < 3) {
		std::cout << "Point cloud is too small to contain a plane" << std::endl;
	}
	std::vector<size_t> pointIndices;
	for (size_t i = 0; i < cloud->size(); ++i) {
		pointIndices.push_back(i);
	}

	// For max iterations
	for (int iter = 0; iter < maxIterations; ++iter) {
		inliersCurrent.clear();

		std::vector<pcl::PointXYZ> sampledLinePts;
		std::random_shuffle(pointIndices.begin(), pointIndices.end());
		
		pcl::PointXYZ p1 = cloud->at(pointIndices[0]);
		pcl::PointXYZ p2 = cloud->at(pointIndices[1]);
		pcl::PointXYZ p3 = cloud->at(pointIndices[2]);

		// vector<float> v1{p2.x-p1.x, p2.y-p1.y, p2.z-p1.z};
		// vector<float> v2{p3.x-p1.x, p3.y-p1.y, p3.z-p1.z};

		float i = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		float j = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		float k = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*p1.x + j*p1.y + k*p1.z);

		float distanceDenom = sqrtf(A*A + B*B + C*C);
		for (size_t idx : pointIndices) {
			pcl::PointXYZ pCur = cloud->at(idx);
			float d = fabsf(A*pCur.x + B*pCur.y + C*pCur.z + D) / distanceDenom;
			if (d < distanceTol) {
				inliersCurrent.insert((int) idx);
			}
		}

		if (inliersCurrent.size() > bestInliers) {
			bestInliers = inliersCurrent.size();
			inliersResult = inliersCurrent;
		}
	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
