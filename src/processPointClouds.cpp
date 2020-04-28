// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    pcl::CropBox<PointT> regionCropBox(true);
    regionCropBox.setMin(minPoint);
    regionCropBox.setMax(maxPoint);
    regionCropBox.setInputCloud(cloudFiltered);
    regionCropBox.filter(*cloudRegion);

    std::vector<int> indices;
    Eigen::Vector4f roofMin{-1.4, -1.4, -1.0, 1.};
    Eigen::Vector4f roofMax{0.0, 1.4, -0.1, 1.};

    pcl::CropBox<PointT> roofCropBox(true);
    roofCropBox.setMin(roofMin);
    roofCropBox.setMax(roofMax);
    roofCropBox.setInputCloud(cloudRegion);
    roofCropBox.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr inliersCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outliersCloud(new pcl::PointCloud<PointT>());

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inliersCloud);
    std::cerr << "PointCloud representing the planar component: " << inliersCloud->width * inliersCloud->height << " data points." << std::endl;

    extract.setNegative(true);
    extract.filter(*outliersCloud);
    std::cerr << "PointCloud representing obstacles: " << outliersCloud->width * outliersCloud->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inliersCloud, outliersCloud);
    return segResult;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	int bestInliers = 0;
	std::unordered_set<int> inliersCurrent;
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	const auto pointsTotal = cloud->size();

	if (pointsTotal < 3) {
		std::cout << "Point cloud is too small to contain a plane" << std::endl;
        return inliersResult;
	}

	for (int iter = 0; iter < maxIterations; ++iter) {
		inliersCurrent.clear();

		std::unordered_set<int> inliers;
        while (inliers.size() < 3) {
            inliers.insert(rand()%(cloud->points.size()));
        }

        auto itr = inliers.begin();
		PointT p1 = cloud->at(*itr);
        ++itr;
		PointT p2 = cloud->at(*itr);
        ++itr;
		PointT p3 = cloud->at(*itr);

		float i = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		float j = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		float k = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*p1.x + j*p1.y + k*p1.z);

		float distanceDenom = sqrtf(A*A + B*B + C*C);
		for (int i = 0; i < cloud->points.size(); ++i) {
			PointT pCur = cloud->at(i);
			float d = fabsf(A*pCur.x + B*pCur.y + C*pCur.z + D) / distanceDenom;
			if (d < distanceTol) {
				inliersCurrent.insert(i);
			}
		}

		if (inliersCurrent.size() > bestInliers) {
			bestInliers = inliersCurrent.size();
			inliersResult = inliersCurrent;
		}
	}
	
	return inliersResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool usePcl)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    if (!usePcl)
    {
        std::unordered_set<int> inliersSet = Ransac(cloud, maxIterations, distanceThreshold);
        for (int inlierId : inliersSet) {
            inliers->indices.push_back(inlierId);
        }
    }
    else
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for given dataset." << std::endl;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(std::shared_ptr<KdTree> tree, const std::vector<std::vector<float>>& points, std::vector<int> &cluster, std::vector<bool> &processed, float distanceTol, int curPtId)
{
	if (processed[curPtId])
		return;

	processed[curPtId] = true;
	cluster.push_back(curPtId);

	std::vector<float> curPt = points[curPtId];
	std::vector<int> nearby = tree->search(curPt, distanceTol);

	for (int nextPtId : nearby) {
		clusterHelper(tree, points, cluster, processed, distanceTol, nextPtId);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, std::shared_ptr<KdTree> tree, float distanceTol)
{
	std::vector<bool> processed(points.size());
	for (int i = 0; i < processed.size(); ++i) {
		processed[i] = false;
	}

	std::vector<std::vector<int>> clusters;

	for (int i = 0; i < processed.size(); ++i) {
		if (processed[i])
			continue;

		std::vector<int> cluster;
		clusterHelper(tree, points, cluster, processed, distanceTol, i);
		clusters.push_back(cluster);
	}
 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, bool usePcl)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    if (!usePcl)
    {
        std::shared_ptr<KdTree> kdTree{new KdTree};
        std::vector<std::vector<float>> pts;

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;
            std::vector<float> pt{x, y, z};
            pts.push_back(pt);
            kdTree->insert(pt, i);
        }

        std::vector<std::vector<int>> clustersPartition = euclideanCluster(pts, kdTree, clusterTolerance);

        for (size_t clusterIdx = 0; clusterIdx < clustersPartition.size(); ++clusterIdx) {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

            for (int ptIdx : clustersPartition[clusterIdx]) {
                cloudCluster->points.push_back(cloud->points[ptIdx]);
            }

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }
    else
    {
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);

        for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                cloudCluster->points.push_back(cloud->points[*pit]);
            }

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}