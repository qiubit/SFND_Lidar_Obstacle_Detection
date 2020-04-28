/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once

#include "render/render.h"
#include <memory>
#include <vector>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::shared_ptr<Node> left;
	std::shared_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	std::shared_ptr<Node> root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		destructHelper(root);
	}

	void destructHelper(std::shared_ptr<Node> curNode)
	{
		if (curNode == nullptr)
			return;
		
		destructHelper(curNode->left);
		destructHelper(curNode->right);

		curNode->left = nullptr;
		curNode->right = nullptr;
	}

	void insert(std::vector<float> point, int id)
	{
		std::shared_ptr<Node> toInsert(new Node(point, id));
		root = insertHelper(toInsert, root, 0);
	}

	std::vector<std::pair<int, std::vector<float>>> getPoints()
	{
		std::vector<std::pair<int, std::vector<float>>> points;
		getPointsHelper(points, root);
		return points;
	}

	void getPointsHelper(std::vector<std::pair<int, std::vector<float>>> &points, std::shared_ptr<Node> node)
	{
		if (node == nullptr)
			return;

		points.push_back(std::make_pair(node->id, node->point));

		getPointsHelper(points, node->left);
		getPointsHelper(points, node->right);
	}

	std::shared_ptr<Node> insertHelper(std::shared_ptr<Node> toInsert, std::shared_ptr<Node> current, size_t level)
	{
		if (current == nullptr) {
			return toInsert;
		}
		else {
			std::vector<float> &point = current->point;
			size_t coordinateToCompare = level % point.size();

			float toInsertVal = toInsert->point[coordinateToCompare];
			float curVal = current->point[coordinateToCompare];

			if (toInsertVal <= curVal) {
				current->left = insertHelper(toInsert, current->left, level+1);
			} else {
				current->right = insertHelper(toInsert, current->right, level+1);
			}

			return current;
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, ids, root, 0);
		return ids;
	}

	void searchHelper(std::vector<float> &target, float distanceTol, std::vector<int> &ids, std::shared_ptr<Node> curNode, size_t depth)
	{
		if (curNode == nullptr)
			return;

		size_t coordToCompare = depth % target.size();
		float coordDiff = curNode->point[coordToCompare] - target[coordToCompare];

		// float xCoordDiff = curNode->point[0] - target[0];
		// float yCoordDiff = curNode->point[1] - target[1];
		// float zCoordDiff = curNode->point[2] - target[2];

		// bool curNodeInBox = fabsf(xCoordDiff) <= distanceTol
		// 	&& fabsf(yCoordDiff) <= distanceTol
		// 	&& fabsf(zCoordDiff) <= distanceTol;

		bool curNodeInBox = fabsf(coordDiff) <= distanceTol;

		float coordSquareSum = 0.0;
		for (size_t i = 0; i < target.size(); ++i) {
			float curCoordDiff = curNode->point[i] - target[i];
			coordSquareSum += curCoordDiff*curCoordDiff;
		}

		float distance = sqrt(coordSquareSum);

		if (curNodeInBox) {
			if (distance <= distanceTol) {
				ids.push_back(curNode->id);
			}
			searchHelper(target, distanceTol, ids, curNode->left, depth+1);
			searchHelper(target, distanceTol, ids, curNode->right, depth+1);
		} else {
			size_t coordinateToCompare = depth % target.size();

			float curNodeCoord = curNode->point[coordinateToCompare];
			float targetCoord = target[coordinateToCompare];

			if (targetCoord <= curNodeCoord) {
				searchHelper(target, distanceTol, ids, curNode->left, depth+1);
			} else {
				searchHelper(target, distanceTol, ids, curNode->right, depth+1);
			}
		}
	}
};
