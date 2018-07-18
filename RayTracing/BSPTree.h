#ifndef _BSP_TREE_H_
#define _BSP_TREE_H_

#include "Geometry.h"
#include <vector>

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

class CBaseObject;

template <typename T>
void ClearVector(std::vector<T> &vec)
{
	std::vector<T>().swap(vec);
}

//############################################ struct BSPNode ############################################
struct BSPNode
{
	CBoundingBox box;
	std::vector<CBaseObject *> objects;
	int splitAxis;
	BSPNode * leftChild, * rightChild;
	BSPNode(CBoundingBox v_box) : box(v_box)
	{
		leftChild = NULL;
		rightChild = NULL;
		splitAxis = -1;
	}
	BSPNode(CBoundingBox v_box, std::vector<CBaseObject *> v_objects) : box(v_box), objects(v_objects)
	{
		leftChild = NULL;
		rightChild = NULL;
		splitAxis = -1;
	}
};

//############################################ class BSPTree ############################################
class BSPTree
{
public:
	BSPTree() { root = NULL; }
	BSPTree(CBoundingBox rootBox) { root = new BSPNode(rootBox); Subdivide(root, 0, 0); }
protected:
	void Subdivide(BSPNode * node, int depth, int curAxis);
protected:
	BSPNode * root;

	friend class CScene;
	friend class CPhotonMapper;
	friend CColour RayTracer(CRay ray, int depth, double weight, CScene scene, CColour background);
};

const int maxObjNum = 100;
const int maxDepth = 40;
void BSPTree::Subdivide(BSPNode * node, int depth, int curAxis)
{
	if (node->objects.size() < maxObjNum || depth == maxDepth)
		return;
	node->splitAxis = curAxis;
	// If current node contains too many objects or current depth is not too high
	int nextAxis = 0;
	CBoundingBox left;
	CBoundingBox right;
	CPoint3D min = node->box.min();
	CPoint3D max = node->box.max();
	if (curAxis == X_AXIS)  // 0 for X-axis
	{
		left = CBoundingBox(min, CPoint3D((min.x + max.x) / 2, max.y, max.z));
		right = CBoundingBox(CPoint3D((min.x + max.x) / 2, min.y, min.z), max);
		node->leftChild = new BSPNode(left);
		node->rightChild = new BSPNode(right);
		nextAxis = Y_AXIS;
	}
	else if (curAxis == Y_AXIS)  // 1 for Y-axis
	{
		left = CBoundingBox(min, CPoint3D(max.x, (min.y + max.y) / 2, max.z));
		right = CBoundingBox(CPoint3D(min.x, (min.y + max.y) / 2, min.z), max);
		node->leftChild = new BSPNode(left);
		node->rightChild = new BSPNode(right);
		nextAxis = Z_AXIS;
	}
	else if (curAxis == Z_AXIS)  // 2 for Z-axis
	{
		left = CBoundingBox(min, CPoint3D(max.x, max.y, (min.z + max.z) / 2));
		right = CBoundingBox(CPoint3D(min.x, min.y, (min.z + max.z) / 2), max);
		node->leftChild = new BSPNode(left);
		node->rightChild = new BSPNode(right);
		nextAxis = X_AXIS;
	}
	std::vector<CBaseObject *> inftyObjs;
	for (std::vector<CBaseObject *>::iterator objIter = node->objects.begin(); objIter != node->objects.end(); objIter++)
	{
		if ((*objIter)->PointInBox(left))
			node->leftChild->objects.push_back(*objIter);
		if ((*objIter)->PointInBox(right))
			node->rightChild->objects.push_back(*objIter);
		if (node == root && (*objIter)->AABB() == CBoundingBox(inftyPt, inftyPt))
			inftyObjs.push_back(*objIter);
	}
	ClearVector(node->objects);
	if (node == root)
		node->objects = inftyObjs;
	ClearVector(inftyObjs);
	Subdivide(node->leftChild, depth + 1, nextAxis);
	Subdivide(node->rightChild, depth + 1, nextAxis);
}

#endif