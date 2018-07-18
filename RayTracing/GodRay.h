#ifndef _GOD_RAY_H_
#define _GOD_RAY_H_

#include "Colour.h"
#include "Scene.h"
#include <vector>

//############################################ struct CLightSource ############################################
struct CLightSource
{
	CPoint3D position;
	CPoint3D direction;
	CLightSource() : position(), direction() {}
	CLightSource(CPoint3D v_position, CPoint3D v_direction) : position(v_position), direction(v_direction) {}
};

//############################################ struct CGodPhoton ############################################
struct CGodPhoton
{
	CPoint3D position;
	double energy;
	CGodPhoton() : position() { energy = 0; }
	CGodPhoton(CPoint3D v_position, double v_energy) : position(v_position) { energy = v_energy; }
};

bool compare_x_god(CGodPhoton p1, CGodPhoton p2) { return (p1.position.x < p2.position.x); }
bool compare_y_god(CGodPhoton p1, CGodPhoton p2) { return (p1.position.y < p2.position.y); }
bool compare_z_god(CGodPhoton p1, CGodPhoton p2) { return (p1.position.z < p2.position.z); }

//############################################ struct CPhotonNode ############################################
struct CPhotonNode
{
	CGodPhoton photon;
	int splitAxis;  // normal of the splitting plane
	CPhotonNode * leftChild, *rightChild;
	CPhotonNode() { leftChild = NULL; rightChild = NULL; }
};

//############################################ class CPhotonTree ############################################
class CPhotonTree
{
public:
	CPhotonTree() { root = NULL; }
public:
	void BuildTree(CPhotonNode *& node, std::vector<CGodPhoton> photons);
protected:
	CPhotonNode * root;

	friend class CGodRay;
};

void CPhotonTree::BuildTree(CPhotonNode *& node, std::vector<CGodPhoton> photons)
{
	if (photons.empty())
		return;
	node = new CPhotonNode;
	if (photons.size() == 1)
	{
		node->photon = *photons.begin();
		node->splitAxis = X_AXIS;
		return;
	}
	// Compute variance in x, y, z direction, decide the splitting plane
	double mean_x = 0, mean_y = 0, mean_z = 0;
	double var_x = 0, var_y = 0, var_z = 0;  // variance * N
	for (std::vector<CGodPhoton>::iterator hpIter = photons.begin(); hpIter != photons.end(); hpIter++)
	{
		mean_x += hpIter->position.x;
		mean_y += hpIter->position.y;
		mean_z += hpIter->position.z;
	}
	mean_x /= double(photons.size());
	mean_y /= double(photons.size());
	mean_z /= double(photons.size());
	for (std::vector<CGodPhoton>::iterator hpIter = photons.begin(); hpIter != photons.end(); hpIter++)
	{
		var_x += (hpIter->position.x - mean_x) * (hpIter->position.x - mean_x);
		var_y += (hpIter->position.y - mean_y) * (hpIter->position.y - mean_y);
		var_z += (hpIter->position.z - mean_z) * (hpIter->position.z - mean_z);
	}
	if (var_x >= var_y && var_x >= var_z)
	{
		std::sort(photons.begin(), photons.end(), compare_x_god);
		node->splitAxis = X_AXIS;
	}
	else if (var_y >= var_x && var_y >= var_z)
	{
		std::sort(photons.begin(), photons.end(), compare_y_god);
		node->splitAxis = Y_AXIS;
	}
	else if (var_z >= var_x && var_z >= var_y)
	{
		std::sort(photons.begin(), photons.end(), compare_z_god);
		node->splitAxis = Z_AXIS;
	}
	// Split the 3-d space
	node->photon = *(photons.begin() + photons.size() / 2);
	std::vector<CGodPhoton> hp_left(photons.begin(), photons.begin() + photons.size() / 2);
	BuildTree(node->leftChild, hp_left);
	if (photons.size() >= 3)
	{
		std::vector<CGodPhoton> hp_right(photons.begin() + photons.size() / 2 + 1, photons.end());
		BuildTree(node->rightChild, hp_right);
	}
}


//############################################ class CGodRay ############################################
class CGodRay
{
public:
	CGodRay(CScene * v_scene, CColour v_lightColour) : lightColour(v_lightColour) { scene = v_scene; }
public:
	void PhotonMappingPass();
	CColour RayTracingPass(CRay ray, int depth, double weight, CColour background);
	void AddAreaLightSource_XZ(double y, CPoint2D min, CPoint2D max, double lightSampleInterval, CPoint3D direction);
protected:
	void GeneratePhotons();
	void ConstructPhotonTree() { photonTree.BuildTree(photonTree.root, photons); }
	CColour RadianceEvaluation(CPhotonNode * node, CRay ray);
protected:
	CScene * scene;
	std::vector<CLightSource> lightSources;
	std::vector<CGodPhoton> photons;
	CPhotonTree photonTree;
protected:
	CColour lightColour;
	const double photonRadius = 0.1;
	const double photonSampleInterval = 0.05;
	const double decayRatio = 0.95;
	const double initialEnergy = 0.01;

	friend class CCamera;
};

void CGodRay::PhotonMappingPass()
{
	GeneratePhotons();
	ConstructPhotonTree();
}

CColour CGodRay::RayTracingPass(CRay ray, int depth, double weight, CColour background = CColour(0, 0, 0))
{
	CColour colour(0, 0, 0);
	if (weight < threshold)
		return background;
	// Compute the first intersection point of the ray with the objects in the scene
	CScene::CIntersection intersection;
	intersection = scene->FirstInterPt_BSPTree(ray, true);
	CPoint3D firstInterPt = intersection.pt;
	CBaseObject * firstInterObj = intersection.object;
	if (firstInterObj == NULL)  // No intersection point
		return background;
	// Compute the radiance of god ray
	colour += RadianceEvaluation(photonTree.root, ray);
	// Compute the local illumination
	colour += scene->LocalIllumination(ray, intersection);
	if (depth > 1)
	{
		bool totalReflect = false;
		double refrW = firstInterObj->refrW(ray, firstInterPt);
		double reflW = firstInterObj->reflW(ray, firstInterPt);
		if (refrW != 0)
		{
			CRay refractRay = firstInterObj->RefractRay(ray, firstInterPt);
			if (refractRay.direction() != inftyPt)  // no total reflection
			{
				CColour refractColour = RayTracingPass(refractRay, depth - 1, weight * refrW, background);
				colour += refractColour * refrW;
			}
			else
				totalReflect = true;
		}
		if (reflW != 0)
		{
			CRay reflectRay = firstInterObj->ReflectRay(ray, firstInterPt);
			CColour reflectColour;
			if (totalReflect != true)  // no total reflection
			{
				reflectColour = RayTracingPass(reflectRay, depth - 1, weight * reflW, background);
				colour += reflectColour * reflW;
			}
			else  // total reflection
			{
				reflectColour = RayTracingPass(reflectRay, depth - 1, weight * (reflW + refrW), background);
				colour += reflectColour * (reflW + refrW);
			}
		}
	}
	return colour;
}

void CGodRay::AddAreaLightSource_XZ(double y, CPoint2D min, CPoint2D max, double lightSampleInterval, CPoint3D direction)
{
	for (double x = min.u; x < max.u; x += lightSampleInterval)
		for (double z = min.v; z < max.v; z += lightSampleInterval)
			lightSources.push_back(CLightSource(CPoint3D(x + random(lightSampleInterval / 3, 10000),
			y, z + random(lightSampleInterval / 3, 10000)), direction));

}

void CGodRay::GeneratePhotons()
{
	for (std::vector<CLightSource>::iterator lIter = lightSources.begin(); lIter != lightSources.end(); lIter++)
	{
		CRay ray(lIter->position, lIter->direction);
		CPoint3D pt = scene->FirstInterPt_BSPTree(ray, false).pt;
		double t_range = std::min(Dist(lIter->position, pt), 10.0);
		for (double t = 0, a = 1; t < t_range; t += photonSampleInterval, a *= decayRatio)
		{
			t += random(photonSampleInterval / 3, 10000);
			photons.push_back(CGodPhoton(lIter->position + lIter->direction * t, initialEnergy * a));
		}
	}
}

CColour CGodRay::RadianceEvaluation(CPhotonNode * node, CRay ray)
{
	if (node == NULL)
		return CColour(0, 0, 0);
	CColour colour(0, 0, 0);
	if (ray.Dist(node->photon.position) < photonRadius)
		colour += lightColour * node->photon.energy;
	CPoint3D p0 = node->photon.position;
	CPoint3D normal(0, 0, 0);
	if (node->splitAxis == X_AXIS) normal.x = 1;
	if (node->splitAxis == Y_AXIS) normal.y = 1;
	if (node->splitAxis == Z_AXIS) normal.z = 1;
	CPhotonNode * near = NULL, *far = NULL;
	if (Dot(p0 - ray.beginPt(), normal) > 0)  // ray.beginPt() is in the half-space of leftChild
	{
		near = node->leftChild;
		far = node->rightChild;
	}
	else  // ray.beginPt() is in the half-space of rightChild
	{
		near = node->rightChild;
		far = node->leftChild;
		normal = -normal;
	}
	colour += RadianceEvaluation(near, ray);
	if (Dot(ray.direction(), normal) > -eps || abs(Dot(p0 - ray.beginPt(), normal)) < photonRadius + eps)
		colour += RadianceEvaluation(far, ray);
	return colour;
}

#endif