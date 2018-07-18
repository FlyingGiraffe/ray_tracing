#ifndef _PHOTON_MAPPER_H_
#define _PHOTON_MAPPER_H_

#include "Colour.h"
#include "Scene.h"
#include <stack>
#include <vector>
#include <algorithm>

const double R_initial = 1;
const double alpha = 0.8;

//############################################ struct CHitPoint ############################################
struct CHitpoint
{
	CPoint3D pt;  // Hit location
	CPoint3D norm;  //  Normal at x
	CPoint3D omega;  // Ray direction
	int h, v;  // Pixel location
	double wgt;  // Pixel weight
	double R;  // Current photon radius
	double N;  // Accumulated photon count
	double M;  // Added photon count
	CColour tau;  // Accumulated reflected flux
	CHitpoint() : tau() { R = R_initial; N = 0; M = 0; }
};

bool compare_x(CHitpoint hp1, CHitpoint hp2) { return (hp1.pt.x < hp2.pt.x); }
bool compare_y(CHitpoint hp1, CHitpoint hp2) { return (hp1.pt.y < hp2.pt.y); }
bool compare_z(CHitpoint hp1, CHitpoint hp2) { return (hp1.pt.z < hp2.pt.z); }


//############################################ struct CPhotonHP, photon hit point ############################################
struct CPhotonHP
{
	CPoint3D pt;
	CPoint3D norm;
	CColour colour;
	CPhotonHP(CPoint3D v_pt, CPoint3D v_norm, CColour v_colour) : pt(v_pt), norm(v_norm), colour(v_colour) {}
};

struct CPhoton
{
	CRay path;
	CColour colour;
	double weight;
	CPhoton(CRay v_path, CColour v_colour) : path(v_path), colour(v_colour) { weight = 1; }
};


//############################################ struct KdTreeNode ############################################
struct KdTreeNode
{
	CHitpoint hitpoint;
	int splitAxis;  // normal of the splitting plane
	KdTreeNode * leftChild, *rightChild;
	KdTreeNode() { leftChild = NULL; rightChild = NULL; }
};


//############################################ class KdTree ############################################
class KdTree
{
public:
	KdTree() { root = NULL; }
public:
	void BuildTree(KdTreeNode *& node, std::vector<CHitpoint> hitpoints);
protected:
	KdTreeNode * root;

	friend class CScene;
	friend class CPhotonMapper;
	friend CColour RayTracer(CRay ray, int depth, double weight, CScene scene, CColour background);
};

void KdTree::BuildTree(KdTreeNode *& node, std::vector<CHitpoint> hitpoints)
{	
	if (hitpoints.empty())
		return;
	node = new KdTreeNode;
	if (hitpoints.size() == 1)
	{
		node->hitpoint = *hitpoints.begin();
		node->splitAxis = X_AXIS;
		return;
	}
	// Compute variance in x, y, z direction, decide the splitting plane
	double mean_x = 0, mean_y = 0, mean_z = 0;
	double var_x = 0, var_y = 0, var_z = 0;  // variance * N
	for (std::vector<CHitpoint>::iterator hpIter = hitpoints.begin(); hpIter != hitpoints.end(); hpIter++)
	{
		mean_x += hpIter->pt.x;
		mean_y += hpIter->pt.y;
		mean_z += hpIter->pt.z;
	}
	mean_x /= double(hitpoints.size());
	mean_y /= double(hitpoints.size());
	mean_z /= double(hitpoints.size());
	for (std::vector<CHitpoint>::iterator hpIter = hitpoints.begin(); hpIter != hitpoints.end(); hpIter++)
	{
		var_x += (hpIter->pt.x - mean_x) * (hpIter->pt.x - mean_x);
		var_y += (hpIter->pt.y - mean_y) * (hpIter->pt.y - mean_y);
		var_z += (hpIter->pt.z - mean_z) * (hpIter->pt.z - mean_z);
	}
	if (var_x >= var_y && var_x >= var_z)
	{
		std::sort(hitpoints.begin(), hitpoints.end(), compare_x);
		node->splitAxis = X_AXIS;
	}
	else if (var_y >= var_x && var_y >= var_z)
	{
		std::sort(hitpoints.begin(), hitpoints.end(), compare_y);
		node->splitAxis = Y_AXIS;
	}
	else if (var_z >= var_x && var_z >= var_y)
	{
		std::sort(hitpoints.begin(), hitpoints.end(), compare_z);
		node->splitAxis = Z_AXIS;
	}
	// Split the 3-d space
	node->hitpoint = *(hitpoints.begin() + hitpoints.size() / 2);
	std::vector<CHitpoint> hp_left(hitpoints.begin(), hitpoints.begin() + hitpoints.size() / 2);
	BuildTree(node->leftChild, hp_left);
	if (hitpoints.size() >= 3)
	{
		std::vector<CHitpoint> hp_right(hitpoints.begin() + hitpoints.size() / 2 + 1, hitpoints.end());
		BuildTree(node->rightChild, hp_right);
	}
}


//############################################ class CPhotonMapper ############################################
class CPhotonMapper
{
public:
	CPhotonMapper(CScene * v_scene, cv::Mat * v_picture) : kdTree() { scene = v_scene; picture = v_picture; N_emitted = 0; }
public:
	CColour RayTracingPass(CRay ray, int depth, double weight, int h, int v, CColour background);
	void PhotonMappingPass(int times);
protected:
	void ConstructKdTree() { kdTree.BuildTree(kdTree.root, hitpoints); }
	void GeneratePhotons();
	void TracePhotons();
	void AccumulatePhotons(KdTreeNode * node, CPhotonHP photonHp);
	void RadianceEvaluation(KdTreeNode * node);
	CPoint3D RandomDirection() { return CPoint3D(random(1000, 1000), random(1000, 1000), random(1000, 1000)).Unitize(); }
public:
	CScene * scene;
	cv::Mat * picture;
	KdTree kdTree;
	std::stack<CPhoton> photons;
	std::stack<CPhotonHP> photonHps;
	std::vector<CHitpoint> hitpoints;
	int N_emitted;
	const int photonNum = 50000;
	const double threshold_RayTracingPass = 0.1;
};

CColour CPhotonMapper::RayTracingPass(CRay ray, int depth, double weight, int h, int v, CColour background = CColour(0, 0, 0))
{
	CColour colour(0, 0, 0);
	if (weight < threshold_RayTracingPass)
		return background;
	// Compute the first intersection point of the ray with the objects in the scene
	CScene::CIntersection intersection;
	intersection = scene->FirstInterPt_BSPTree(ray, true);
	CPoint3D firstInterPt = intersection.pt;
	CBaseObject * firstInterObj = intersection.object;
	if (firstInterObj == NULL)  // No intersection point
		return background;
	// Compute the local illumination
	colour = scene->LocalIllumination(ray, intersection);
	// Store the hitpoint
	if (firstInterObj->reflW() == 0 && firstInterObj->refrW() == 0)
	{
		CHitpoint hp;
		hp.pt = firstInterPt;  // Hit location
		hp.norm = firstInterObj->Norm(firstInterPt);  //  Normal at x
		if (Dot(ray.direction(), hp.norm) > eps)
			hp.norm = -hp.norm;
		hp.omega = ray.direction();  // Ray direction
		hp.h = h;
		hp.v = v;  // Pixel location
		hp.wgt = weight;  // Pixel weight
		hitpoints.push_back(hp);
	}
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
				CColour refractColour = RayTracingPass(refractRay, depth - 1, weight * refrW, h, v, background);
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
				reflectColour = RayTracingPass(reflectRay, depth - 1, weight * reflW, h, v, background);
				colour += reflectColour * reflW;
			}
			else  // total reflection
			{
				reflectColour = RayTracingPass(reflectRay, depth - 1, weight * (reflW + refrW), h, v, background);
				colour += reflectColour * (reflW + refrW);
			}
		}
	}
	return colour;
}

void CPhotonMapper::PhotonMappingPass(int times)
{
	ConstructKdTree();
	N_emitted = times * scene->lights.size() * photonNum;
	for (int t = 0; t < times; t++)
	{
		std::cout << t << std::endl;
		GeneratePhotons();
		TracePhotons();
		for (; !photonHps.empty();)
		{
			CPhotonHP photonHp = photonHps.top();
			photonHps.pop();
			AccumulatePhotons(kdTree.root, photonHp);
		}
		RadianceEvaluation(kdTree.root);
	}
}

void CPhotonMapper::GeneratePhotons()
{
	for (std::vector<CBaseLight *>::iterator lIter = scene->lights.begin(); lIter != scene->lights.end(); lIter++)
	{
		if ((*lIter)->type() == _POINT_LIGHT_)
		{
			CPointLight * light = (CPointLight *)(*lIter);
			for (int t = 0; t < photonNum; t++)
				photons.push(CPhoton(CRay(light->position(), RandomDirection()), CColour(255, 255, 255) * light->m_luminosity));
		}
		if ((*lIter)->type() == _AREA_LIGHT_)
		{
			CAreaLight * light = (CAreaLight *)(*lIter);
			for (int t = 0; t < photonNum; t++)
			{
				double r = (light->len / double(light->sampleNum)) / 2;
				CPoint3D O = CPoint3D(light->m_position.x - light->sampleNum * r,
					light->m_position.y - light->sampleNum * r, light->m_position.z);
				for (int i = 0; i < light->sampleNum; i++)
				{
					for (int j = 0; j < light->sampleNum; j++)
					{
						double x0 = O.x + i * r * 2, y0 = O.y + j * r * 2, z0 = O.z;
						CPoint3D samplePt = CPoint3D(x0 + random(r, 1000), y0 + random(r, 1000), z0);
						photons.push(CPhoton(CRay(samplePt, RandomDirection()), CColour(255, 255, 255) * light->lumPerSamplePt));
					}
				}
			}
		}
	}
}

void CPhotonMapper::TracePhotons()
{
	for (; !photons.empty();)
	{
		CPhoton photon = photons.top();
		photons.pop();
		CScene::CIntersection intersection = scene->FirstInterPt_BSPTree(photon.path, true);
		for (int i = 0; i < 10 && photon.weight > 0.2 && intersection.object != NULL; i++)
		{
			// surface of diffusion
			if (intersection.object->reflW() == 0 && intersection.object->refrW() == 0)
			{
				CPoint3D hp = intersection.pt;
				CPoint3D norm = intersection.object->Norm(hp);
				if (Dot(photon.path.direction(), norm) > eps)
					norm = -norm;
				photonHps.push(CPhotonHP(hp, norm, photon.colour * photon.weight));
				// reflected photon
				CPoint3D dir = RandomDirection();
				if (Dot(dir, norm) < 0)
					dir = -dir;
				photon.path = CRay(hp, dir);
				photon.weight /= 4;
				CColour objColour = intersection.object->colour(hp);
				double s = (photon.colour.R() + photon.colour.G() + photon.colour.B()) /
					(objColour.R() + objColour.G() + objColour.B());
				photon.colour = (photon.colour + objColour * s) / 2;
			}
			// surface of refraction (transmission)
			else if (intersection.object->refrW() > 0)
			{
				CPoint3D hp = intersection.pt;
				CPoint3D norm = intersection.object->Norm(hp);
				// refracted photon
				CRay new_path = intersection.object->RefractRay(photon.path, hp);
				if (new_path.direction() != inftyPt)
					photon.path = new_path;  // no total reflection
				else
					photon.path = intersection.object->ReflectRay(photon.path, hp);  // total reflection
				CColour objColour = intersection.object->colour(hp);
				double s = (photon.colour.R() + photon.colour.G() + photon.colour.B()) /
					(objColour.R() + objColour.G() + objColour.B());
				photon.colour = (photon.colour + objColour * s) / 2;
			}
			// surface of mirror reflection
			else if (intersection.object->reflW() > 0)
			{
				CPoint3D hp = intersection.pt;
				CPoint3D norm = intersection.object->Norm(hp);
				// reflected photon
				photon.path = intersection.object->ReflectRay(photon.path, hp);
				CColour objColour = intersection.object->colour(hp);
				double s = (photon.colour.R() + photon.colour.G() + photon.colour.B()) /
					(objColour.R() + objColour.G() + objColour.B());
				photon.colour = (photon.colour + objColour * s) / 2;
			}
			intersection = scene->FirstInterPt_BSPTree(photon.path, true);
		}
	}
}

void CPhotonMapper::AccumulatePhotons(KdTreeNode * node, CPhotonHP photonHp)
{
	if (node == NULL)
		return;
	if (Dist(node->hitpoint.pt, photonHp.pt) < node->hitpoint.R + eps &&
		Dot(node->hitpoint.norm, photonHp.norm) > eps * 100)
	{
		node->hitpoint.M++;
		node->hitpoint.tau += photonHp.colour / ((PI * node->hitpoint.R * node->hitpoint.R * N_emitted) / 3000);
	}
	CPoint3D p = photonHp.pt;
	CPoint3D p0 = node->hitpoint.pt;
	CPoint3D normal(0, 0, 0);
	if (node->splitAxis == X_AXIS) normal.x = 1;
	if (node->splitAxis == Y_AXIS) normal.y = 1;
	if (node->splitAxis == Z_AXIS) normal.z = 1;
	double l = Dot(p - p0, normal);
	if (l < eps || abs(l) < R_initial + eps)
		AccumulatePhotons(node->leftChild, photonHp);
	if (l > -eps || abs(l) < R_initial + eps)
		AccumulatePhotons(node->rightChild, photonHp);
}

void CPhotonMapper::RadianceEvaluation(KdTreeNode * node)
{
	if (node == NULL)
		return;
	// Recompute the colour for the pixel
	if (picture->type() == CV_8UC3)
		picture->at<cv::Vec3b>(node->hitpoint.v, node->hitpoint.h) +=
			cv::Vec3b(node->hitpoint.tau.B(), node->hitpoint.tau.G(), node->hitpoint.tau.R()) * node->hitpoint.wgt;
	else if (picture->type() == CV_32SC3)
		picture->at<cv::Vec3i>(node->hitpoint.v, node->hitpoint.h) +=
			cv::Vec3i(node->hitpoint.tau.B(), node->hitpoint.tau.G(), node->hitpoint.tau.R()) * node->hitpoint.wgt;
	node->hitpoint.tau = CColour(0, 0, 0);
	// Recompute the N's and R's
	if (node->hitpoint.M > 0)
		node->hitpoint.R *= sqrt((node->hitpoint.N + node->hitpoint.M * alpha) / (node->hitpoint.N + node->hitpoint.M));
	node->hitpoint.N += node->hitpoint.M * alpha;
	node->hitpoint.M = 0;
	RadianceEvaluation(node->leftChild);
	RadianceEvaluation(node->rightChild);
}

#endif