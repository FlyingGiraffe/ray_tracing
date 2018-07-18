#ifndef _SCENE_H_
#define _SCENE_H_

#include "Object.h"
#include "Light.h"
#include "Parser.h"
#include "BSPTree.h"
#include <vector>
#include <algorithm>

//############################################ class CScene ############################################
class CScene
{
public:
	CScene() : objects(), lights() { ambient = 0; }
	CScene(double v_ambient) : objects(), lights() { ambient = v_ambient; }
protected:
	std::vector<CBaseObject *> objects;
	std::vector<CBaseLight *> lights;
	double ambient;
	BSPTree bspTree;
public:
	struct CIntersection
	{
		CPoint3D pt;  // Location of the intersection point
		CBaseObject * object;  // With which object the ray intersect
		CIntersection() { pt = inftyPt; object = NULL; }
	};
	CIntersection FirstInterPt(CRay ray, bool record);  // Retrun inftyPt if no intersection point
	CIntersection FirstInterPt_BSPTree(CRay ray, bool record);
	CColour LocalIllumination(CRay ray, CIntersection intersection);
	// Add objects to the scene
	void AddObject(CBaseObject * object) { objects.push_back(object); }
	void AddSphere(CPoint3D center, double radius, CMaterial material, CColour colour);
	void AddInftyPlane(CPoint3D  p0, CPoint3D normal, CMaterial material, CColour colour);
	void ReadObjFile(std::string filename, CPoint3D center, double scale, CMaterial material, CColour colour);
	void ReadObjFile_Triangle(std::string filename, CPoint3D center, double scale, CMaterial material, CColour colour);
	void RotationalSurfaceToMesh(CRotationalSurface surface, int uNum, int vNum,
		CPoint3D center, CMaterial material, CColour colour);
	void RotationalSurfaceToMesh_Texture(CRotationalSurface surface, int uNum, int vNum, cv::Mat * texture,
		CPoint3D center, CMaterial material, CColour colour);
	// Add lights to the scene
	void AddLight(CBaseLight * light) { lights.push_back(light); }
	void AddParallelLight(CPoint3D direction, double luminosity);
	void AddPointLight(CPoint3D position, double luminosity);
	void AddAreaLight(CPoint3D v_position, double v_luminosity, double v_len, int v_sampleNum);
	// Construct BSP tree in the scene
	void ConstructBSPTree();
protected:
	CBoundingBox BiggestAABB();
	CScene::CIntersection RayTreeIntersect(CRay ray, BSPNode * node, bool record);
	double IntersectWithPlane(CRay ray, CPoint3D p0, CPoint3D norm);  // intersection point with a plane

	friend class CBaseLight;
	friend class CPointLight;
	friend class CParallelLight;
	friend class CAreaLight;
	friend CColour RayTracer(CRay ray, int depth, double weight, CScene scene, CColour background);
	friend class CPhotonMapper;
};

CScene::CIntersection CScene::FirstInterPt(CRay ray, bool record)
{
	CIntersection intersection;
	for (std::vector<CBaseObject *>::iterator objIter = objects.begin(); objIter != objects.end(); objIter++)
	{
		CPoint3D curInterPt;
		if (record == true)
			curInterPt = (*objIter)->FirstInterPt(ray);
		else
			curInterPt = (*objIter)->FirstInterPt_Unrecorded(ray);
		if (curInterPt != inftyPt)
		{
			if (intersection.pt == inftyPt ||
				Dist(curInterPt, ray.beginPt()) < Dist(intersection.pt, ray.beginPt()) - eps)
			{
				intersection.pt = curInterPt;
				intersection.object = (*objIter);
			}
		}
	}
	return intersection;
}

CScene::CIntersection CScene::FirstInterPt_BSPTree(CRay ray, bool record)
{
	if (bspTree.root == NULL)
		return FirstInterPt(ray, record);
	// Traverse the BSP tree to find the first intersection point
	return RayTreeIntersect(ray, bspTree.root, record);
}

CColour CScene::LocalIllumination(CRay ray, CIntersection intersection)
{
	CPoint3D interPt = intersection.pt;
	CBaseObject * interObj = intersection.object;
	CColour colour(0, 0, 0);
	CPoint3D norm = interObj->Norm(interPt);
	if (Dot(ray.direction(), norm) > eps)
		norm = -norm;
	CColour objColour = interObj->colour(interPt);
	for (std::vector<CBaseLight *>::iterator lightIter = lights.begin(); lightIter != lights.end(); lightIter++)
	{
		CBaseLight * light = *lightIter;
		double L = 0;
		if (light->type() == _AREA_LIGHT_ && bspTree.root != NULL)
			L = ((CAreaLight *)(light))->luminosity_BSPTree(interPt, this);
		else
			L = light->luminosity(interPt, objects);
		// Compute local illumination
		double emissive = 0;
		double diffuse = 0;
		double specluar = 0;
		CRay reflectRay = interObj->ReflectRay(ray, interPt);
		emissive = interObj->emissive();
		diffuse = interObj->diff() * std::max(0.0, -Dot(norm, light->Direction(interPt)) * L);
		specluar = interObj->spec() * L *
			pow(std::max(0.0, -Dot(light->Direction(interPt), reflectRay.direction())), interObj->shiness());
		colour += objColour * (emissive + ambient * interObj->ambient() + diffuse + specluar);
	}
	return colour;
}

void CScene::AddSphere(CPoint3D center, double radius, CMaterial material, CColour colour)
{
	CSphere * sphere = new CSphere(center, radius, material, colour);
	objects.push_back((CBaseObject *)(sphere));
}

void CScene::AddInftyPlane(CPoint3D  p0, CPoint3D normal, CMaterial material, CColour colour)
{
	CInftyPlane * inftyPlane = new CInftyPlane(p0, normal, material, colour);
	objects.push_back((CBaseObject *)(inftyPlane));
}

void CScene::ReadObjFile(std::string filename, CPoint3D center, double scale, CMaterial material, CColour colour)
{
	CObjMesh mesh(filename);
	for (std::vector<CObjMesh::CObjFace>::iterator faceIter = mesh.faces.begin(); faceIter != mesh.faces.end(); faceIter++)
	{
		std::vector<CPoint3D> vertex;
		for (std::vector<CObjMesh::CPointTuple>::iterator tupleIter = faceIter->indices.begin();
			tupleIter != faceIter->indices.end(); tupleIter++)
			vertex.push_back(mesh.points[tupleIter->pointIndex] * scale + center);
		CPolygon * polygon = new CPolygon(vertex, material, colour);
		AddObject((CBaseObject *)(polygon));
	}
}

void CScene::ReadObjFile_Triangle(std::string filename, CPoint3D center, double scale, CMaterial material, CColour colour)
{
	CObjMesh mesh(filename);
	for (std::vector<CObjMesh::CObjFace>::iterator faceIter = mesh.faces.begin(); faceIter != mesh.faces.end(); faceIter++)
	{
		std::vector<CPoint3D> vertex;
		std::vector<CPoint3D> vertexNorm;
		for (std::vector<CObjMesh::CPointTuple>::iterator tupleIter = faceIter->indices.begin();
			tupleIter != faceIter->indices.end(); tupleIter++)
		{
			vertex.push_back(mesh.points[tupleIter->pointIndex] * scale + center);
			vertexNorm.push_back(mesh.normals[tupleIter->normIndex]);
		}
		CTriangle * triangle = new CTriangle(vertex, vertexNorm, material, colour);
		AddObject((CBaseObject *)(triangle));
	}
}

void CScene::RotationalSurfaceToMesh(CRotationalSurface surface, int uNum, int vNum,
	CPoint3D center, CMaterial material, CColour colour)
{
	std::vector<CPoint3D> points;
	std::vector<CPoint3D> normals;
	double du = 1 / double(uNum), dv = 2 * PI / double(vNum);
	int ptId = 0;  // point id
	for (double u = 0, i = 0; u < 1 + eps; u += du, i++)
	{
		if (abs(u - 1) < 100 * eps)
			u = 1;
		for (double v = 0, j = 0; v < 2 * PI + eps; v += dv, j++)
		{
			if (abs(v - 2 * PI) < 100 * eps)
				v = 2 * PI;
			points.push_back(surface.Point3D(u, v));
			normals.push_back(surface.Norm(u, v));
			if (i != 0 && j != 0)
			{
				std::vector<CPoint3D> vertex1;
				std::vector<CPoint3D> vertexNorm1;
				vertex1.push_back(points[ptId] + center);
				vertex1.push_back(points[ptId - 1] + center);
				vertex1.push_back(points[ptId - vNum - 1] + center);
				vertexNorm1.push_back(normals[ptId]);
				vertexNorm1.push_back(normals[ptId - 1]);
				vertexNorm1.push_back(normals[ptId - vNum - 1]);
				CTriangle * triangle1 = new CTriangle(vertex1, vertexNorm1, material, colour);
				objects.push_back(triangle1);

				std::vector<CPoint3D> vertex2;
				std::vector<CPoint3D> vertexNorm2;
				vertex2.push_back(points[ptId - 1] + center);
				vertex2.push_back(points[ptId - vNum - 2] + center);
				vertex2.push_back(points[ptId - vNum - 1] + center);
				vertexNorm2.push_back(normals[ptId - 1]);
				vertexNorm2.push_back(normals[ptId - vNum - 2]);
				vertexNorm2.push_back(normals[ptId - vNum - 1]);
				CTriangle * triangle2 = new CTriangle(vertex2, vertexNorm2, material, colour);
				objects.push_back(triangle2);
			}
			ptId++;
		}
	}

}

void CScene::RotationalSurfaceToMesh_Texture(CRotationalSurface surface, int uNum, int vNum, cv::Mat * texture,
	CPoint3D center, CMaterial material, CColour colour)
{
	std::vector<CPoint3D> points;
	std::vector<CPoint3D> normals;
	double du = 1 / double(uNum), dv = 2 * PI / double(vNum);
	int ptId = 0;  // point id
	for (double u = 0, i = 0; u < 1 + eps; u += du, i++)
	{
		for (double v = 0, j = 0; v < 2 * PI + eps; v += dv, j++)
		{
			points.push_back(surface.Point3D(u, v));
			normals.push_back(surface.Norm(u, v));
			if (i != 0 && j != 0)
			{
				std::vector<CPoint3D> vertex1;
				std::vector<CPoint3D> vertexNorm1;
				std::vector<CPoint2D> textureCoord1;
				vertex1.push_back(points[ptId] + center);
				vertex1.push_back(points[ptId - 1] + center);
				vertex1.push_back(points[ptId - vNum - 1] + center);
				vertexNorm1.push_back(normals[ptId]);
				vertexNorm1.push_back(normals[ptId - 1]);
				vertexNorm1.push_back(normals[ptId - vNum - 1]);
				textureCoord1.push_back(CPoint2D(u * texture->rows, v * texture->cols / (2 * PI)));
				textureCoord1.push_back(CPoint2D(u * texture->rows, (v - dv) * texture->cols / (2 * PI)));
				textureCoord1.push_back(CPoint2D((u - du) * texture->rows, v * texture->cols / (2 * PI)));
				CTriangle * triangle1 = new CTriangle(vertex1, vertexNorm1, textureCoord1, material, colour);
				triangle1->AddTexture(texture);
				objects.push_back(triangle1);

				std::vector<CPoint3D> vertex2;
				std::vector<CPoint3D> vertexNorm2;
				std::vector<CPoint2D> textureCoord2;
				vertex2.push_back(points[ptId - 1] + center);
				vertex2.push_back(points[ptId - vNum - 2] + center);
				vertex2.push_back(points[ptId - vNum - 1] + center);
				vertexNorm2.push_back(normals[ptId - 1]);
				vertexNorm2.push_back(normals[ptId - vNum - 2]);
				vertexNorm2.push_back(normals[ptId - vNum - 1]);
				textureCoord2.push_back(CPoint2D(u * texture->rows, (v - dv) * texture->cols / (2 * PI)));
				textureCoord2.push_back(CPoint2D((u - du) * texture->rows, (v - dv) * texture->cols / (2 * PI)));
				textureCoord2.push_back(CPoint2D((u - du) * texture->rows, v * texture->cols / (2 * PI)));
				CTriangle * triangle2 = new CTriangle(vertex2, vertexNorm2, textureCoord2, material, colour);
				triangle2->AddTexture(texture);
				objects.push_back(triangle2);
			}
			ptId++;
		}
	}

}

void CScene::AddParallelLight(CPoint3D direction, double luminosity)
{
	CParallelLight * parallelLight = new CParallelLight(direction, luminosity);
	lights.push_back((CBaseLight *)(parallelLight));
}

void CScene::AddPointLight(CPoint3D position, double luminosity)
{
	CPointLight * pointLight = new CPointLight(position, luminosity);
	lights.push_back((CBaseLight *)(pointLight));
}

void CScene::AddAreaLight(CPoint3D v_position, double v_luminosity, double v_len, int v_sampleNum)
{
	CAreaLight * areaLight = new CAreaLight(v_position, v_luminosity, v_len, v_sampleNum);
	lights.push_back((CBaseLight *)(areaLight));
}

void CScene::ConstructBSPTree()
{
	CBoundingBox bigBox = BiggestAABB();
	if (bigBox == CBoundingBox(inftyPt, inftyPt))
		return;
	bspTree.root = new BSPNode(bigBox, objects);
	bspTree.Subdivide(bspTree.root, 0, 0);
}

CBoundingBox CScene::BiggestAABB()
{
	double x_min = infty, x_max = -infty;
	double y_min = infty, y_max = -infty;
	double z_min = infty, z_max = -infty;
	for (std::vector<CBaseObject *>::iterator objIter = objects.begin(); objIter != objects.end(); objIter++)
	{
		CBoundingBox box = (*objIter)->AABB();
		if (box == CBoundingBox(inftyPt, inftyPt))
			continue;
		if (box.min().x < x_min) x_min = box.min().x;
		if (box.min().y < y_min) y_min = box.min().y;
		if (box.min().z < z_min) z_min = box.min().z;
		if (box.max().x > x_max) x_max = box.max().x;
		if (box.max().y > y_max) y_max = box.max().y;
		if (box.max().z > z_max) z_max = box.max().z;
	}
	if (x_min != infty && y_min != infty && z_min != infty &&
		x_max != -infty && y_max != -infty && z_max != infty)
		return CBoundingBox(CPoint3D(x_min, y_min, z_min), CPoint3D(x_max, y_max, z_max));
	else
		return CBoundingBox(inftyPt, inftyPt);
}

CScene::CIntersection CScene::RayTreeIntersect(CRay ray, BSPNode * node, bool record)
{	
	CPoint2D t_min_max = node->box.Collide(ray);
	double t_min = t_min_max.u;
	double t_max = t_min_max.v;
	CPoint3D inPt = ray.beginPt() + ray.direction() * t_min;
	CPoint3D outPt = ray.beginPt() + ray.direction() * t_max;
	if (node == NULL || t_min_max == inftyPt2D)
	{
		CIntersection intersection;
		for (std::vector<CBaseObject *>::iterator objIter = bspTree.root->objects.begin();
			objIter != bspTree.root->objects.end(); objIter++)
		{
			CPoint3D curInterPt;
			if (record == true)
				curInterPt = (*objIter)->FirstInterPt(ray);
			else
				curInterPt = (*objIter)->FirstInterPt_Unrecorded(ray);
			if (curInterPt != inftyPt)
			{
				if (intersection.pt == inftyPt ||
					Dist(curInterPt, ray.beginPt()) < Dist(intersection.pt, ray.beginPt()) - eps)
				{
					intersection.pt = curInterPt;
					intersection.object = (*objIter);
				}
			}
		}
		return intersection;
	}
	if (node->leftChild == NULL && node->rightChild == NULL)  // Current node is a leaf
	{
		CIntersection intersection;
		for (std::vector<CBaseObject *>::iterator objIter = node->objects.begin(); objIter != node->objects.end(); objIter++)
		{
			CPoint3D curInterPt;
			if (record == true)
				curInterPt = (*objIter)->FirstInterPt(ray);
			else
				curInterPt = (*objIter)->FirstInterPt_Unrecorded(ray);
			if (curInterPt != inftyPt)
			{
				if (intersection.pt == inftyPt ||
					Dist(curInterPt, ray.beginPt()) < Dist(intersection.pt, ray.beginPt()) - eps)
				{
					intersection.pt = curInterPt;
					intersection.object = (*objIter);
				}
			}
		}
		for (std::vector<CBaseObject *>::iterator objIter = bspTree.root->objects.begin();
			objIter != bspTree.root->objects.end(); objIter++)
		{
			CPoint3D curInterPt;
			if (record == true)
				curInterPt = (*objIter)->FirstInterPt(ray);
			else
				curInterPt = (*objIter)->FirstInterPt_Unrecorded(ray);
			if (curInterPt != inftyPt)
			{
				if (intersection.pt == inftyPt ||
					Dist(curInterPt, ray.beginPt()) < Dist(intersection.pt, ray.beginPt()) - eps)
				{
					intersection.pt = curInterPt;
					intersection.object = (*objIter);
				}
			}
		}
		return intersection;
	}
	CPoint3D p0 = (node->box.min() + node->box.max()) / 2;
	CPoint3D normal(0, 0, 0);
	if (node->splitAxis == X_AXIS) normal.x = 1;
	if (node->splitAxis == Y_AXIS) normal.y = 1;
	if (node->splitAxis == Z_AXIS) normal.z = 1;
	double dist = IntersectWithPlane(ray, p0, normal);
	BSPNode * near = NULL, * far = NULL;
	if (Dot(p0 - ray.beginPt(), normal) > 0)  // ray.beginPt() is in the half-space of leftChild
	{
		near = node->leftChild;
		far = node->rightChild;
	}
	else  // ray.beginPt() is in the half-space of rightChild
	{
		near = node->rightChild;
		far = node->leftChild;
	}
	if (dist > t_max + eps || dist < -eps)  // Only need to test the near side
		return RayTreeIntersect(ray, near, record);
	else if (dist < t_min - eps && abs(dist) > eps)  // Only need to test the far side
		return RayTreeIntersect(ray, far, record);
	else  // Need to test both sides
	{
		CIntersection inter1 = RayTreeIntersect(ray, near, record);
		CIntersection inter2 = RayTreeIntersect(ray, far, record);
		if (Dist2(inter1.pt, ray.beginPt()) < Dist2(inter2.pt, ray.beginPt()) - eps)
			return inter1;
		else
			return inter2;
	}
}

double CScene::IntersectWithPlane(CRay ray, CPoint3D p0, CPoint3D norm)
{
	if (abs(Dot(norm, ray.direction())) < eps)
	{
		if (abs(Dot(p0 - ray.beginPt(), norm)) <= eps)  // The eye is looking directly at the "lateral" of the plane
			return 0;
		else
			return infty;
	}
	double D = -Dot(p0, norm);
	return -(D + Dot(norm, ray.beginPt())) / Dot(norm, ray.direction());
}


//############################################ This should be in file "Light.h", but emmm... ############################################
double CAreaLight::luminosity_BSPTree(CPoint3D pt, CScene * scene)
{
	double L = 0;
	double r = (len / double(sampleNum)) / 2;
	CPoint3D O = CPoint3D(m_position.x - sampleNum * r, m_position.y - sampleNum * r, m_position.z);
	for (int i = 0; i < sampleNum; i++)
	{
		for (int j = 0; j < sampleNum; j++)
		{
			double x0 = O.x + i * r * 2, y0 = O.y + j * r * 2, z0 = O.z;
			CPoint3D samplePt = CPoint3D(x0 + random(r, 1000), y0 + random(r, 1000), z0);
			CRay ray(pt, samplePt - pt);
			CPoint3D interPt = scene->FirstInterPt_BSPTree(ray, false).pt;
			if (interPt != inftyPt  && Dot(pt - interPt, m_position - interPt) < -eps)
				continue;
			L += lumPerSamplePt;
		}
	}
	return L;
}

#endif