#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "Geometry.h"
#include "Bezier.h"
#include "Material.h"
#include "Colour.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
#include <string>
#include <Eigen>

using namespace Eigen;

#define _SPHERE_ 0
#define _TRIANGLE_ 1
#define _POLYGON_ 2
#define _CIRCLE_ 3
#define _INFTY_PLANE_ 4
#define _ROTATIONAL_OBJECT_ 5
#define _BEZIER_OBJECT_ 6

//############################################ class CBaseObject ############################################
class CBaseObject
{
public:
	CBaseObject(CMaterial v_material, CColour v_colour) :
		material(v_material), m_colour(v_colour)
	{
		haveTexture = false;
		haveBumpTexture = false;
		m_AABB = CBoundingBox(inftyPt, inftyPt);
	}
public:
	virtual CPoint3D FirstInterPt(CRay ray) = 0;  // Retrun inftyPt if no intersection point
	virtual CPoint3D FirstInterPt_Unrecorded(CRay ray) = 0;
	virtual CPoint3D InitialNorm(CPoint3D pt) = 0;  // Return the normal vector of the given point
	virtual CPoint3D Norm(CPoint3D pt);
	CRay ReflectRay(CRay ray, CPoint3D interPt);
	CRay RefractRay(CRay ray, CPoint3D interPt);
	double reflW() { return material.reflW(); }
	double refrW() { return material.refrW(); }
	double reflW(CRay ray, CPoint3D interPt);
	double refrW(CRay ray, CPoint3D interPt);
	CColour colour(CPoint3D pt);  // the colour at the given point (if there is texture)
	double diff() { return material.diff(); }
	double spec() { return material.spec(); }
	double shiness() { return material.shiness(); }
	double emissive() { return material.emissive(); }
	double ambient() { return material.ambient(); }
	void AddTexture(std::string filename);
	void AddTexture(cv::Mat * v_texture) { texture = v_texture; haveTexture = true; }
	void AddBumpTexture(std::string filename);
	void AddBumpTexture(cv::Mat * v_bumpTexture) { bumpTexture = v_bumpTexture; haveBumpTexture = true; }
	int type() { return m_type; }
	bool PointInBox(CBoundingBox box); // If the object has points in the bounding box, return true
	CBoundingBox AABB() { return m_AABB; }
protected:
	virtual CPoint2D UV_colour(CPoint3D pt) { return CPoint2D(); }  // u, v \in [texture.rows, texture.cols]
	virtual CPoint2D UV_norm(CPoint3D pt) { return CPoint2D(); }  // u, v \in [0, 1]
	virtual CPoint3D Tangent(CPoint3D pt) { return CPoint3D(); }
	virtual CPoint3D Bitangent(CPoint3D pt) { return CPoint3D(); }
protected:
	CMaterial material;
	CColour m_colour;
	int m_type;
	// texture
	cv::Mat * texture;
	bool haveTexture;
	// bump texture
	cv::Mat * bumpTexture;
	bool haveBumpTexture;
	// bounding box
	CBoundingBox m_AABB;
};

CPoint3D CBaseObject::Norm(CPoint3D pt)
{
	CPoint3D N = InitialNorm(pt);
	if (haveBumpTexture == false)
		return N;
	else
	{
		CPoint2D pt2D = UV_norm(pt);
		cv::Vec3b cur_norm = bumpTexture->at<cv::Vec3b>(int(pt2D.u * bumpTexture->rows), int(pt2D.v * bumpTexture->cols));
		return (Bitangent(pt) * (cur_norm[2] / 127.5 - 1) +
			Tangent(pt) * (cur_norm[1] / 127.5 - 1) +
			N * (cur_norm[0] / 127.5 - 1)).Unitize();
	}
}

CRay CBaseObject::ReflectRay(CRay ray, CPoint3D interPt)
{
	CPoint3D N = Norm(interPt);
	if (Dot(ray.direction(), N) > eps)
		N = -N;
	CPoint3D dir = ray.direction() - N * (2 * Dot(ray.direction(), N));
	return CRay(interPt, dir.Unitize());
}

CRay CBaseObject::RefractRay(CRay ray, CPoint3D interPt)
{
	CPoint3D N = Norm(interPt);
	CPoint3D T = inftyPt;  // direction of refraction ray
	if (Dot(ray.direction(), N) < eps)  // From optically thinner medium to optically denser medium
	{
		double ni_nt = 1 / material.RI();
		double cos_theta_i2 = Dot(ray.direction(), N) * Dot(ray.direction(), N);  // cos(\theta_i)^2
		double cos_theta_t = sqrt(1 - ni_nt * ni_nt * (1 - cos_theta_i2));
		T = N * (ni_nt * Dot(-ray.direction(), N) - cos_theta_t) + ray.direction() * ni_nt;
	}
	else  // From optically denser medium to optically thinner medium
	{
		N = -N;
		double ni_nt = material.RI();
		double cos_theta_i2 = Dot(ray.direction(), N) * Dot(ray.direction(), N);  // cos(\theta_i)^2
		if (1 - ni_nt * ni_nt * (1 - cos_theta_i2) > eps)  // no total reflection
		{
			double cos_theta_t = sqrt(1 - ni_nt * ni_nt * (1 - cos_theta_i2));
			T = N * (ni_nt * Dot(-ray.direction(), N) - cos_theta_t) + ray.direction() * ni_nt;
		}
	}
	return CRay(interPt, T.Unitize());
}

double CBaseObject::reflW(CRay ray, CPoint3D interPt)
{
	if (reflW() == 0 || refrW() == 0)
		return reflW();
	CPoint3D N = Norm(interPt);
	double ni_nt = 0;
	double cos_theta_i = 0;
	double cos_theta_t = 0;
	if (Dot(ray.direction(), N) < eps)  // From optically thinner medium to optically denser medium
	{
		ni_nt = 1 / material.RI();
		double cos_theta_i2 = Dot(ray.direction(), N) * Dot(ray.direction(), N);  // cos(\theta_i)^2
		cos_theta_i = sqrt(cos_theta_i2);
		cos_theta_t = sqrt(1 - ni_nt * ni_nt * (1 - cos_theta_i2));
	}
	else  // From optically denser medium to optically thinner medium
	{
		N = -N;
		ni_nt = material.RI();
		double cos_theta_i2 = Dot(ray.direction(), N) * Dot(ray.direction(), N);  // cos(\theta_i)^2
		cos_theta_i = sqrt(cos_theta_i2);
		if (1 - ni_nt * ni_nt * (1 - cos_theta_i2) > eps)  // no total reflection
			cos_theta_t = sqrt(1 - ni_nt * ni_nt * (1 - cos_theta_i2));
	}
	double r_p = (ni_nt * cos_theta_i - cos_theta_t) / (ni_nt * cos_theta_i + cos_theta_t);  // r_parallel
	double r_v = (cos_theta_i - ni_nt * cos_theta_t) / (cos_theta_i + ni_nt * cos_theta_t);  // r_vertical
	return (r_p * r_p + r_v * r_v) / 2;
}

double CBaseObject::refrW(CRay ray, CPoint3D interPt)
{
	if (reflW() == 0 || refrW() == 0)
		return refrW();
	CPoint3D N = Norm(interPt);
	double ni_nt = 0;
	double cos_theta_i = 0;
	double cos_theta_t = 0;
	if (Dot(ray.direction(), N) < eps)  // From optically thinner medium to optically denser medium
	{
		ni_nt = 1 / material.RI();
		double cos_theta_i2 = Dot(ray.direction(), N) * Dot(ray.direction(), N);  // cos(\theta_i)^2
		cos_theta_i = sqrt(cos_theta_i2);
		cos_theta_t = sqrt(1 - ni_nt * ni_nt * (1 - cos_theta_i2));
	}
	else  // From optically denser medium to optically thinner medium
	{
		N = -N;
		ni_nt = material.RI();
		double cos_theta_i2 = Dot(ray.direction(), N) * Dot(ray.direction(), N);  // cos(\theta_i)^2
		cos_theta_i = sqrt(cos_theta_i2);
		if (1 - ni_nt * ni_nt * (1 - cos_theta_i2) > eps)  // no total reflection
			cos_theta_t = sqrt(1 - ni_nt * ni_nt * (1 - cos_theta_i2));
	}
	double r_p = (ni_nt * cos_theta_i - cos_theta_t) / (ni_nt * cos_theta_i + cos_theta_t);  // r_parallel
	double r_v = (cos_theta_i - ni_nt * cos_theta_t) / (cos_theta_i + ni_nt * cos_theta_t);  // r_vertical
	return 1 - (r_p * r_p + r_v * r_v) / 2;
}

CColour CBaseObject::colour(CPoint3D pt)  // the colour at the given point (if there is texture)
{
	if (haveTexture == false)
		return m_colour;
	else
	{
		CPoint2D uv = UV_colour(pt);
		cv::Vec3b cur_colour = texture->at<cv::Vec3b>(int(uv.u), int(uv.v));
		return CColour(cur_colour[2], cur_colour[1], cur_colour[0]);
	}
}

void CBaseObject::AddTexture(std::string filename)
{
	texture = new cv::Mat;
	*texture = cv::imread(filename);
	haveTexture = true;
}

void CBaseObject::AddBumpTexture(std::string filename)
{
	bumpTexture = new cv::Mat;
	*bumpTexture = cv::imread(filename);
	haveBumpTexture = true;
}

bool CBaseObject::PointInBox(CBoundingBox box)
{
	if (box == CBoundingBox(inftyPt, inftyPt))
		return true;
	CBoundingBox b = AABB();
	if (b.min().x > box.max().x + eps || b.max().x < box.min().x - eps ||
		b.min().y > box.max().y + eps || b.max().y < box.min().y - eps || 
		b.min().z > box.max().z + eps || b.max().z < box.min().z - eps)
		return false;
	return true;
}


//############################################ class CSphere ############################################
class CSphere : public CBaseObject
{
public:
	CSphere(CPoint3D v_center, double v_radius, CMaterial v_material, CColour v_colour) :
		CBaseObject(v_material, v_colour), center(v_center)
	{
		radius = v_radius;
		m_type = _SPHERE_;
		m_AABB = CBoundingBox(center - CPoint3D(radius, radius, radius), center + CPoint3D(radius, radius, radius));
	}
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray) { return FirstInterPt(ray); }
	CPoint3D InitialNorm(CPoint3D pt) { return (pt - center).Unitize(); }  // Return the normal vector of the given point
protected:
	CPoint2D UV_colour(CPoint3D pt);
	CPoint2D UV_norm(CPoint3D pt);
	CPoint3D Tangent(CPoint3D pt);
	CPoint3D Bitangent(CPoint3D pt);
protected:
	CPoint3D center;
	double radius;
};

CPoint3D CSphere::FirstInterPt(CRay ray)  // Retrun inftyPt if no intersection point
{
	CPoint3D l = center - ray.beginPt();
	double tp = Dot(l, ray.direction());
	double d2 = l.Len2() - tp * tp;
	if (d2 > radius * radius + eps)
		return inftyPt;
	double t_prime = sqrt(radius * radius - d2);
	if (l.Len2() < radius * radius - eps)  // ray.beginPt is inside the sphere
	{
		double t = tp + t_prime;
		return ray.beginPt() + ray.direction() * t;
	}
	else if (l.Len2() > radius * radius + eps)  // ray.beginPt is outside the sphere
	{
		double t = tp - t_prime;
		if (t > eps)
			return ray.beginPt() + ray.direction() * t;
		else
			return inftyPt;
	}
	else  // ray.beginPt is on the sphere
	{
		if (tp < eps)
			return inftyPt;
		else
			return ray.beginPt() + ray.direction() * (tp * 2);
	}
}

CPoint2D CSphere::UV_colour(CPoint3D pt)
{
	if (pt.x == 0 && pt.y == 0)
		return CPoint2D(0, 0);
	double v0 = (atan2((pt - center).y, (pt - center).x) + PI) / (2 * PI);
	double u0 = (asin((pt - center).z / radius) + PI / 2) / PI;
	return CPoint2D(int(u0 * texture->rows), int(v0 * texture->cols));
}

CPoint2D CSphere::UV_norm(CPoint3D pt)
{
	if (pt.x == 0 && pt.y == 0)
		return CPoint2D(0, 0);
	double v0 = (atan2((pt - center).y, (pt - center).x) + PI) / (2 * PI);
	double u0 = (asin((pt - center).z / radius) + PI / 2) / PI;
	return CPoint2D(u0, v0);
}

CPoint3D CSphere::Tangent(CPoint3D pt)
{
	CPoint2D pt2D = UV_norm(pt);
	double u = pt2D.u;
	double v = pt2D.v;
	double dx_dv = -2 * PI * radius * sin(2 * PI * v - PI) * cos(PI * u - PI / 2);
	double dy_dv = 2 * PI * radius * cos(2 * PI * v - PI) * cos(PI * u - PI / 2);
	CPoint3D dP_dv(dx_dv, dy_dv, 0);
	return dP_dv.Unitize();
}

CPoint3D CSphere::Bitangent(CPoint3D pt)
{
	CPoint2D pt2D = UV_norm(pt);
	double u = pt2D.u;
	double v = pt2D.v;
	double dx_du = -PI * radius * cos(2 * PI * v - PI) * sin(PI * u - PI / 2);
	double dy_du = -PI * radius * sin(2 * PI * v - PI) * sin(PI * u - PI / 2);
	double dz_du = PI * radius * cos(PI * u - PI / 2);
	CPoint3D dP_du(dx_du, dy_du, dz_du);
	return dP_du.Unitize();
}

//############################################ class CTriangle ############################################
class CTriangle : public CBaseObject
{
public:
	CTriangle(CPoint3D v_vertex[3], CMaterial v_material, CColour v_colour);
	CTriangle(std::vector<CPoint3D> v_vertex, CMaterial v_material, CColour v_colour);
	CTriangle(std::vector<CPoint3D> v_vertex, std::vector<CPoint3D> v_vertexNorm, CMaterial v_material, CColour v_colour);
	CTriangle(std::vector<CPoint3D> v_vertex, std::vector<CPoint3D> v_vertexNorm, std::vector<CPoint2D> v_textureCoord,
		CMaterial v_material, CColour v_colour);
	CTriangle(CPoint3D v_v0, CPoint3D v_v1, CPoint3D v_v2, CMaterial v_material, CColour v_colour);
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray) { return FirstInterPt(ray); }
	CPoint3D InitialNorm(CPoint3D pt); // Return the normal vector of the given point
	void Reverse() { m_norm = -m_norm; }
	void InitializeAABB();
	CPoint2D UV_colour(CPoint3D pt);  // u, v \in [texture.rows, texture.cols]
	CPoint2D UV_norm(CPoint3D pt);  // u, v \in [0, 1]
protected:
	CPoint3D vertex[3];
	CPoint3D vertexNorm[3];
	CPoint2D textureCoord[3];
	CPoint3D m_norm;
	bool haveVertexNorm;
};

CTriangle::CTriangle(CPoint3D v_vertex[3], CMaterial v_material, CColour v_colour) :
CBaseObject(v_material, v_colour)
{
	for (int i = 0; i < 3; i++)
		vertex[i] = v_vertex[i];
	m_norm = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Unitize();
	for (int i = 0; i < 3; i++)
		vertexNorm[i] = m_norm;
	haveVertexNorm = false;
	m_type = _TRIANGLE_;
	InitializeAABB();
}

CTriangle::CTriangle(std::vector<CPoint3D> v_vertex, CMaterial v_material, CColour v_colour) :
CBaseObject(v_material, v_colour)
{
	for (int i = 0; i < 3; i++)
		vertex[i] = v_vertex[i];
	m_norm = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Unitize();
	for (int i = 0; i < 3; i++)
		vertexNorm[i] = m_norm;
	haveVertexNorm = false;
	m_type = _TRIANGLE_;
	InitializeAABB();
}

CTriangle::CTriangle(std::vector<CPoint3D> v_vertex, std::vector<CPoint3D> v_vertexNorm, CMaterial v_material, CColour v_colour) :
CBaseObject(v_material, v_colour)
{
	for (int i = 0; i < 3; i++)
		vertex[i] = v_vertex[i];
	m_norm = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Unitize();
	for (int i = 0; i < 3; i++)
		vertexNorm[i] = v_vertexNorm[i];
	haveVertexNorm = true;
	m_type = _TRIANGLE_;
	InitializeAABB();
}

CTriangle::CTriangle(std::vector<CPoint3D> v_vertex, std::vector<CPoint3D> v_vertexNorm, std::vector<CPoint2D> v_textureCoord,
	CMaterial v_material, CColour v_colour) :
CBaseObject(v_material, v_colour)
{
	for (int i = 0; i < 3; i++)
		vertex[i] = v_vertex[i];
	m_norm = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Unitize();
	for (int i = 0; i < 3; i++)
		vertexNorm[i] = v_vertexNorm[i];
	for (int i = 0; i < 3; i++)
		textureCoord[i] = v_textureCoord[i];
	haveVertexNorm = true;
	m_type = _TRIANGLE_;
	InitializeAABB();
}

CTriangle::CTriangle(CPoint3D v_v0, CPoint3D v_v1, CPoint3D v_v2, CMaterial v_material, CColour v_colour) :
CBaseObject(v_material, v_colour)
{
	vertex[0] = v_v0;
	vertex[1] = v_v1;
	vertex[2] = v_v2;
	m_norm = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Unitize();
	for (int i = 0; i < 2; i++)
		vertexNorm[i] = m_norm;
	haveVertexNorm = false;
	m_type = _TRIANGLE_;
	InitializeAABB();
}

CPoint3D CTriangle::FirstInterPt(CRay ray)  // Retrun inftyPt if no intersection point
{
	if (abs(Dot(ray.direction(), m_norm)) < eps)
		return inftyPt;
	CPoint3D E1 = vertex[0] - vertex[1];
	CPoint3D E2 = vertex[0] - vertex[2];
	CPoint3D S = vertex[0] - ray.beginPt();
	Matrix3d A1, A2, A3, B;
	A1 << S.x, E1.x, E2.x,
		  S.y, E1.y, E2.y,
		  S.z, E1.z, E2.z;
	A2 << ray.direction().x, S.x, E2.x,
		  ray.direction().y, S.y, E2.y,
		  ray.direction().z, S.z, E2.z;
	A3 << ray.direction().x, E1.x, S.x,
		  ray.direction().y, E1.y, S.y,
		  ray.direction().z, E1.z, S.z;
	B << ray.direction().x, E1.x, E2.x,
		 ray.direction().y, E1.y, E2.y,
		 ray.direction().z, E1.z, E2.z;
	double t = A1.determinant() / B.determinant();
	double belta = A2.determinant() / B.determinant();
	double gamma = A3.determinant() / B.determinant();
	if (t > eps &&
		belta > -eps && belta < 1 + eps &&
		gamma > -eps && gamma < 1 + eps &&
		belta + gamma < 1 + eps)
		return ray.beginPt() + ray.direction() * t;
	else
		return inftyPt;
}

CPoint3D CTriangle::InitialNorm(CPoint3D pt)
{
	if (haveVertexNorm == false)
		return m_norm;
	double S0 = ((vertex[1] - pt) ^ (vertex[2] - pt)).Len();
	double S1 = ((vertex[0] - pt) ^ (vertex[2] - pt)).Len();
	double S2 = ((vertex[0] - pt) ^ (vertex[1] - pt)).Len();
	double S = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Len();
	return vertexNorm[0] * (S0 / S) + vertexNorm[1] * (S1 / S) + vertexNorm[2] * (S2 / S);
}

void CTriangle::InitializeAABB()
{
	double x_min = std::min(std::min(vertex[0].x, vertex[1].x), vertex[2].x);
	double x_max = std::max(std::max(vertex[0].x, vertex[1].x), vertex[2].x);
	double y_min = std::min(std::min(vertex[0].y, vertex[1].y), vertex[2].y);
	double y_max = std::max(std::max(vertex[0].y, vertex[1].y), vertex[2].y);
	double z_min = std::min(std::min(vertex[0].z, vertex[1].z), vertex[2].z);
	double z_max = std::max(std::max(vertex[0].z, vertex[1].z), vertex[2].z);
	m_AABB = CBoundingBox(CPoint3D(x_min - eps, y_min - eps, z_min - eps), CPoint3D(x_max + eps, y_max + eps, z_max + eps));
}

CPoint2D CTriangle::UV_colour(CPoint3D pt)  // u, v \in [texture.rows, texture.cols]
{
	double S0 = ((vertex[1] - pt) ^ (vertex[2] - pt)).Len();
	double S1 = ((vertex[0] - pt) ^ (vertex[2] - pt)).Len();
	double S2 = ((vertex[0] - pt) ^ (vertex[1] - pt)).Len();
	double S = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Len();
	return textureCoord[0] * (S0 / S) + textureCoord[1] * (S1 / S) + textureCoord[2] * (S2 / S);
}

CPoint2D CTriangle::UV_norm(CPoint3D pt)  // u, v \in [0, 1]
{
	double S0 = ((vertex[1] - pt) ^ (vertex[2] - pt)).Len();
	double S1 = ((vertex[0] - pt) ^ (vertex[2] - pt)).Len();
	double S2 = ((vertex[0] - pt) ^ (vertex[1] - pt)).Len();
	double S = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[0])).Len();
	return textureCoord[0] * (S0 / S) + textureCoord[1] * (S1 / S) + textureCoord[2] * (S2 / S);
}


//############################################ class CPolygon ############################################
class CPolygon : public CBaseObject
{
public:
	CPolygon(std::vector<CPoint3D> v_vertex, CMaterial v_material, CColour v_colour);
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray) { return FirstInterPt(ray); }
	CPoint3D InitialNorm(CPoint3D pt); // Return the normal vector of the given point
	void Reverse() { m_norm = -m_norm; }
	void InitializeAABB();
protected:
	CPoint2D Projection(CPoint3D pt, CPoint3D interPt);
protected:
	std::vector<CPoint3D> vertex;
	std::vector<CPoint3D> vertexNorm;
	int vertexNum;
	CPoint3D m_norm;
	bool haveVertexNorm;
};

CPolygon::CPolygon(std::vector<CPoint3D> v_vertex, CMaterial v_material, CColour v_colour):
vertex(v_vertex), CBaseObject(v_material, v_colour)
{
	vertexNum = vertex.size();
	m_norm = ((vertex[1] - vertex[0]) ^ (vertex[2] - vertex[1])).Unitize();
	m_type = _POLYGON_;
	InitializeAABB();
	haveVertexNorm = false;
}

CPoint3D CPolygon::FirstInterPt(CRay ray)  // Retrun inftyPt if no intersection point
{
	CPoint3D interPt = inftyPt;
	// Compute the intersection point with the plane
	if (abs(Dot(m_norm, ray.direction())) < eps)
	{
		if (abs(Dot(vertex[0] - ray.beginPt(), m_norm)) < eps)  // The eye is looking directly at the "lateral" of the plane
			return ray.beginPt();
		else
			return inftyPt;
	}
	double D = -Dot(vertex[0], m_norm);
	double t = -(D + Dot(m_norm, ray.beginPt())) / Dot(m_norm, ray.direction());
	if (t < eps)
		return inftyPt;
	else
		interPt = ray.beginPt() + ray.direction() * t;
	// Judge whether the intersection point lies inside the polygon
	if (0.8 < interPt.y && interPt.y < 1.2 && 5 < interPt.x && interPt.x < 5.2)
		interPt = interPt;
	int angleSum = 0;
	for (int i = 0; i < vertex.size(); i++)
	{
		CPoint2D vi = Projection(vertex[i], interPt);
		CPoint2D vj;
		if (i < vertex.size() - 1)
			vj = Projection(vertex[i + 1], interPt);
		else
			vj = Projection(vertex[0], interPt);
		if ((vi.u >= 0 && vi.v > 0 && vj.u >= 0 && vj.v > 0) ||  // I to I
			(vi.u < 0 && vi.v >= 0 && vj.u < 0 && vj.v >= 0) ||  // II to II
			(vi.u <= 0 && vi.v < 0 && vj.u <= 0 && vj.v < 0) ||  // III to III
			(vi.u > 0 && vi.v <= 0 && vj.u > 0 && vj.v <= 0))    // IV to IV
			continue;
		else if ((vi.u >= 0 && vi.v > 0 && vj.u < 0 && vj.v >= 0) ||  // I to II
				 (vi.u < 0 && vi.v >= 0 && vj.u <= 0 && vj.v < 0) ||  // II to III
				 (vi.u <= 0 && vi.v < 0 && vj.u > 0 && vj.v <= 0) ||  // III to IV
				 (vi.u > 0 && vi.v <= 0 && vj.u >= 0 && vj.v > 0))    // IV to I
			angleSum++;  // *(PI/2)
		else if ((vi.u >= 0 && vi.v > 0 && vj.u <= 0 && vj.v < 0) ||  // I to III
				 (vi.u < 0 && vi.v >= 0 && vj.u > 0 && vj.v <= 0) ||  // II to IV
				 (vi.u <= 0 && vi.v < 0 && vj.u >= 0 && vj.v > 0) ||  // III to I
				 (vi.u > 0 && vi.v <= 0 && vj.u < 0 && vj.v >= 0))    // IV to II
		{
			double f = vj.v * vi.u - vj.u * vi.v;
			if (abs(f) < eps)  // f = 0, through the origin (The intersection point is on the edge)
				return interPt;
			else if (f > eps)  // f > 0
				angleSum += 2;  // *(PI/2)
			else  // f < 0
				angleSum -= 2;  // *(PI/2)
		}
		else  // I to IV, II to I, III to II, IV to III
			angleSum--;  // *(PI/2)
	}
	if (abs(angleSum) == 4)  // *(PI/2)
		return interPt;
	else if (abs(angleSum) == 2)  // *(PI/2)
		return interPt;
	else  // angleSum = 0
		return inftyPt;
}

CPoint3D CPolygon::InitialNorm(CPoint3D pt)
{
	if (haveVertexNorm == false)
		return m_norm;
	CPoint3D N(0, 0, 0);
	double d = 0;
	CPoint3D center(0, 0, 0);
	for (int i = 0; i < vertexNum; i++)
		center += vertex[i];
	center /= vertexNum;
	double dn = pow(Dist(center, pt), 2.5 );
	if (dn < eps)
		return m_norm;
	d += 1 / dn;
	for (int i = 0; i < vertexNum; i++)
	{
		double di = pow(Dist(pt, vertex[i]), 2.5);
		if (di < eps)
			return vertexNorm[i];
		d += 1 / di;
	}
	for (int i = 0; i < vertexNum; i++)
		N += vertexNorm[i] * ((1 / pow(Dist(pt, vertex[i]), 2.5)) / d);
	N += m_norm * ((1 / pow(Dist(pt, center), 2.5)) / d);
	return N.Unitize();
}

CPoint2D CPolygon::Projection(CPoint3D pt, CPoint3D interPt)
{
	if (abs(m_norm.z) > 0.5)
		return CPoint2D((pt - interPt).x, (pt - interPt).y);
	else if (abs(m_norm.y) > 0.5)
		return CPoint2D((pt - interPt).x, (pt - interPt).z);
	else
		return CPoint2D((pt - interPt).y, (pt - interPt).z);
}

void CPolygon::InitializeAABB()
{
	double x_min = infty;
	double x_max = -infty;
	double y_min = infty;
	double y_max = -infty;
	double z_min = infty;
	double z_max = -infty;
	for (std::vector<CPoint3D>::iterator vIter = vertex.begin(); vIter != vertex.end(); vIter++)
	{
		if (vIter->x < x_min)
			x_min = vIter->x;
		if (vIter->x > x_max)
			x_max = vIter->x;
		if (vIter->y < y_min)
			y_min = vIter->y;
		if (vIter->y > y_max)
			y_max = vIter->y;
		if (vIter->z < z_min)
			z_min = vIter->z;
		if (vIter->z > z_max)
			z_max = vIter->z;
	}
	m_AABB = CBoundingBox(CPoint3D(x_min, y_min, z_min), CPoint3D(x_max, y_max, z_max));
}


//############################################ class CCircle ############################################
class CCircle : public CBaseObject
{
public:
	CCircle(CPoint3D v_p0, CPoint3D v_normal, double v_radius, CMaterial v_material, CColour v_colour) :
		CBaseObject(v_material, v_colour), p0(v_p0), normal(v_normal.Unitize())
	{
		radius = v_radius;
		m_type = _CIRCLE_;
	}
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray) { return FirstInterPt(ray); }
	CPoint3D InitialNorm(CPoint3D pt) { return normal; }  // Return the normal vector of the given point
protected:
	CPoint3D p0;
	CPoint3D normal;
	double radius;
};

CPoint3D CCircle::FirstInterPt(CRay ray)
{
	if (abs(Dot(normal, ray.direction())) < eps)
		return inftyPt;
	double D = -Dot(p0, normal);
	double t = -(D + Dot(normal, ray.beginPt())) / Dot(normal, ray.direction());
	CPoint3D pt = ray.beginPt() + ray.direction() * t;
	if (t < eps || Dist(pt, p0) > radius - eps)
		return inftyPt;
	else
		return pt;
}


//############################################ class CInftyPlane ############################################
class CInftyPlane : public CBaseObject
{
public:
	CInftyPlane(CPoint3D v_p0, CPoint3D v_normal, CMaterial v_material, CColour v_colour) :
		CBaseObject(v_material, v_colour), p0(v_p0), normal(v_normal.Unitize())
	{
		m_type = _INFTY_PLANE_;
	}
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray) { return FirstInterPt(ray); }
	CPoint3D InitialNorm(CPoint3D pt) { return normal; }  // Return the normal vector of the given point
	void AddTexture(std::string filename, double v_textureScale);
	void AddBumpTexture(std::string filename, double v_bumpTextureScale);
protected:
	CPoint2D UV_colour(CPoint3D pt);
	CPoint2D UV_norm(CPoint3D pt);
	CPoint3D CInftyPlane::Tangent(CPoint3D pt);
	CPoint3D CInftyPlane::Bitangent(CPoint3D pt) { return normal ^ Tangent(pt); }
protected:
	CPoint3D p0;
	CPoint3D normal;
	double textureScale;  // how many pixels in the texture does a unit length match
	double bumpTextureScale;
};

CPoint3D CInftyPlane::FirstInterPt(CRay ray)  // Retrun inftyPt if no intersection point
{
	if (abs(Dot(normal, ray.direction())) < eps)
	{
		if (abs(Dot(p0 - ray.beginPt(), normal)) < eps)  // The eye is looking directly at the "lateral" of the plane
			return ray.beginPt();
		else
			return inftyPt;
	}
	double D = -Dot(p0, normal);
	double t = -(D + Dot(normal, ray.beginPt())) / Dot(normal, ray.direction());
	if (t < eps)
		return inftyPt;
	else
		return ray.beginPt() + ray.direction() * t;
}

CPoint2D CInftyPlane::UV_colour(CPoint3D pt)
{
	CPoint3D a(1, 0, 0), b(0, 1, 0), c(0, 0, 1);
	CPoint3D u0, v0;
	if (abs(Dot(a, normal)) < 0.5)
		u0 = (a - normal * Dot(a, normal)).Unitize();
	else if (abs(Dot(b, normal)) < 0.5)
		u0 = (b - normal * Dot(b, normal)).Unitize();
	else
		u0 = (c - normal * Dot(c, normal)).Unitize();
	v0 = normal ^ u0;
	CPoint2D pt2D(Dot(pt - p0, u0), Dot(pt - p0, v0));
	int uPos = int(pt2D.u * textureScale) % texture->rows;
	int vPos = int(pt2D.v * textureScale) % texture->cols;
	uPos = (uPos + texture->rows) % texture->rows;
	vPos = (vPos + texture->cols) % texture->cols;
	return CPoint2D(uPos, vPos);
}

CPoint2D CInftyPlane::UV_norm(CPoint3D pt)
{
	CPoint3D a(1, 0, 0), b(0, 1, 0), c(0, 0, 1);
	CPoint3D u0, v0;
	if (abs(Dot(a, normal)) < 0.5)
		u0 = (a - normal * Dot(a, normal)).Unitize();
	else if (abs(Dot(b, normal)) < 0.5)
		u0 = (b - normal * Dot(b, normal)).Unitize();
	else
		u0 = (c - normal * Dot(c, normal)).Unitize();
	v0 = normal ^ u0;
	CPoint2D pt2D(Dot(pt - p0, u0), Dot(pt - p0, v0));
	pt2D /= bumpTextureScale;
	return CPoint2D(pt2D.u - floor(pt2D.u), pt2D.v - floor(pt2D.v));
}

CPoint3D CInftyPlane::Tangent(CPoint3D pt)
{
	CPoint3D a(1, 0, 0), b(0, 1, 0), c(0, 0, 1);
	CPoint3D u0, v0;
	if (abs(Dot(a, normal)) < 0.5)
		u0 = (a - normal * Dot(a, normal)).Unitize();
	else if (abs(Dot(b, normal)) < 0.5)
		u0 = (b - normal * Dot(b, normal)).Unitize();
	else
		u0 = (c - normal * Dot(c, normal)).Unitize();
	return u0;
}

void CInftyPlane::AddTexture(std::string filename, double v_textureScale)
{
	texture = new cv::Mat;
	*texture = cv::imread(filename);
	haveTexture = true;
	textureScale = v_textureScale;
}

void CInftyPlane::AddBumpTexture(std::string filename, double v_bumpTextureScale)
{
	bumpTexture = new cv::Mat;
	*bumpTexture = cv::imread(filename);
	haveBumpTexture = true;
	bumpTextureScale = v_bumpTextureScale;
}


//############################################ class CRatationalObject ############################################
class CRotationalObject : public CBaseObject
{
public:
	CRotationalObject(CBezierCurve v_bezierCurve, CPoint3D v_center, CMaterial v_material, CColour v_colour,
		int v_uSampleNum, double v_uRange);
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray);
	CPoint3D InitialNorm(CPoint3D pt) { return surface.Norm(cur_u, cur_v); }  // Return the normal vector of paramaters (t, theta)
protected:
	CRotationalSurface surface;
	CPoint3D center;
	double uRange;
	int uSampleNum;  // u belongs to [0, 1]
protected:  // Record the paramaters corresponding to the latest intersection point
	double cur_u;
	double cur_v;
	double cur_t;
protected:
	CPoint2D UV_colour(CPoint3D pt) { return CPoint2D(int(cur_u * texture->rows), int(cur_v * texture->cols / (2 * PI))); }
	CPoint2D UV_norm(CPoint3D pt) { return CPoint2D(cur_u, cur_v / (2 * PI)); }
	CPoint3D Tangent(CPoint3D pt) { return surface.dS_dtheta(cur_u, cur_v).Unitize(); }
	CPoint3D Bitangent(CPoint3D pt) { return surface.dS_dt(cur_u, cur_v).Unitize(); }
	void InitializeAABB();
	CPoint2D t_v_Initial(double u, CRay ray);  // initial values for Newton's iteration
};

CRotationalObject::CRotationalObject(CBezierCurve v_bezierCurve, CPoint3D v_center, CMaterial v_material, CColour v_colour,
	int v_uSampleNum, double v_uRange = 1) :
surface(v_bezierCurve), center(v_center), CBaseObject(v_material, v_colour)
{
	uSampleNum = v_uSampleNum;
	uRange = v_uRange;
	cur_u = infty;
	cur_v = infty;
	cur_t = infty;
	m_type = _ROTATIONAL_OBJECT_;
	InitializeAABB();
}

const int maxStepNum = 10;
const double minStepLen = eps;
const double maxDeviation = 0.3;
CPoint3D CRotationalObject::FirstInterPt(CRay ray)  // Retrun inftyPt if no intersection point
{
	if (m_AABB.Collide(ray) == inftyPt2D)
		return inftyPt;
	cur_u = infty;
	cur_v = infty;
	cur_t = infty;
	bool haveInterPt = false;
	// Compute the intersection point using Newton's iteration
	double uSampleInterval = uRange / double(uSampleNum);
	for (int i = 0; i <= uSampleNum; i++)  // different initial values
	{
		double u = i * uSampleInterval;
		CPoint2D t_v = t_v_Initial(u, ray);
		if (t_v == inftyPt2D)
			continue;
		double v = t_v.v;
		double t = t_v.u;
		CPoint3D w = ray.direction();
		for (int step = 0; step < maxStepNum; step++)  // iterations
		{
			CPoint3D dP_du = surface.dS_dt(u, v);
			CPoint3D dP_dv = surface.dS_dtheta(u, v);
			double D = Dot(w, dP_du ^ dP_dv);
			// df(x_n)(x_n+1 - x_n) = -f(x_n)
			CPoint3D df = center + surface.Point3D(u, v) - (ray.beginPt() + ray.direction() * t);
			double t_prime = t + Dot(dP_du, dP_dv ^ df) / D;
			double u_prime = u + Dot(w, dP_dv ^ df) / D;
			double v_prime = v - Dot(w, dP_du ^ df) / D;
			if (u_prime < -maxDeviation || u_prime > 1 + maxDeviation ||
				//v_prime < -maxDeviation || v_prime > 2 * PI + maxDeviation ||
				t_prime < -maxDeviation)
				break;
			if (CPoint3D(t_prime - t, u_prime - u, v_prime - v).Len2() < minStepLen
				&& t_prime > eps * 10
				&& 0 < u_prime && u_prime < uRange
				//&& -eps < v_prime && v_prime < 2 * PI + eps
				)
			{
				haveInterPt = true;
				// Judge whether current intersection point is the nearer than the previous one
				if (t_prime < cur_t)
				{
					cur_u = u_prime;
					cur_v = v_prime;
					cur_t = t_prime;
					break;
				}
			}
			u = u_prime;
			v = v_prime;
			t = t_prime;
		}
	}
	if (haveInterPt == true)
		return center + surface.Point3D(cur_u, cur_v);
	else
		return inftyPt;
}

CPoint3D CRotationalObject::FirstInterPt_Unrecorded(CRay ray)
{
	if (m_AABB.Collide(ray) == inftyPt2D)
		return inftyPt;
	double _u_ = infty;
	double _v_ = infty;
	double _t_ = infty;
	bool haveInterPt = false;
	// Compute the intersection point using Newton's iteration
	double uSampleInterval = uRange / double(uSampleNum);
	for (int i = 0; i <= uSampleNum; i++)  // different initial values
	{
		double u = i * uSampleInterval;
		CPoint2D t_v = t_v_Initial(u, ray);
		if (t_v == inftyPt2D)
			continue;
		double v = t_v.v;
		double t = t_v.u;
		CPoint3D w = ray.direction();
		for (int step = 0; step < maxStepNum; step++)  // iterations
		{
			CPoint3D dP_du = surface.dS_dt(u, v);
			CPoint3D dP_dv = surface.dS_dtheta(u, v);
			double D = Dot(w, dP_du ^ dP_dv);
			// df(x_n)(x_n+1 - x_n) = -f(x_n)
			CPoint3D df = center + surface.Point3D(u, v) - (ray.beginPt() + ray.direction() * t);
			double t_prime = t + Dot(dP_du, dP_dv ^ df) / D;
			double u_prime = u + Dot(w, dP_dv ^ df) / D;
			double v_prime = v - Dot(w, dP_du ^ df) / D;
			if (u_prime < -maxDeviation || u_prime > 1 + maxDeviation ||
				//v_prime < -maxDeviation || v_prime > 2 * PI + maxDeviation ||
				t_prime < -maxDeviation)
				break;
			if (CPoint3D(t_prime - t, u_prime - u, v_prime - v).Len2() < minStepLen
				&& t_prime > eps * 5000
				&& 0 < u_prime && u_prime < uRange
				//&& -eps < v_prime && v_prime < 2 * PI + eps
				)
			{
				haveInterPt = true;
				// Judge whether current intersection point is the nearer than the previous one
				if (t_prime < _t_)
				{
					_u_ = u_prime;
					_v_ = v_prime;
					_t_ = t_prime;
					break;
				}
			}
			u = u_prime;
			v = v_prime;
			t = t_prime;
		}
	}
	if (haveInterPt == true)
		return center + surface.Point3D(_u_, _v_);
	else
		return inftyPt;
}

void CRotationalObject::InitializeAABB()
{
	double z_max = center.z + std::max(surface.bezierCur.controlPts.begin()->v, (surface.bezierCur.controlPts.end() - 1)->v);
	double z_min = center.z + std::min(surface.bezierCur.controlPts.begin()->v, (surface.bezierCur.controlPts.end() - 1)->v);
	double r = 0;
	for (std::vector<CPoint2D>::iterator ptIter = surface.bezierCur.controlPts.begin();
		ptIter != surface.bezierCur.controlPts.end(); ptIter++)
	{
		if (ptIter->u > r)
			r = ptIter->u;
	}
	double x_max = center.x + r;
	double x_min = center.x - r;
	double y_max = center.y + r;
	double y_min = center.y - r;
	m_AABB = CBoundingBox(CPoint3D(x_min - eps, y_min - eps, z_min - eps), CPoint3D(x_max + eps, y_max + eps, z_max + eps));
}

CPoint2D CRotationalObject::t_v_Initial(double u, CRay ray)  // initial values for Newton's iteration
{
	CPoint2D pt2D = surface.bezierCur.Point2D(u);
	CPoint3D p0(center.x, center.y, center.z + pt2D.v);
	CPoint3D n(0, 0, 1);
	double v = 0;
	
	if (abs(Dot(n, ray.direction())) < 0.1)
	{
		// intersection point with the sphere
		CPoint3D p;
		double t = 0;
		CPoint3D l = p0 - ray.beginPt();
		double tp = Dot(l, ray.direction());
		double d2 = l.Len2() - tp * tp;
		if (d2 > pt2D.u * pt2D.u + eps)
			return inftyPt2D;
		double t_prime = sqrt(pt2D.u * pt2D.u - d2);
		if (l.Len2() < pt2D.u * pt2D.u - eps * 1000)  // ray.beginPt is inside the sphere
		{
			t = tp + t_prime;
			p = ray.beginPt() + ray.direction() * t;
		}
		else if (l.Len2() > pt2D.u * pt2D.u + eps * 1000)  // ray.beginPt is outside the sphere
		{
			t = tp - t_prime;
			if (t > eps)
				p = ray.beginPt() + ray.direction() * t;
			else
				return inftyPt2D;
		}
		else  // ray.beginPt is on the sphere
		{
			if (tp < eps * 1000)
				return inftyPt2D;
			else
			{
				t = tp * 2;
				p = ray.beginPt() + ray.direction() * (tp * 2);
			}	
		}
		double cos_theta = (p - p0).x / (p - p0).Len();
		double theta = acos(cos_theta);
		if (p.y > p0.y || (p.y == p0.y && p.x > p0.x))
			v = theta;
		else
			v = 2 * PI - theta;
		return CPoint2D(t, v);
	}
	double D = -Dot(p0, n);
	double t = -(D + Dot(n, ray.beginPt())) / Dot(n, ray.direction());
	CPoint3D p = ray.beginPt() + ray.direction() * t;
	if (t < eps)
		return inftyPt2D;
	else
	{
		// intersection point with the plane
		double cos_theta = (p - p0).x / (p - p0).Len();
		double theta = acos(cos_theta);
		if (p.y > p0.y || (p.y == p0.y && p.x > p0.x))
			v = theta;
		else
			v = 2 * PI - theta;
		return CPoint2D(t, v);
	}
}


//############################################ class CBezierObject ############################################
class CBezierObject : public CBaseObject
{
public:
	CBezierObject(CBezierSurface v_surface, CPoint3D v_center, CMaterial v_material, CColour v_colour);
	CBezierObject(CPoint3D v_controlPts[4][4], CPoint3D v_center, CMaterial v_material, CColour v_colour);
public:
	CPoint3D FirstInterPt(CRay ray);  // Retrun inftyPt if no intersection point
	CPoint3D FirstInterPt_Unrecorded(CRay ray);
	CPoint3D InitialNorm(CPoint3D pt) { return surface.Norm(cur_u, cur_v); }
protected:
	CPoint2D UV_colour(CPoint3D pt) { return CPoint2D(int(cur_u * texture->rows), int(cur_v * texture->cols)); }
	CPoint2D UV_norm(CPoint3D pt) { return CPoint2D(cur_u, cur_v); }
	CPoint3D Tangent(CPoint3D pt) { return surface.dS_du(cur_u, cur_v).Unitize(); }
	CPoint3D Bitangent(CPoint3D pt) { return surface.dS_dv(cur_u, cur_v).Unitize(); }
	void InitializeAABB();
protected:
	CBezierSurface surface;
	CPoint3D center;
protected:  // Record the paramaters corresponding to the latest intersection point
	double cur_u;
	double cur_v;
	double cur_t;
};

CBezierObject::CBezierObject(CBezierSurface v_surface, CPoint3D v_center, CMaterial v_material, CColour v_colour) :
surface(v_surface), center(v_center), CBaseObject(v_material, v_colour)
{
	cur_u = infty;
	cur_v = infty;
	cur_t = infty;
	m_type = _BEZIER_OBJECT_;
	InitializeAABB();
}

CBezierObject::CBezierObject(CPoint3D v_controlPts[4][4], CPoint3D v_center, CMaterial v_material, CColour v_colour) :
surface(v_controlPts), center(v_center), CBaseObject(v_material, v_colour)
{
	cur_u = infty;
	cur_v = infty;
	cur_t = infty;
	m_type = _BEZIER_OBJECT_;
	InitializeAABB();
}

const int uSampleNum2 = 4;  // u belongs to [0, 1]
const int vSampleNum2 = 4;  // v belongs to [0, 1]
CPoint3D CBezierObject::FirstInterPt(CRay ray)  // Retrun inftyPt if no intersection point
{
	cur_u = infty;
	cur_v = infty;
	cur_t = infty;
	bool haveInterPt = false;
	// Compute the intersection point using Newton's iteration
	double uSampleInterval = 1 / double(uSampleNum2);
	double vSampleInterval = 1 / double(vSampleNum2);
	for (int i = 0; i <= uSampleNum2; i++)  // different initial values
	{
		for (int j = 0; j <= vSampleNum2; j++)
		{
			double u = i * uSampleInterval;
			double v = j * vSampleInterval;
			double t = Dist(surface.Point3D(u, v) + center, ray.beginPt());
			CPoint3D w = ray.direction();
			for (int step = 0; step < maxStepNum; step++)  // iterations
			{
				CPoint3D pt = surface.Point3D(u, v);
				CPoint3D dP_du = surface.dS_du(u, v);
				CPoint3D dP_dv = surface.dS_dv(u, v);
				double D = Dot(w, dP_du ^ dP_dv);
				// df(x_n)(x_n+1 - x_n) = -f(x_n)
				CPoint3D df = center + surface.Point3D(u, v) - (ray.beginPt() + ray.direction() * t);
				double t_prime = t + Dot(dP_du, dP_dv ^ df) / D;
				double u_prime = u + Dot(w, dP_dv ^ df) / D;
				double v_prime = v - Dot(w, dP_du ^ df) / D;
				if (u_prime < -maxDeviation || u_prime > 1 + maxDeviation ||
					v_prime < -maxDeviation || v_prime > 1 + maxDeviation ||
					t_prime < -maxDeviation)
					break;
				if (CPoint3D(t_prime - t, u_prime - u, v_prime - v).Len2() < minStepLen
					&& t_prime > eps
					&& 0 < u_prime && u_prime < 1
					&& 0 < v_prime && v_prime < 1)
				{
					haveInterPt = true;
					// Judge whether current intersection point is the nearer than the previous one
					if (t_prime < cur_t - eps)
					{
						cur_u = u_prime;
						cur_v = v_prime;
						cur_t = t_prime;
						break;
					}
				}
				u = u_prime;
				v = v_prime;
				t = t_prime;
			}
		}
	}
	if (haveInterPt == true)
		return center + surface.Point3D(cur_u, cur_v);
	else
		return inftyPt;
}

CPoint3D CBezierObject::FirstInterPt_Unrecorded(CRay ray)
{
	double _u_ = infty;
	double _v_ = infty;
	double _t_ = infty;
	bool haveInterPt = false;
	// Compute the intersection point using Newton's iteration
	double uSampleInterval = 1 / double(uSampleNum2);
	double vSampleInterval = 1 / double(vSampleNum2);
	for (int i = 0; i <= uSampleNum2; i++)  // different initial values
	{
		for (int j = 0; j <= vSampleNum2; j++)
		{
			double u = i * uSampleInterval;
			double v = j * vSampleInterval;
			double t = Dist(surface.Point3D(u, v) + center, ray.beginPt());
			CPoint3D w = ray.direction();
			for (int step = 0; step < maxStepNum; step++)  // iterations
			{
				CPoint3D dP_du = surface.dS_du(u, v);
				CPoint3D dP_dv = surface.dS_dv(u, v);
				double D = Dot(w, dP_du ^ dP_dv);
				// df(x_n)(x_n+1 - x_n) = -f(x_n)
				CPoint3D df = center + surface.Point3D(u, v) - (ray.beginPt() + ray.direction() * t);
				double t_prime = t + Dot(dP_du, dP_dv ^ df) / D;
				double u_prime = u + Dot(w, dP_dv ^ df) / D;
				double v_prime = v - Dot(w, dP_du ^ df) / D;
				if (u_prime < -maxDeviation || u_prime > 1 + maxDeviation ||
					v_prime < -maxDeviation || v_prime > 1 + maxDeviation ||
					t_prime < -maxDeviation)
					break;
				if (CPoint3D(t_prime - t, u_prime - u, v_prime - v).Len2() < minStepLen
					&& t_prime > eps
					&& 0 < u_prime && u_prime < 1
					&& 0 < v_prime && v_prime < 1)
				{
					haveInterPt = true;
					// Judge whether current intersection point is the nearer than the previous one
					if (t_prime < _t_ - eps)
					{
						_u_ = u_prime;
						_v_ = v_prime;
						_t_ = t_prime;
						break;
					}
				}
				u = u_prime;
				v = v_prime;
				t = t_prime;
			}
		}
	}
	if (haveInterPt == true)
		return center + surface.Point3D(_u_, _v_);
	else
		return inftyPt;
}

void CBezierObject::InitializeAABB()
{
	double x_min = infty, y_min = infty, z_min = infty;
	double x_max = -infty, y_max = -infty, z_max = -infty;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (surface.controlPts[i][j].x < x_min)
				x_min = surface.controlPts[i][j].x;
			if (surface.controlPts[i][j].y < y_min)
				y_min = surface.controlPts[i][j].y;
			if (surface.controlPts[i][j].z < z_min)
				z_min = surface.controlPts[i][j].z;
			if (surface.controlPts[i][j].x > x_max)
				x_max = surface.controlPts[i][j].x;
			if (surface.controlPts[i][j].y > y_max)
				y_max = surface.controlPts[i][j].y;
			if (surface.controlPts[i][j].z > z_max)
				z_max = surface.controlPts[i][j].z;
		}
	}
	m_AABB = CBoundingBox(CPoint3D(x_min, y_min, z_min) + center, CPoint3D(x_max, y_max, z_max) + center);
}

#endif  // _OBJECT_H_