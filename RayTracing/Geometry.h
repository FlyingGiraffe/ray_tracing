#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <Eigen>
#include <omp.h>

using namespace Eigen;

const double eps = 1e-8;
const double PI = 3.1415926535;

//############################################ class CPoint3D ############################################
class CPoint3D
{
public:
	CPoint3D() { x = 0; y = 0; z = 0; }
	CPoint3D(double v_x, double v_y, double v_z) { x = v_x; y = v_y; z = v_z; }
public:
	double x, y, z;
public:
	CPoint3D operator + (const CPoint3D & pt) const { return CPoint3D(x + pt.x, y + pt.y, z + pt.z); }
	CPoint3D operator - (const CPoint3D & pt) const { return CPoint3D(x - pt.x, y - pt.y, z - pt.z); }
	CPoint3D operator * (const double & lambda) const { return CPoint3D(x * lambda, y * lambda, z * lambda); }
	CPoint3D operator / (const double & lambda) const { return CPoint3D(x / lambda, y / lambda, z / lambda); }
	CPoint3D operator ^ (const CPoint3D & pt) const;  // (as vector) wedge product
	CPoint3D & operator += (const CPoint3D & pt);
	CPoint3D & operator -= (const CPoint3D & pt);
	CPoint3D & operator *= (const double & lambda);  // scalar multiplication
	CPoint3D & operator /= (const double & lambda);
	CPoint3D & operator - ();
	bool operator == (const CPoint3D & pt);
	bool operator != (const CPoint3D & pt);
	double Len() { return Dist(*this, CPoint3D(0, 0, 0)); }  // (as vector)
	double Len2() { return Dist2(*this, CPoint3D(0, 0, 0)); }  // (as vector) square of length
	CPoint3D Unitize();  // (as vector)
	bool Compare_x(CPoint3D pt1, CPoint3D pt2) { return (pt1.x < pt2.x); }
	bool Compare_y(CPoint3D pt1, CPoint3D pt2) { return (pt1.y < pt2.y); }
	bool Compare_z(CPoint3D pt1, CPoint3D pt2) { return (pt1.z < pt2.z); }

	friend double Dot(CPoint3D pt1, CPoint3D pt2);  // (as vector) dot product
	friend double Dist(CPoint3D pt1, CPoint3D pt2);
	friend double Dist2(CPoint3D pt1, CPoint3D pt2);  // square of distance
};

const double infty = 1e+8;
const CPoint3D inftyPt(infty, infty, infty);

CPoint3D CPoint3D::operator ^ (const CPoint3D & pt) const  // wedge product
{
	return CPoint3D(y * pt.z - z * pt.y,
					z * pt.x - x * pt.z,
					x * pt.y - y * pt.x);
}

CPoint3D & CPoint3D::operator += (const CPoint3D & pt)
{
	x += pt.x;
	y += pt.y;
	z += pt.z;
	return *this;
}

CPoint3D & CPoint3D::operator -= (const CPoint3D & pt)
{
	x -= pt.x;
	y -= pt.y;
	z -= pt.z;
	return *this;
}

CPoint3D & CPoint3D::operator *= (const double & lambda)
{
	x *= lambda;
	y *= lambda;
	z *= lambda;
	return *this;
}

CPoint3D & CPoint3D::operator /= (const double & lambda)
{
	x /= lambda;
	y /= lambda;
	z /= lambda;
	return *this;
}

CPoint3D & CPoint3D::operator - ()
{
	CPoint3D pt(-x, -y, -z);
	CPoint3D & pt_ref = pt;
	return pt_ref;
}

bool CPoint3D::operator == (const CPoint3D & pt)
{
	if (abs(x - pt.x) < eps && abs(y - pt.y) < eps && abs(z - pt.z) < eps)
		return true;
	else
		return false;
}

bool CPoint3D::operator != (const CPoint3D & pt)
{
	if (abs(x - pt.x) < eps && abs(y - pt.y) < eps && abs(z - pt.z) < eps)
		return false;
	else
		return true;
}

CPoint3D CPoint3D::Unitize()  // (as vector)
{
	if (*this == CPoint3D(0, 0, 0) || *this == inftyPt)
		return inftyPt;
	else
		return CPoint3D(x / Len(), y / Len(), z / Len());
}

double Dot(CPoint3D pt1, CPoint3D pt2)  // (friend, as vector) dot product
{
	return pt1.x * pt2.x + pt1.y * pt2.y + pt1.z * pt2.z;
}

double Dist(CPoint3D pt1, CPoint3D pt2)  // (friend, as vector)
{
	return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) +
		(pt1.y - pt2.y) * (pt1.y - pt2.y) +
		(pt1.z - pt2.z) * (pt1.z - pt2.z));
}

double Dist2(CPoint3D pt1, CPoint3D pt2)  // (friend) square of distance
{
	return (pt1.x - pt2.x) * (pt1.x - pt2.x) +
		(pt1.y - pt2.y) * (pt1.y - pt2.y) +
		(pt1.z - pt2.z) * (pt1.z - pt2.z);
}


//############################################ class CPoint2D ############################################
class CPoint2D
{
public:
	CPoint2D() { u = 0; v = 0; }
	CPoint2D(double v_u, double v_v) { u = v_u; v = v_v;}
public:
	double u, v;
public:
	CPoint2D operator + (const CPoint2D & pt2D) const { return CPoint2D(u + pt2D.u, v + pt2D.v); }
	CPoint2D operator - (const CPoint2D & pt2D) const { return CPoint2D(u - pt2D.u, v - pt2D.v); }
	CPoint2D operator * (const double & lambda) const { return CPoint2D(u * lambda, v * lambda); }
	CPoint2D operator / (const double & lambda) const { return CPoint2D(u / lambda, v / lambda); }
	CPoint2D & operator += (const CPoint2D & pt2D);
	CPoint2D & operator -= (const CPoint2D & pt2D);
	CPoint2D & operator *= (const double & lambda);  // scalar multiplication
	CPoint2D & operator /= (const double & lambda);
	CPoint2D & operator - ();
	bool operator == (const CPoint2D & pt2D);
	bool operator != (const CPoint2D & pt2D);
	double Len() { return Dist(*this, CPoint2D(0, 0)); }  // (as vector)
	double Len2() { return Dist2(*this, CPoint2D(0, 0)); }  // (as vector) square of length
	CPoint2D Unitize();  // (as vector)
	friend double Dot(CPoint2D pt2D1, CPoint2D pt2D2);  // (as vector) dot product
	friend double Dist(CPoint2D pt2D1, CPoint2D pt2D2);
	friend double Dist2(CPoint2D pt2D1, CPoint2D pt2D2);  // square of distance
};

const CPoint2D inftyPt2D(infty, infty);

CPoint2D & CPoint2D::operator += (const CPoint2D & pt2D)
{
	u += pt2D.u;
	v += pt2D.v;
	return *this;
}

CPoint2D & CPoint2D::operator -= (const CPoint2D & pt2D)
{
	u -= pt2D.u;
	v -= pt2D.v;
	return *this;
}

CPoint2D & CPoint2D::operator *= (const double & lambda)
{
	u *= lambda;
	v *= lambda;
	return *this;
}

CPoint2D & CPoint2D::operator /= (const double & lambda)
{
	u /= lambda;
	v /= lambda;
	return *this;
}

CPoint2D & CPoint2D::operator - ()
{
	CPoint2D pt2D(-u, -v);
	CPoint2D & pt2D_ref = pt2D;
	return pt2D_ref;
}

bool CPoint2D::operator == (const CPoint2D & pt2D)
{
	if (abs(u - pt2D.u) < eps && abs(v - pt2D.v) < eps)
		return true;
	else
		return false;
}

bool CPoint2D::operator != (const CPoint2D & pt2D)
{
	if (abs(u - pt2D.u) < eps && abs(v - pt2D.v) < eps)
		return false;
	else
		return true;
}

CPoint2D CPoint2D::Unitize()  // (as vector)
{
	if (*this == CPoint2D(0, 0) || *this == inftyPt2D)
		return inftyPt2D;
	else
		return CPoint2D(u / Len(), v / Len());
}

double Dot(CPoint2D pt2D1, CPoint2D pt2D2)  // (friend, as vector) dot product
{
	return pt2D1.u * pt2D2.u + pt2D1.v * pt2D2.v;
}

double Dist(CPoint2D pt2D1, CPoint2D pt2D2)  // (friend, as vector)
{
	return sqrt((pt2D1.u - pt2D2.u) * (pt2D1.u - pt2D2.u) +
		(pt2D1.v - pt2D2.v) * (pt2D1.v - pt2D2.v));
}

double Dist2(CPoint2D pt2D1, CPoint2D pt2D2)  // (friend) square of distance
{
	return (pt2D1.u - pt2D2.u) * (pt2D1.u - pt2D2.u) +
		(pt2D1.v - pt2D2.v) * (pt2D1.v - pt2D2.v);
}


//############################################ class CRay ############################################
class CRay
{
public:
	CRay() : m_beginPt(), m_direction() {}
	CRay(CPoint3D v_beginPt, CPoint3D v_direction) :
		m_beginPt(v_beginPt), m_direction(v_direction.Unitize()) {}
protected:
	CPoint3D m_beginPt;
	CPoint3D m_direction;  // a unit vector
public:
	double Dist(CPoint3D pt);  // the distance from the given point to the ray
	double Dist2(CPoint3D pt);  // the distance from the given point to the ray
	CPoint3D beginPt() { return m_beginPt; }
	CPoint3D direction() { return m_direction; }
};

double CRay::Dist(CPoint3D pt)
{
	// Retrun the distance from the given point to the ray
	CPoint3D l = pt - beginPt();
	double tp = Dot(l, direction());
	double d2 = l.Len2() - tp * tp;
	return sqrt(d2);
}

double CRay::Dist2(CPoint3D pt)
{
	// Retrun the distance from the given point to the ray
	CPoint3D l = pt - beginPt();
	double tp = Dot(l, direction());
	double d2 = l.Len2() - tp * tp;
	return d2;
}


//############################################ class CBoundingBox (parallel to the axises) ############################################
class CBoundingBox
{
public:
	CBoundingBox() { m_min = inftyPt; m_max = inftyPt; }
	CBoundingBox(CPoint3D v_min, CPoint3D v_max){ m_min = v_min; m_max = v_max; }
public:
	CPoint3D min() { return m_min; }
	CPoint3D max() { return m_max; }
	CPoint2D Collide(CRay ray);  // Return (t_min, t_max) if there is 1 or 2 intersection point(s), else return inftyPt2D
	bool operator == (const CBoundingBox & box);
	bool operator != (const CBoundingBox & box);
	bool InBox(CPoint3D pt);
protected:
	CPoint3D m_min, m_max;  // min = (xMin, yMin, zMin), max = (xMax, yMax, zMax)
	double IntersectWithPlane(CRay ray, CPoint3D p0, CPoint3D norm);  // intersection point with a plane
};

CPoint2D CBoundingBox::Collide(CRay ray)
{
	double t1 = 0, t2 = 0;  // t's with a slab
	// slab perpendicular to X axis
	double t_x_min = 0, t_x_max = 0;
	t1 = IntersectWithPlane(ray, m_min, CPoint3D(1, 0, 0));
	t2 = IntersectWithPlane(ray, m_max, CPoint3D(1, 0, 0));
	t_x_min = std::min(t1, t2);
	t_x_max = std::max(t1, t2);
	// slab perpendicular to Y axis
	double t_y_min = 0, t_y_max = 0;
	t1 = IntersectWithPlane(ray, m_min, CPoint3D(0, 1, 0));
	t2 = IntersectWithPlane(ray, m_max, CPoint3D(0, 1, 0));
	t_y_min = std::min(t1, t2);
	t_y_max = std::max(t1, t2);
	// slab perpendicular to Z axis
	double t_z_min = 0, t_z_max = 0;
	t1 = IntersectWithPlane(ray, m_min, CPoint3D(0, 0, 1));
	t2 = IntersectWithPlane(ray, m_max, CPoint3D(0, 0, 1));
	t_z_min = std::min(t1, t2);
	t_z_max = std::max(t1, t2);
	double t_max = std::min(std::min(t_x_max, t_y_max), t_z_max);
	// Delete the infinity of t_i_min's
	t_x_min = (t_x_min == infty) ? -infty : t_x_min;
	t_y_min = (t_y_min == infty) ? -infty : t_y_min;
	t_z_min = (t_z_min == infty) ? -infty : t_z_min;
	double t_min = std::max(std::max(t_x_min, t_y_min), t_z_min);
	if (t_min < t_max + eps)
		return CPoint2D(t_min, t_max);
	else
		return inftyPt2D;
}

double CBoundingBox::IntersectWithPlane(CRay ray, CPoint3D p0, CPoint3D norm)
{
	if (abs(Dot(norm, ray.direction())) < eps)
		return infty;
	double D = -Dot(p0, norm);
	double t = -(D + Dot(norm, ray.beginPt())) / Dot(norm, ray.direction());
	return t;
}

bool CBoundingBox::operator == (const CBoundingBox & box)
{
	if (m_min == box.m_min && m_max == box.m_max)
		return true;
	else
		return false;
}

bool CBoundingBox::operator != (const CBoundingBox & box)
{
	if (m_min == box.m_min && m_max == box.m_max)
		return false;
	else
		return true;
}

bool CBoundingBox::InBox(CPoint3D pt)
{
	if (m_min.x - eps < pt.x && pt.x < m_max.x + eps &&
		m_min.y - eps < pt.y && pt.y < m_max.y + eps &&
		m_min.z - eps < pt.z && pt.z < m_max.z + eps)
		return true;
	else
		return false;
}

#endif  // _GEOMETRY_H_