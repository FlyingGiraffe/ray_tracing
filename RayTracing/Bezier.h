#ifndef _BEZIER_H_
#define _BEZIER_H_

#include "Geometry.h"
#include <vector>

//############################################ class CBezierCurve, P(t) ############################################
class CBezierCurve
{
public:
	CBezierCurve() { degree = -1; }
protected:
	std::vector<CPoint2D> controlPts;
	int degree;
public:
	CPoint2D Point2D(double t) { return Point2D(t, 0, degree); }  // Get the point corresponding to paramater t
	CPoint2D TanVec(double t);  // tangent vector
	CPoint2D Norm(double t);  // normal vector
	void AddControlPt(CPoint2D v_pt2D) { controlPts.push_back(v_pt2D); degree++; }
protected:
	CPoint2D Point2D(double t, int i, int deg);  // Get the point corresponding to paramater t using de Casteljau algorithm

	friend class CRotationalObject;
	friend void Layout5_Wine();
};

// Get the point corresponding to paramater t using de Casteljau algorithm
CPoint2D CBezierCurve::Point2D(double t, int i, int deg)
{
	if (deg == 0)
		return controlPts[i];
	else
		return Point2D(t, i, deg - 1) * (1 - t) + Point2D(t, i + 1, deg - 1) * t;
}

CPoint2D CBezierCurve::TanVec(double t)  // the tangent vector
{
	return (Point2D(t, 1, degree - 1) - Point2D(t, 0, degree - 1)) * degree;
}

CPoint2D CBezierCurve::Norm(double t)
{
	CPoint2D tanVec = TanVec(t);
	return CPoint2D(tanVec.v, -(tanVec.u)).Unitize();
}


//############################################ class CRotionalSurface ############################################
//  S(t, \theta) = \phi(P(t), \theta)
class CRotationalSurface
{
public:
	CRotationalSurface(CBezierCurve v_bezierCur) : bezierCur(v_bezierCur) {}
protected:
	CBezierCurve bezierCur;
public:
	CPoint3D Point3D(double t, double theta);  // Get the point corresponding to paramater (t, theta)
	CPoint3D Norm(double t, double theta);  // the normal vector
	CPoint3D dS_dt(double t, double theta);
	CPoint3D dS_dtheta(double t, double theta);

	friend class CRotationalObject;
};

CPoint3D CRotationalSurface::Point3D(double t, double theta)  // Get the point corresponding to paramater (t, theta)
{
	CPoint2D pt2D = bezierCur.Point2D(t);
	return CPoint3D(pt2D.u * cos(theta), pt2D.u * sin(theta), pt2D.v);
}

CPoint3D CRotationalSurface::Norm(double t, double theta)
{
	CPoint2D R_t = bezierCur.Norm(t);
	return CPoint3D(R_t.u * cos(theta), R_t.u * sin(theta), R_t.v).Unitize();
}

CPoint3D CRotationalSurface::dS_dt(double t, double theta)
{
	CPoint2D dPhi_dt = bezierCur.TanVec(t);
	return CPoint3D(cos(theta) * dPhi_dt.u, sin(theta) * dPhi_dt.u, dPhi_dt.v);
}

CPoint3D CRotationalSurface::dS_dtheta(double t, double theta)
{
	CPoint2D pt2D = bezierCur.Point2D(t);
	return CPoint3D(-(pt2D.u) * sin(theta), pt2D.u * cos(theta), 0);
}


//############################################ class BezierSurface (degree = 3) ############################################
class CBezierSurface
{
public:
	CBezierSurface(CPoint3D v_controlPts[4][4]);
protected:
	CPoint3D controlPts[4][4];
public:
	CPoint3D Point3D(double u, double v) { return Point3D(u, v, 0, 0, m, n); }
	CPoint3D Norm(double u, double v) { return (dS_du(u, v) ^ dS_dv(u, v)).Unitize(); }  // the normal vector
	CPoint3D dS_du(double u, double v);
	CPoint3D dS_dv(double u, double v);
protected:
	CPoint3D Point3D(double u, double v, int i, int j, int k, int l);
	const int m = 3, n = 3;

	friend class CBezierObject;
};

CBezierSurface::CBezierSurface(CPoint3D v_controlPts[4][4])
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			controlPts[i][j] = v_controlPts[i][j];
}

CPoint3D CBezierSurface::Point3D(double u, double v, int i, int j, int k, int l)
{
	if (k == 0 && l == 0)
		return controlPts[i][j];
	else if (l == 0)  // k \in [1, m]; l = 0
		return Point3D(u, v, i, j, k - 1, 0) * (1 - u) + Point3D(u, v, i + 1, j, k - 1, 0) * u;
	else if (k == m)  // k = m; l \in [1, n]
		return Point3D(u, v, 0, j, m, l - 1) * (1 - v) + Point3D(u, v, 0, j + 1, m, l - 1) * v;
	else if (k == 0)  // k = 0; l \in [1, n]
		return Point3D(u, v, i, j, 0, l - 1) * (1 - v) + Point3D(u, v, i, j + 1, 0, l - 1) * v;
	else if (l == n)  // k \in [1, m]; l = 0
		return Point3D(u, v, i, 0, k - 1, n) * (1 - u) + Point3D(u, v, i + 1, 0, k - 1, n) * u;
}

CPoint3D CBezierSurface::dS_du(double u, double v)
{
	return (Point3D(u, v, 1, 0, m - 1, n) - Point3D(u, v, 0, 0, m - 1, n)) * m;
}

CPoint3D CBezierSurface::dS_dv(double u, double v)
{
	return (Point3D(u, v, 0, 1, m, n - 1) - Point3D(u, v, 0, 0, m, n - 1)) * n;
}

#endif