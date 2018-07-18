#ifndef _LIGHT_H_
#define _LIGHT_H_

#include <string>
#include <vector>

#define _POINT_LIGHT_ 100
#define _PARALLEL_LIGHT_ 101
#define _AREA_LIGHT_ 102

inline double random(double r, int x) { return 2 * r * double(rand() % x) / double(x) - r; }

class CScene;
class CBaseObject;

//############################################ class CBaseLight ############################################
class CBaseLight
{
public:
	CBaseLight(double v_luminosity) { m_luminosity = v_luminosity; }
public:
	virtual CPoint3D Direction(CPoint3D pt) = 0;
	int type() { return m_type; }
	virtual double luminosity(CPoint3D pt, std::vector<CBaseObject *> objects);
protected:
	virtual bool InShadow(CPoint3D pt, CBaseObject * object) = 0;  // Judge whether the given point is in the shadow of the object
protected:
	int m_type;
	double m_luminosity;

	friend class CPhotonMapper;
};

double CBaseLight::luminosity(CPoint3D pt, std::vector<CBaseObject *> objects)
{
	double L = m_luminosity;
	// Judge whether the intersection point is in the shade of current light
	for (std::vector<CBaseObject *>::iterator objIter = objects.begin(); objIter != objects.end(); objIter++)
	{
		if (InShadow(pt, *objIter) == true)
				return 0;
	}
	return L;
}


//############################################ class CPointLight ############################################
class CPointLight : public CBaseLight
{
public:
	CPointLight(CPoint3D v_position, double v_luminosity) : CBaseLight(v_luminosity)
		{ m_type = _POINT_LIGHT_; m_position = v_position; }
public:
	CPoint3D position() { return m_position; }
	CPoint3D Direction(CPoint3D pt) { return (pt - m_position).Unitize(); }
protected:
	bool InShadow(CPoint3D pt, CBaseObject * object);  // Judge whether the given point is in the shadow of the object
protected:
	CPoint3D m_position;
};

bool CPointLight::InShadow(CPoint3D pt, CBaseObject * object)  // Judge whether the given point is in the shadow of the object
{
	CRay ray(pt, position() - pt);
	CPoint3D interPt = object->FirstInterPt_Unrecorded(ray);
	if (interPt != inftyPt  && Dot(pt - interPt, m_position - interPt) < -eps)
		return true;
	return false;
}


//############################################ class CParallelLight ############################################
class CParallelLight : public CBaseLight
{
public:
	CParallelLight(CPoint3D v_direction, double v_luminosity) : CBaseLight(v_luminosity)
		{ m_type = _PARALLEL_LIGHT_; m_direction = v_direction.Unitize(); }
public:
	CPoint3D direction() { return m_direction; }	
	CPoint3D Direction(CPoint3D pt) { return m_direction; }
protected:
	bool InShadow(CPoint3D pt, CBaseObject * object);  // Judge whether the given point is in the shadow of the object
protected:
	CPoint3D m_direction;
};

bool CParallelLight::InShadow(CPoint3D pt, CBaseObject * object)  // Judge whether the given point is in the shadow of the object
{
	CRay ray(pt, -m_direction);
	CPoint3D interPt = object->FirstInterPt_Unrecorded(ray);
	if (interPt != inftyPt)
		return true;
	return false;
}


//############################################ class CAreaLight ############################################
class CAreaLight : public CBaseLight
{
public:
	CAreaLight(CPoint3D v_position, double v_luminosity, double v_len, int v_sampleNum) : CBaseLight(v_luminosity)
	{
		m_type = _AREA_LIGHT_;
		m_position = v_position;
		len = v_len;
		sampleNum = v_sampleNum;
		lumPerSamplePt = m_luminosity / (sampleNum * sampleNum);
	}
public:
	CPoint3D position() { return m_position; }
	CPoint3D Direction(CPoint3D pt) { return (pt - m_position).Unitize(); }
	double luminosity(CPoint3D pt, std::vector<CBaseObject *> objects);
	double luminosity_BSPTree(CPoint3D pt, CScene * scene);
protected:
	bool InShadow(CPoint3D pt, CBaseObject * object) { return false; }
	bool InShadow(CPoint3D pt, CPoint3D samplePoint, CBaseObject * object);
	double luminosity(CPoint3D pt, CPoint3D samplePoint, std::vector<CBaseObject *> objects);
protected:
	CPoint3D m_position;
	double len;
	int sampleNum;
	double lumPerSamplePt;
	friend class CPhotonMapper;
};

double CAreaLight::luminosity(CPoint3D pt, std::vector<CBaseObject *> objects)
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
			L += luminosity(pt, samplePt, objects);
		}	
	}
	return L;
}

bool CAreaLight::InShadow(CPoint3D pt, CPoint3D samplePoint, CBaseObject * object)
{
	CRay ray(pt, samplePoint - pt);
	CPoint3D interPt = object->FirstInterPt_Unrecorded(ray);
	if (interPt != inftyPt  && Dot(pt - interPt, m_position - interPt) < -eps)
		return true;
	return false;
}

double CAreaLight::luminosity(CPoint3D pt, CPoint3D samplePoint, std::vector<CBaseObject *> objects)
{
	double L = lumPerSamplePt;
	// Judge whether the intersection point is in the shade of current light
	for (std::vector<CBaseObject *>::iterator objIter = objects.begin(); objIter != objects.end(); objIter++)
	{
		if (InShadow(pt, samplePoint, *objIter) == true)
			return 0;
	}
	return L;
}

#endif