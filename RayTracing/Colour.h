#ifndef _COLOUR_H_
#define _COLOUR_H_

#include <algorithm>

//############################################ class CColour ############################################
class CColour
{
public:
	CColour() { m_R = 0; m_G = 0; m_B = 0; }
	CColour(double v_R, double v_G, double v_B) { m_R = v_R; m_G = v_G; m_B = v_B; }
protected:
	double m_R, m_G, m_B;
public:
	CColour operator + (const CColour & colour) const;
	CColour operator * (const double & weight) const;
	CColour operator / (const double & weight) const;
	CColour & operator += (const CColour & colour);
	CColour & operator *= (const double & weight);
	CColour & operator /= (const double & weight);
	double R() { return m_R; }
	double G() { return m_G; }
	double B() { return m_B; }
};

CColour CColour::operator + (const CColour & colour) const
{
	return CColour(std::min(255.0, m_R + colour.m_R), std::min(255.0, m_G + colour.m_G), std::min(255.0, m_B + colour.m_B));
}

CColour CColour::operator * (const double & weight) const
{
	return CColour(std::min(255.0, m_R * weight), std::min(255.0, m_G * weight), std::min(255.0, m_B * weight));
}

CColour CColour::operator / (const double & weight) const
{
	return CColour(std::min(255.0, m_R / weight), std::min(255.0, m_G / weight), std::min(255.0, m_B / weight));
}

CColour & CColour::operator += (const CColour & colour)
{
	m_R = std::min(255.0, m_R + colour.m_R);
	m_G = std::min(255.0, m_G + colour.m_G);
	m_B = std::min(255.0, m_B + colour.m_B);
	return *this;
}

CColour & CColour::operator *= (const double & weight)
{
	m_R = std::min(255.0, m_R * weight);
	m_G = std::min(255.0, m_G * weight);
	m_B = std::min(255.0, m_B * weight);
	return *this;
}

CColour & CColour::operator /= (const double & weight)
{
	m_R = std::min(255.0, m_R / weight);
	m_G = std::min(255.0, m_G / weight);
	m_B = std::min(255.0, m_B / weight);
	return *this;
}

#endif  // _COLOUR_H_