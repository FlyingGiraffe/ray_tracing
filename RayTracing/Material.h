#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#define _MIRROR_ 200
#define _GLASS_ 201
#define _ICE_ 202
#define _METAL_ 203
#define _PORCELAIN_ 204
#define _RUBBER_ 205
#define _PLASTIC_ 206

//############################################ class CMaterial ############################################
class CMaterial
{
public:
	CMaterial(double v_reflW, double v_refrW, double v_RI)
	{
		m_reflW = v_reflW;
		m_refrW = v_refrW;
		m_RI = v_RI;
	}
	CMaterial(int materialName);
public:
	double reflW() { return m_reflW; }
	double refrW() { return m_refrW; }
	double RI() { return m_RI; }
	// Phong model
	double diff() { return m_diff; }
	double spec() { return m_spec; }
	double shiness() { return m_shiness; }
	double emissive() { return m_emissive; }
	double ambient() { return m_ambient; }
protected:
	double m_reflW;  // Reflection weight of an intersecting ray
	double m_refrW;  // Refraction weight of an intersecting ray
	double m_RI;  // refractive index
	// Phong model
	double m_diff;
	double m_spec;
	double m_shiness;
	double m_emissive;
	double m_ambient;
};
 
CMaterial::CMaterial(int materialName)
{
	if (materialName == _MIRROR_)
	{
		m_reflW = 1;
		m_refrW = 0;
		m_RI = 0;
		m_diff = 0;
		m_spec = 1;
		m_shiness = 2;
		m_emissive = 0;
		m_ambient = 0;
	}
	if (materialName == _GLASS_)
	{
		m_reflW = 0.05;
		m_refrW = 0.95;
		m_RI = 1.52;
		m_diff = 0;
		m_spec = 0.05;
		m_shiness = 20;
		m_emissive = 0;
		m_ambient = 0;
	}
	if (materialName == _ICE_)
	{
		m_reflW = 0.05;
		m_refrW = 0.95;
		m_RI = 1.31;
		m_diff = 0;
		m_spec = 0.05;
		m_shiness = 20;
		m_emissive = 0;
		m_ambient = 1;
	}
	if (materialName == _METAL_)
	{
		m_reflW = 0.7;
		m_refrW = 0;
		m_RI = 0;
		m_diff = 0.2;
		m_spec = 0.8;
		m_shiness = 20;
		m_emissive = 0;
		m_ambient = 1;
	}
	if (materialName == _PORCELAIN_)
	{
		m_reflW = 0.1;
		m_refrW = 0;
		m_RI = 0;
		m_diff = 1;
		m_spec = 0.1;
		m_shiness = 2;
		m_emissive = 0;
		m_ambient = 1;
	}
	if (materialName == _RUBBER_)
	{
		m_reflW = 0;
		m_refrW = 0;
		m_RI = 0;
		m_diff = 0.3;
		m_spec = 0.4;
		m_shiness = 1;
		m_emissive = 0;
		m_ambient = 1;
	}
	if (materialName == _PLASTIC_)
	{
		m_reflW = 0;
		m_refrW = 0;
		m_RI = 0;
		m_diff = 0.6;
		m_spec = 0.0;
		m_shiness = 1;
		m_emissive = 0;
		m_ambient = 1;
	}
}

#endif