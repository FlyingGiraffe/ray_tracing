#ifndef _PARSER_H_
#define _PARSER_H_

#include "Geometry.h"
#include <fstream>
#include <sstream>

//############################################ class CObjMesh ############################################
class CObjMesh
{
public:
	CObjMesh(std::string filename) { LoadObjFile(filename); }
public:
	struct CPointTuple
	{
		int pointIndex;  // index of "v"
		int texCoordsIndex;  // index of "vt"
		int normIndex;  // index of "vn"
	};
	struct CObjFace
	{
		std::vector<CPointTuple> indices;
	};
protected:
	void LoadObjFile(std::string filename);
protected:
	std::vector<CPoint3D> points;  // "v" flag
	std::vector<CPoint2D> texCoords;  // "vt" flag
	std::vector<CPoint3D> normals;  // "vn" flag
	std::vector<CObjFace> faces;  // "f" flag

	friend class CScene;
};

void CObjMesh::LoadObjFile(std::string filename)
{
	std::fstream objStream;
	objStream.open(filename, std::ios_base::in);
	std::string objFlag;
	for (; objStream.good();)
	{
		objStream >> objFlag;
		if (objFlag == "v")
		{
			double x = 0, y = 0, z = 0;
			objStream >> x >> y >> z;
			points.push_back(CPoint3D(x, y, z));
		}
		else if (objFlag == "vt")
		{
			double u = 0, v = 0;
			objStream >> u >> v;
			texCoords.push_back(CPoint2D(u, v));
		}
		else if (objFlag == "vn")
		{
			double x = 0, y = 0, z = 0;
			objStream >> x >> y >> z;
			normals.push_back(CPoint3D(x, y, z));
		}
		else if (objFlag == "f")
		{
			CObjFace face;
			char tuples[2048]{};
			objStream.getline(tuples, 2048, '\n');
			int vIndex = 0, vtIndex = 0, vnIndex = 0;
			int read = 0;
			for (int i = 0; sscanf_s(&tuples[i], "%i/%i/%i%n", &vIndex, &vtIndex, &vnIndex, &read) == 3;)
			{
				i += read;
				CPointTuple tuple;
				tuple.pointIndex = vIndex - 1;
				tuple.texCoordsIndex = vtIndex - 1;
				tuple.normIndex = vnIndex - 1;
				face.indices.push_back(tuple);
			}
			if (face.indices.size() != 0)
				faces.push_back(face);
		}
	}
	objStream.close();
}

#endif