#ifndef _LAYOUT_H_
#define _LAYOUT_H_

#include "Camera.h"

void Object_Cup(CScene & scene, CPoint3D center, CMaterial material, CColour colour)
{
	CBezierCurve bezierCurve1;
	bezierCurve1.AddControlPt(CPoint2D(1, 0.1));
	bezierCurve1.AddControlPt(CPoint2D(0.1, 0.1));
	bezierCurve1.AddControlPt(CPoint2D(0.1, 1.1));
	bezierCurve1.AddControlPt(CPoint2D(0.1, 2.1));
	CRotationalSurface surface1(bezierCurve1);
	scene.RotationalSurfaceToMesh(surface1, 5, 14, center, material, colour);

	CBezierCurve bezierCurve2;
	bezierCurve2.AddControlPt(CPoint2D(0.1, 2.1));
	bezierCurve2.AddControlPt(CPoint2D(0.1, 3.1));
	bezierCurve2.AddControlPt(CPoint2D(1, 3.1));
	bezierCurve2.AddControlPt(CPoint2D(0.85, 5.1));
	CRotationalSurface surface2(bezierCurve2);
	scene.RotationalSurfaceToMesh(surface2, 8, 14, center, material, colour);

	CBezierCurve bezierCurve3;
	bezierCurve3.AddControlPt(CPoint2D(0.75, 5.1));
	bezierCurve3.AddControlPt(CPoint2D(0.7, 4.1));
	bezierCurve3.AddControlPt(CPoint2D(0.5, 3.1));
	bezierCurve3.AddControlPt(CPoint2D(0.1, 3.1));
	CRotationalSurface surface3(bezierCurve3);
	scene.RotationalSurfaceToMesh(surface3, 8, 14, center, material, colour);

	CBezierCurve bezierCurve4;
	bezierCurve4.AddControlPt(CPoint2D(1, 0));
	bezierCurve4.AddControlPt(CPoint2D(1.05, 0.05));
	bezierCurve4.AddControlPt(CPoint2D(1, 0.1));
	CRotationalSurface surface4(bezierCurve4);
	scene.RotationalSurfaceToMesh(surface4, 4, 14, center, material, colour);

	CBezierCurve bezierCurve5;
	bezierCurve5.AddControlPt(CPoint2D(0.85, 5.1));
	bezierCurve5.AddControlPt(CPoint2D(0.8, 5.2));
	bezierCurve5.AddControlPt(CPoint2D(0.75, 5.1));
	CRotationalSurface surface5(bezierCurve5);
	scene.RotationalSurfaceToMesh(surface5, 4, 14, center, material, colour);
}

void Object_Apple(CScene & scene, CPoint3D center, CMaterial material, CColour colour)
{
	cv::Mat * texture_apple_up = new cv::Mat;
	*texture_apple_up = cv::imread("./Textures/texture_apple_up.jpg");
	cv::Mat * texture_apple_down = new cv::Mat;
	*texture_apple_down = cv::imread("./Textures/texture_apple_down.jpg");

	CBezierCurve bezierCurve1;
	bezierCurve1.AddControlPt(CPoint2D(0, 0.25));
	bezierCurve1.AddControlPt(CPoint2D(0, 0));
	bezierCurve1.AddControlPt(CPoint2D(1.2, 0));
	bezierCurve1.AddControlPt(CPoint2D(1.25, 1));
	CRotationalSurface surface1(bezierCurve1);
	scene.RotationalSurfaceToMesh_Texture(surface1, 5, 15, texture_apple_down, center, material, colour);

	CBezierCurve bezierCurve2;
	bezierCurve2.AddControlPt(CPoint2D(1.25, 1));
	bezierCurve2.AddControlPt(CPoint2D(1.35, 2));
	bezierCurve2.AddControlPt(CPoint2D(0.35, 2));
	bezierCurve2.AddControlPt(CPoint2D(0, 1.75));
	CRotationalSurface surface2(bezierCurve2);
	scene.RotationalSurfaceToMesh_Texture(surface2, 5, 15, texture_apple_up, center, material, colour);
}

void Layout1_JugAndCups()
{
	CScene scene(0.05);

	CInftyPlane inftyPlane1(CPoint3D(0, 0, 0), CPoint3D(0, 0, 1), CMaterial(_RUBBER_), CColour(0, 0, 0));
	inftyPlane1.AddTexture("./Textures/texture5.jpg", 50);
	inftyPlane1.AddBumpTexture("./BumpTextures/bump7.jpg", 30);
	scene.AddObject((CBaseObject *)(&inftyPlane1));

	CInftyPlane inftyPlane2(CPoint3D(20, 0, 0), CPoint3D(-1, 0, 0), CMaterial(_RUBBER_), CColour(0, 0, 0));
	inftyPlane2.AddTexture("./Textures/texture6.jpg", 20);
	inftyPlane2.AddBumpTexture("./BumpTextures/bump5.jpg", 50);
	scene.AddObject((CBaseObject *)(&inftyPlane2));

	CInftyPlane inftyPlane3(CPoint3D(0, -10, 0), CPoint3D(0, 1, 0), CMaterial(_RUBBER_), CColour(0, 0, 0));
	inftyPlane3.AddTexture("./Textures/texture6.jpg", 20);
	inftyPlane3.AddBumpTexture("./BumpTextures/bump5.jpg", 50);
	scene.AddObject((CBaseObject *)(&inftyPlane3));

	Object_Cup(scene, CPoint3D(-2.5, 1.5, 0), CMaterial(_GLASS_), CColour(255, 255, 255));
	Object_Cup(scene, CPoint3D(-4, 0, 0), CMaterial(_METAL_), CColour(200, 200, 200));
	Object_Cup(scene, CPoint3D(-5.5, -1.5, 0), CMaterial(_METAL_), CColour(200, 200, 200));
	Object_Apple(scene, CPoint3D(-2, 7, 0), CMaterial(_RUBBER_), CColour(180, 0, 0));

	scene.ReadObjFile_Triangle("./ObjFiles/Jug_Triangle.obj", CPoint3D(5, 5, 0), 10,
		CMaterial(_METAL_), CColour(200, 200, 200));

	scene.AddPointLight(CPoint3D(-10, 50, 50), 2);

	scene.ConstructBSPTree();

	CCamera camera(CPoint3D(-13.5, 16.5, 10), CPoint3D(5, -5, -1), CPoint3D(-0.8, -1, 0), 5, 6, 6, 200, 5);
	camera.Render_RayTracer_Antialiasing(scene);
}

void Layout2_WallAndPaper()
{
	CScene scene(0.05);

	CInftyPlane inftyPlane2(CPoint3D(20, 0, 0), CPoint3D(-1, 0, 0), CMaterial(_RUBBER_), CColour(0, 0, 0));
	inftyPlane2.AddTexture("./Textures/texture_graffiti.jpg", 15);
	inftyPlane2.AddBumpTexture("./BumpTextures/bump_wall.jpg", 20);
	scene.AddObject((CBaseObject *)(&inftyPlane2));

	CPoint3D p[4][4];
	p[0][0] = CPoint3D(-2, 6, 9); p[0][1] = CPoint3D(-1, 2, 9); p[0][2] = CPoint3D(-1, -2, 9); p[0][3] = CPoint3D(-2, -6, 9);
	p[1][0] = CPoint3D(-1, 6, 3); p[1][1] = CPoint3D(0, 2, 3); p[1][2] = CPoint3D(0, -2, 3); p[1][3] = CPoint3D(-1, -6, 3);
	p[2][0] = CPoint3D(-1, 6, -3); p[2][1] = CPoint3D(0, 2, -2); p[2][2] = CPoint3D(0, -2, -3); p[2][3] = CPoint3D(-1, -6, -3);
	p[3][0] = CPoint3D(-2, 6, -9); p[3][1] = CPoint3D(-1, 2, -9); p[3][2] = CPoint3D(-1, -2, -9); p[3][3] = CPoint3D(-2, -6, -9);
	CBezierObject paper(p, CPoint3D(20, 25, 17), CMaterial(_RUBBER_), CColour(255, 255, 255));
	paper.AddTexture("./Textures/texture_algebra.jpg");
	paper.AddBumpTexture("./BumpTextures/bump_paper.jpg");
	scene.AddObject((CBaseObject *)(&paper));

	scene.ReadObjFile_Triangle("./ObjFiles/WallLight.obj", CPoint3D(20, 13, 20), 0.7,
		CMaterial(_METAL_), CColour(50, 50, 60));

	scene.AddParallelLight(CPoint3D(1, -1, -1), 2);

	scene.ConstructBSPTree();

	CCamera camera(CPoint3D(-10, 20, 20), CPoint3D(1, 0, 0), CPoint3D(-0.8, -1, 0), 5, 6, 6, 200, 2);
	camera.Render_RayTracer_Antialiasing(scene);
}

void Layout3_Icosahedron()
{
	CScene scene(0.5);

	CInftyPlane inftyPlane1(CPoint3D(0, 0, 0), CPoint3D(0, 0, 1), CMaterial(_PLASTIC_), CColour(255, 255, 255));
	inftyPlane1.AddTexture("./Textures/texture7.jpg", 150);
	scene.AddObject((CBaseObject *)(&inftyPlane1));

	CInftyPlane inftyPlane2(CPoint3D(10, 0, 0), CPoint3D(-1, 0, 0), CMaterial(_PLASTIC_), CColour(0, 0, 0));
	inftyPlane2.AddTexture("./Textures/texture8.jpg", 50);
	scene.AddObject((CBaseObject *)(&inftyPlane2));

	//scene.AddSphere(CPoint3D(0, -0.5, 0.5), 0.5, CMaterial("Plastic"), CColour(241, 46, 24));

	scene.ReadObjFile_Triangle("./ObjFiles/Icosahedron_Triangle.obj", CPoint3D(0, -1, 0), 0.0015,
		CMaterial(_PLASTIC_), CColour(241, 46, 24));

	scene.ConstructBSPTree();

	scene.AddAreaLight(CPoint3D(-10, 20, 20), 1, 10, 10);

	CCamera camera(CPoint3D(-10, 0, 1.5), CPoint3D(1, 0, 0), CPoint3D(0, -1, 0), 10, 6, 6, 200, 1);
	//camera.Render_RayTracer(scene, CColour(200, 200, 200));
	camera.Render_Focusing_Antialiasing(scene, 0.1, CColour(200, 200, 200));
}

void Layout4_Rings()
{
	CScene scene(0.05);

	CInftyPlane inftyPlane1(CPoint3D(0, 0, 0), CPoint3D(0, 0, 1), CMaterial(_PLASTIC_), CColour(255, 255, 255));
	inftyPlane1.AddTexture("./Textures/texture_wood.jpg", 50);
	scene.AddObject((CBaseObject *)(&inftyPlane1));

	CSphere sphere(CPoint3D(0, 0, 0), 100, CMaterial(_PLASTIC_), CColour(255, 255, 255));
	sphere.AddTexture("./Textures/texture_wood2.jpg");
	scene.AddObject((CBaseObject *)(&sphere));

	CSphere sphere1(CPoint3D(6.5, -1, 2), 2, CMaterial(_GLASS_), CColour(161, 143, 81));
	sphere1.AddBumpTexture("./BumpTextures/bump4_2.jpg");
	scene.AddObject((CBaseObject *)(&sphere1));

	scene.ReadObjFile_Triangle("./ObjFiles/Rings.obj", CPoint3D(2.3, 1.5, 0), 0.08,
		CMaterial(_METAL_), CColour(161, 143, 81));

	scene.ConstructBSPTree();

	scene.AddAreaLight(CPoint3D(-10, 20, 10), 2, 10, 10);

	CCamera camera(CPoint3D(-10, 0, 10), CPoint3D(2, 0, -1), CPoint3D(0, -1, 0), 10, 6, 6, 200, 3);
	//camera.Render_RayTracer(scene, CColour(50, 50, 50));
	camera.Render_PhotonMapper(&scene, 10, CColour(50, 50, 50));
}

void Object_Cup2(CScene & scene, CPoint3D center, CMaterial material, CColour colour)
{
	CBezierCurve bezierCurve1;
	bezierCurve1.AddControlPt(CPoint2D(2, 0.2));
	bezierCurve1.AddControlPt(CPoint2D(0.2, 0.2));
	bezierCurve1.AddControlPt(CPoint2D(0.2, 2.2));
	bezierCurve1.AddControlPt(CPoint2D(0.2, 4.2));
	//CRotationalSurface surface1(bezierCurve1);
	//scene.RotationalSurfaceToMesh(surface1, 5, 14, center, material, colour);
	CRotationalObject * rObj1 = new CRotationalObject(bezierCurve1, center, material, colour, 50);
	scene.AddObject((CBaseObject *)(rObj1));
	
	CBezierCurve bezierCurve2;
	bezierCurve2.AddControlPt(CPoint2D(0.2, 4.2));
	bezierCurve2.AddControlPt(CPoint2D(0.2, 6.2));
	bezierCurve2.AddControlPt(CPoint2D(2, 6.2));
	bezierCurve2.AddControlPt(CPoint2D(1.7, 10.2));
	//CRotationalSurface surface2(bezierCurve2);
	//scene.RotationalSurfaceToMesh(surface2, 8, 14, center, material, colour);
	CRotationalObject * rObj2 = new CRotationalObject(bezierCurve2, center, material, colour, 50);
	scene.AddObject((CBaseObject *)(rObj2));
	
	CBezierCurve bezierCurve3;
	bezierCurve3.AddControlPt(CPoint2D(1.6, 10.2));
	bezierCurve3.AddControlPt(CPoint2D(1.65, 8.2));
	bezierCurve3.AddControlPt(CPoint2D(1.3, 6.2));
	bezierCurve3.AddControlPt(CPoint2D(0, 6.2));
	//CRotationalSurface surface3(bezierCurve3);
	//scene.RotationalSurfaceToMesh(surface3, 8, 14, center, material, colour);
	CRotationalObject * rObj3 = new CRotationalObject(bezierCurve3, center, material, colour, 50);
	scene.AddObject((CBaseObject *)(rObj3));
	
	CBezierCurve bezierCurve4;
	bezierCurve4.AddControlPt(CPoint2D(2, 0));
	bezierCurve4.AddControlPt(CPoint2D(2.1, 0.1));
	bezierCurve4.AddControlPt(CPoint2D(2, 0.2));
	CRotationalObject * rObj4 = new CRotationalObject(bezierCurve4, center, material, colour, 10);
	scene.AddObject((CBaseObject *)(rObj4));

	CBezierCurve bezierCurve5;
	bezierCurve5.AddControlPt(CPoint2D(1.7, 10.2));
	bezierCurve5.AddControlPt(CPoint2D(1.65, 10.3));
	bezierCurve5.AddControlPt(CPoint2D(1.6, 10.2));
	CRotationalObject * rObj5 = new CRotationalObject(bezierCurve5, center, material, colour, 10);
	scene.AddObject((CBaseObject *)(rObj5));
}

void Layout5_Wine()
{
	CScene scene(0.5);

	CInftyPlane inftyPlane1(CPoint3D(0, 0, 0), CPoint3D(0, 0, 1), CMaterial(_RUBBER_), CColour(0, 0, 0));
	inftyPlane1.AddTexture("./Textures/texture_wood.jpg", 50);
	scene.AddObject((CBaseObject *)(&inftyPlane1));

	CInftyPlane inftyPlane2(CPoint3D(10, 0, 0), CPoint3D(-1, 0, 0), CMaterial(_PLASTIC_), CColour(0, 0, 0));
	inftyPlane2.AddTexture("./Textures/texture6.jpg", 20);
	scene.AddObject((CBaseObject *)(&inftyPlane2));

	CInftyPlane inftyPlane3(CPoint3D(0, -10, 0), CPoint3D(0, 1, 0), CMaterial(_PLASTIC_), CColour(0, 0, 0));
	inftyPlane3.AddTexture("./Textures/texture6.jpg", 20);
	scene.AddObject((CBaseObject *)(&inftyPlane3));

	Object_Cup2(scene, CPoint3D(-2, -2, 0), CMaterial(_GLASS_), CColour(155, 155, 155));
	CBezierCurve bezierCurve1;
	bezierCurve1.AddControlPt(CPoint2D(0, 6.2 - 10 * eps));
	bezierCurve1.AddControlPt(CPoint2D(1.3 + 10 * eps, 6.2 - 10 * eps));
	bezierCurve1.AddControlPt(CPoint2D(1.65 + 10 * eps, 8.2 - 10 * eps));
	bezierCurve1.AddControlPt(CPoint2D(1.6 + 10 * eps, 10.2 - 10 * eps));
	CRotationalObject * rObj1 = new CRotationalObject(bezierCurve1, CPoint3D(-2, -2, 0),
		CMaterial(_ICE_), CColour(151, 57, 29), 50, 0.7);
	scene.AddObject((CBaseObject *)(rObj1));
	CPoint2D pt1 = bezierCurve1.Point2D(0.7);
	CCircle * circle1 = new CCircle(CPoint3D(-2, -2, pt1.v), CPoint3D(0, 0, 1), pt1.u + 10 * eps, CMaterial(_ICE_), CColour(151, 57, 29));
	scene.AddObject((CBaseObject *)(circle1));

	Object_Cup2(scene, CPoint3D(2.2, 2.2, 0), CMaterial(_GLASS_), CColour(155, 155, 155));
	CBezierCurve bezierCurve2;
	bezierCurve2.AddControlPt(CPoint2D(0, 6.2));
	bezierCurve2.AddControlPt(CPoint2D(1.3 + 10 * eps, 6.2 - 10 * eps));
	bezierCurve2.AddControlPt(CPoint2D(1.65 + 10 * eps, 8.2 - 10 * eps));
	bezierCurve2.AddControlPt(CPoint2D(1.6 + 10 * eps, 10.2 - 10 * eps));
	CRotationalObject * rObj2 = new CRotationalObject(bezierCurve1, CPoint3D(2.2, 2.2, 0),
		CMaterial(_ICE_), CColour(100, 25, 30), 50, 0.5);
	scene.AddObject((CBaseObject *)(rObj2));
	CPoint2D pt2 = bezierCurve1.Point2D(0.5);
	CCircle * circle2 = new CCircle(CPoint3D(2.2, 2.2, pt2.v), CPoint3D(0, 0, 1), pt2.u + 10 * eps, CMaterial(_ICE_), CColour(100, 25, 30));
	scene.AddObject((CBaseObject *)(circle2));

	Object_Cup2(scene, CPoint3D(5, 5, 0), CMaterial(_GLASS_), CColour(155, 155, 155));
	CBezierCurve bezierCurve3;
	bezierCurve3.AddControlPt(CPoint2D(0, 6.2 + 10 * eps));
	bezierCurve3.AddControlPt(CPoint2D(1.3 + 10 * eps, 6.2 - 10 * eps));
	bezierCurve3.AddControlPt(CPoint2D(1.65 + 10 * eps, 8.2 - 10 * eps));
	bezierCurve3.AddControlPt(CPoint2D(1.6 + 10 * eps, 10.2 - 10 * eps));
	CRotationalObject * rObj3 = new CRotationalObject(bezierCurve1, CPoint3D(5, 5, 0),
		CMaterial(_ICE_), CColour(100, 25, 30), 50, 0.8);
	scene.AddObject((CBaseObject *)(rObj3));
	CPoint2D pt3 = bezierCurve1.Point2D(0.8);
	CCircle * circle3 = new CCircle(CPoint3D(5, 5, pt3.v), CPoint3D(0, 0, 1), pt3.u + 10 * eps, CMaterial(_ICE_), CColour(100, 25, 30));
	scene.AddObject((CBaseObject *)(circle3));

	//scene.AddPointLight(CPoint3D(-10, 25, 25), 2);
	scene.AddAreaLight(CPoint3D(-10, 25, 25), 2, 2, 5);
	
	CCamera camera(CPoint3D(-23.5, 27.5, 24), CPoint3D(2.9, -3, -2), CPoint3D(-1, -1, 0), 15, 8, 6, 100, 8);
	//camera.Render_RayTracer(scene);
	//camera.Render_RayTracer_Antialiasing(scene);
	//camera.Render_PhotonMapper(&scene, 30);
	camera.Render_PhotonMapper_Antialiasing(&scene, 30);
}

void Layout6_Room()
{
	CScene scene(0.1);

	CInftyPlane inftyPlane2(CPoint3D(50, 0, 0), CPoint3D(-1, 0, 0), CMaterial(_PLASTIC_), CColour(0, 0, 0));
	inftyPlane2.AddTexture("./Textures/texture8.jpg", 50);
	scene.AddObject((CBaseObject *)(&inftyPlane2));

	scene.ReadObjFile_Triangle("./ObjFiles/Corridor_Triangle.obj", CPoint3D(16, -1, 0), 1,
		CMaterial(_PLASTIC_), CColour(241, 46, 24));

	scene.ConstructBSPTree();

	scene.AddAreaLight(CPoint3D(-10, 20, 20), 2, 10, 10);

	CGodRay godRay(&scene, CColour(155, 155, 50));
	godRay.AddAreaLightSource_XZ(6, CPoint2D(-3, 0), CPoint2D(9, 10), 0.1, CPoint3D(1, -2, -1));

	CCamera camera(CPoint3D(-10, 0, 2.5), CPoint3D(1, 0, 0), CPoint3D(0, -1, 0), 10, 8, 6, 100, 1);
	camera.Render_RayTracer_GodRay_Antialiasing(&godRay);
	//camera.Render_RayTracer_Antialiasing(scene);
}

#endif