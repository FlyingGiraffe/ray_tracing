#include "Camera.h"
#include "Layout.h"

int main()
{
	//Layout1_JugAndCups();
	//Layout2_WallAndPaper();
	//Layout3_Icosahedron();
	//Layout4_Rings();
	//Layout5_Wine();
	Layout6_Room();


	// god ray test
	/*
	CScene scene(0.5);

	scene.AddInftyPlane(CPoint3D(0, 0, 0), CPoint3D(0, 0, 1), CMaterial(_PLASTIC_), CColour(155, 155, 155));
	scene.AddInftyPlane(CPoint3D(0, 0, 3), CPoint3D(0, 0, -1), CMaterial(_PLASTIC_), CColour(155, 155, 155));
	scene.AddInftyPlane(CPoint3D(0.8, 0, 0), CPoint3D(-1, 0, 0), CMaterial(_PLASTIC_), CColour(155, 155, 155));
	scene.AddInftyPlane(CPoint3D(0, 1.5, 0), CPoint3D(0, -1, 0), CMaterial(_PLASTIC_), CColour(0, 0, 255));
	scene.AddInftyPlane(CPoint3D(0, -1.5, 0), CPoint3D(0, 1, 0), CMaterial(_PLASTIC_), CColour(255, 0, 0));

	scene.AddPointLight(CPoint3D(-0.4, 0, 3 - eps * 100), 1);

	CGodRay godRay(&scene, CColour(155, 155, 50));
	godRay.AddAreaLightSource_XZ(1.5, CPoint2D(-0.2, 1), CPoint2D(0.3, 2), 0.1, CPoint3D(0, -1, -1));

	CCamera camera(CPoint3D(-10, 0, 1.5), CPoint3D(1, 0, 0), CPoint3D(0, -1, 0), 10, 6, 6, 100, 1);
	camera.Render_RayTracer_GodRay(&godRay, CColour(200, 200, 200));
	*/

	return 0;
}