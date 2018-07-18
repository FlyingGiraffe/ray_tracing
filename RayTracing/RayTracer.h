#ifndef _RAY_TRACER_H_
#define _RAY_TRACER_H_

#include "Colour.h"
#include "Scene.h"

const double threshold = 0.1;
CColour RayTracer(CRay ray, int depth, double weight, CScene scene, CColour background = CColour(0, 0, 0))
{
	CColour colour(0, 0, 0);
	if (weight < threshold)
		return background;
	// Compute the first intersection point of the ray with the objects in the scene
	CScene::CIntersection intersection;
	intersection = scene.FirstInterPt_BSPTree(ray, true);
	CPoint3D firstInterPt = intersection.pt;
	CBaseObject * firstInterObj = intersection.object;
	if (firstInterObj == NULL)  // No intersection point
		return background;
	// Compute the local illumination
	colour = scene.LocalIllumination(ray, intersection);
	if (depth > 1)
	{
		bool totalReflect = false;
		double refrW = firstInterObj->refrW(ray, firstInterPt);
		double reflW = firstInterObj->reflW(ray, firstInterPt);
		if (refrW != 0)
		{
			CRay refractRay = firstInterObj->RefractRay(ray, firstInterPt);
			if (refractRay.direction() != inftyPt)  // no total reflection
			{
				CColour refractColour = RayTracer(refractRay, depth - 1, weight * refrW, scene, background);
				colour += refractColour * refrW;
			}
			else
				totalReflect = true;
		}
		if (reflW != 0)
		{
			CRay reflectRay = firstInterObj->ReflectRay(ray, firstInterPt);
			CColour reflectColour;
			if (totalReflect != true)  // no total reflection
			{
				reflectColour = RayTracer(reflectRay, depth - 1, weight * reflW, scene, background);
				colour += reflectColour * reflW;
			}
			else  // total reflection
			{
				reflectColour = RayTracer(reflectRay, depth - 1, weight * (reflW + refrW), scene, background);
				colour += reflectColour * (reflW + refrW);
			}
		}
	}
	return colour;
}

#endif  // _RAY_TRACER_H_