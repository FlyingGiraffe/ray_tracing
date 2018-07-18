#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "RayTracer.h"
#include "PhotonMapper.h"
#include "GodRay.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
#include <string>

//############################################ class CCamera ############################################
class CCamera
{
public:
	CCamera(CPoint3D v_viewPt, CPoint3D v_eyeDirection, CPoint3D v_left, double v_distOfPlane,
		double v_horRange, double v_verRange, int v_resolution, int v_depth) :
		viewPt(v_viewPt), eyeDirection(v_eyeDirection.Unitize()), up((v_left ^ v_eyeDirection).Unitize()), left()
	{
		left = (up ^ eyeDirection).Unitize();
		distOfPlane = v_distOfPlane;
		horRange = v_horRange;
		verRange = v_verRange;
		resolution = v_resolution;
		depth = v_depth;
		centerPt = viewPt + eyeDirection * distOfPlane;
	}
protected:
	CPoint3D viewPt;
	CPoint3D eyeDirection;  // direction of the looking-at point
	CPoint3D centerPt;  // the center point in the taken picture
	CPoint3D up;
	CPoint3D left;
	double distOfPlane;  // distance from view point to the plane of projection
	double horRange;  // horizontal range on the plane of projection
	double verRange;  // vertical range on the plane of projection
	int resolution;  // resolution ratio (number of pixels per unit lenth)
	int depth;  // exposure time of the camera
public:
	void Render_RayTracer(CScene scene, CColour background);
	void Render_RayTracer_Antialiasing(CScene scene, CColour background);
	void Render_Focusing(CScene scene, double radius, CColour background);
	void Render_Focusing_Antialiasing(CScene scene, double radius, CColour background);
	void Render_PhotonMapper(CScene * scene, int times, CColour background);
	void Render_PhotonMapper_Antialiasing(CScene * scene, int times, CColour background);
	void Render_RayTracer_GodRay(CGodRay * godRay, CColour background);
	void Render_RayTracer_GodRay_Antialiasing(CGodRay * godRay, CColour background);
protected:
	cv::Mat Laplace(cv::Mat mat);
};

void CCamera::Render_RayTracer(CScene scene, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CColour colour = RayTracer(ray, depth, 1, scene, background);
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	cv::imshow("output_RayTracer", picture);
	cv::imwrite("output_RayTracer.png", picture);
	cv::waitKey();
}

const int oversampleTimes = 100;
const double r0 = 0.007;  // radius for the oversampling at the margins
const double depth_grad_threshold = 0.8;
const double norm_grad_threshold = 0.8;
const double type_grad_threshold = 0.5;
const double colour_grad_threshold = 80;
void CCamera::Render_RayTracer_Antialiasing(CScene scene, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	// Get the depth map and normal vector map
	cv::Mat depthMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat typeMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CScene::CIntersection intersection = scene.FirstInterPt(ray, true);
			CPoint3D interPt = intersection.pt;
			CBaseObject * interObj = intersection.object;
			CPoint3D norm = interObj->Norm(interPt);
			double t = Dist(interPt, ray.beginPt());
			depthMap.at<double>(v, h) = t;
			normMap1.at<double>(v, h) = norm.x;
			normMap2.at<double>(v, h) = norm.y;
			normMap3.at<double>(v, h) = norm.z;
			typeMap.at<double>(v, h) = interObj->type();
			CColour colour0 = RayTracer(ray, depth, 1, scene, background);
			colourMap1.at<double>(v, h) = colour0.R();
			colourMap2.at<double>(v, h) = colour0.G();
			colourMap3.at<double>(v, h) = colour0.B();
		}
	}
	// Find out the margins of the depth map and normal vector map
	cv::Mat depth_grad = Laplace(depthMap);
	cv::Mat norm_grad1 = Laplace(normMap1);
	cv::Mat norm_grad2 = Laplace(normMap2);
	cv::Mat norm_grad3 = Laplace(normMap3);
	cv::Mat type_grad = Laplace(typeMap);
	cv::Mat colour_grad1 = Laplace(colourMap1);
	cv::Mat colour_grad2 = Laplace(colourMap2);
	cv::Mat colour_grad3 = Laplace(colourMap3);
	// Render the picture
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			CColour colour(0, 0, 0);
			// Generate a ray from view point to current point (on the picture)
			if (abs(depth_grad.at<double>(v, h)) > depth_grad_threshold ||
				abs(norm_grad1.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad2.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad3.at<double>(v, h)) > norm_grad_threshold ||
				abs(type_grad.at<double>(v, h)) > type_grad_threshold ||
				abs(colour_grad1.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad2.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad3.at<double>(v, h)) > colour_grad_threshold
				)
			{
				for (int j = 0; j < oversampleTimes; j++)
				{
					CPoint3D curPt1(curPt.x + random(r0, 1000), curPt.y + random(r0, 1000), curPt.z + random(r0, 1000));
					CRay ray1(viewPt, (curPt1 - viewPt).Unitize());
					colour += RayTracer(ray1, depth, 1, scene, background) / oversampleTimes;
				}
				picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
				continue;
			}
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			colour = RayTracer(ray, depth, 1, scene, background);
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	cv::imshow("output_RayTracer", picture);
	cv::imwrite("output_RayTracer.png", picture);
	cv::waitKey();
}

const int sampleTimes = 100;
void CCamera::Render_Focusing(CScene scene, double radius, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CColour colour(0, 0, 0);
			for (int i = 0; i < sampleTimes; i++)
			{
				CPoint3D p(viewPt.x + random(radius, 1000), viewPt.y + random(radius, 1000), viewPt.z + random(radius, 1000));
				CRay ray(p, (curPt - p).Unitize());
				colour += RayTracer(ray, depth, 1, scene, background) / sampleTimes;
			}
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	cv::imshow("output_Focusing", picture);
	cv::imwrite("output_Focusing.png", picture);
	cv::waitKey();
}

void CCamera::Render_Focusing_Antialiasing(CScene scene, double radius, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	// Get the depth map and normal vector map
	cv::Mat depthMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat typeMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CScene::CIntersection intersection = scene.FirstInterPt(ray, true);
			CPoint3D interPt = intersection.pt;
			CBaseObject * interObj = intersection.object;
			CPoint3D norm = interObj->Norm(interPt);
			double t = Dist(interPt, ray.beginPt());
			depthMap.at<double>(v, h) = t;
			normMap1.at<double>(v, h) = norm.x;
			normMap2.at<double>(v, h) = norm.y;
			normMap3.at<double>(v, h) = norm.z;
			typeMap.at<double>(v, h) = interObj->type();
		}
	}
	// Find out the margins of the depth map and normal vector map
	cv::Mat depth_grad = Laplace(depthMap);
	cv::Mat norm_grad1 = Laplace(normMap1);
	cv::Mat norm_grad2 = Laplace(normMap2);
	cv::Mat norm_grad3 = Laplace(normMap3);
	cv::Mat type_grad = Laplace(typeMap);
	// Render the picture
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
	int t = 0;
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			t++;
			std::cout << t << " ";
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CColour colour(0, 0, 0);
#pragma omp parallel for
			for (int i = 0; i < sampleTimes; i++)
			{
				CPoint3D p(viewPt.x + random(radius, 1000), viewPt.y + random(radius, 1000), viewPt.z + random(radius, 1000));
				CRay ray(p, (curPt - p).Unitize());
				if (abs(depth_grad.at<double>(v, h)) > depth_grad_threshold ||
					abs(norm_grad1.at<double>(v, h)) > norm_grad_threshold ||
					abs(norm_grad2.at<double>(v, h)) > norm_grad_threshold ||
					abs(norm_grad3.at<double>(v, h)) > norm_grad_threshold ||
					abs(type_grad.at<double>(v, h)) > type_grad_threshold)
				{
					for (int j = 0; j < oversampleTimes; j++)
					{
						CPoint3D curPt1(curPt.x + random(r0, 1000), curPt.y + random(r0, 1000), curPt.z + random(r0, 1000));
						CRay ray1(p, (curPt1 - p).Unitize());
						colour += RayTracer(ray1, depth, 1, scene, background) / (sampleTimes * oversampleTimes);
					}
					continue;
				}
				colour += RayTracer(ray, depth, 1, scene, background) / sampleTimes;
			}
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	cv::imshow("output_Focusing", picture);
	cv::imwrite("output_Focusing.png", picture);
	cv::waitKey();
}

cv::Mat CCamera::Laplace(cv::Mat mat)
{
	cv::Mat grad;
	double kernel[9] = {0, 1, 0, 1, -4, 1, 0, 1, 0};
	cv::Mat k(3, 3, CV_64FC1, kernel);
	cv::filter2D(mat, grad, mat.depth(), k);
	return grad;
}

void CCamera::Render_PhotonMapper(CScene * scene, int times, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
	CPhotonMapper ppm(scene, &picture);
	for (int h = 0; h < horPixelNum; h++)
	{
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CColour colour = ppm.RayTracingPass(ray, depth, 1, h, v, background);
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	ppm.PhotonMappingPass(times);
	cv::imshow("output_PhotonMapper", picture);
	cv::imwrite("output_PhotonMapper.png", picture);
	cv::waitKey();
}

void CCamera::Render_PhotonMapper_Antialiasing(CScene * scene, int times, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	// Get the depth map and normal vector map
	cv::Mat depthMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat typeMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	//#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
		//#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CScene::CIntersection intersection = scene->FirstInterPt(ray, true);
			CPoint3D interPt = intersection.pt;
			CBaseObject * interObj = intersection.object;
			CPoint3D norm = interObj->Norm(interPt);
			double t = Dist(interPt, ray.beginPt());
			depthMap.at<double>(v, h) = t;
			normMap1.at<double>(v, h) = norm.x;
			normMap2.at<double>(v, h) = norm.y;
			normMap3.at<double>(v, h) = norm.z;
			typeMap.at<double>(v, h) = interObj->type();
			CColour colour0 = RayTracer(ray, depth, 1, *scene, background);
			colourMap1.at<double>(v, h) = colour0.R();
			colourMap2.at<double>(v, h) = colour0.G();
			colourMap3.at<double>(v, h) = colour0.B();
		}
	}
	// Find out the margins of the depth map and normal vector map
	cv::Mat depth_grad = Laplace(depthMap);
	cv::Mat norm_grad1 = Laplace(normMap1);
	cv::Mat norm_grad2 = Laplace(normMap2);
	cv::Mat norm_grad3 = Laplace(normMap3);
	cv::Mat type_grad = Laplace(typeMap);
	cv::Mat colour_grad1 = Laplace(colourMap1);
	cv::Mat colour_grad2 = Laplace(colourMap2);
	cv::Mat colour_grad3 = Laplace(colourMap3);
	// Render the picture
	cv::Mat picture_ori(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat picture(verPixelNum, horPixelNum, CV_32SC3, cv::Scalar(0, 0, 0));
	CPhotonMapper ppm(scene, &picture);
	for (int h = 0; h < horPixelNum; h++)
	{
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			CColour colour(0, 0, 0);
			// Generate a ray from view point to current point (on the picture)
			if (abs(depth_grad.at<double>(v, h)) > depth_grad_threshold ||
				abs(norm_grad1.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad2.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad3.at<double>(v, h)) > norm_grad_threshold ||
				abs(type_grad.at<double>(v, h)) > type_grad_threshold ||
				abs(colour_grad1.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad2.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad3.at<double>(v, h)) > colour_grad_threshold
				)
			{
				for (int j = 0; j < oversampleTimes; j++)
				{
					CPoint3D curPt1(curPt.x + random(r0, 1000), curPt.y + random(r0, 1000), curPt.z + random(r0, 1000));
					CRay ray1(viewPt, (curPt1 - viewPt).Unitize());
					colour += ppm.RayTracingPass(ray1, depth, 1, h, v, background) / oversampleTimes;
				}
				picture_ori.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
				continue;
			}
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			colour = ppm.RayTracingPass(ray, depth, 1, h, v, background);
			picture_ori.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	ppm.PhotonMappingPass(times);
	for (int h = 0; h < horPixelNum; h++)
	{
		for (int v = 0; v < verPixelNum; v++)
		{
			// Generate a ray from view point to current point (on the picture)
			if (abs(depth_grad.at<double>(v, h)) > depth_grad_threshold ||
				abs(norm_grad1.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad2.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad3.at<double>(v, h)) > norm_grad_threshold ||
				abs(type_grad.at<double>(v, h)) > type_grad_threshold ||
				abs(colour_grad1.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad2.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad3.at<double>(v, h)) > colour_grad_threshold
				)
			{
				picture_ori.at<cv::Vec3b>(v, h) += picture.at<cv::Vec3i>(v, h) / oversampleTimes;
				continue;
			}
			picture_ori.at<cv::Vec3b>(v, h) += picture.at<cv::Vec3i>(v, h);
		}
	}
	cv::imshow("output_PhotonMapper", picture_ori);
	cv::imwrite("output_PhotonMapper.png", picture_ori);
	cv::waitKey();
}

void CCamera::Render_RayTracer_GodRay(CGodRay * godRay, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
	godRay->PhotonMappingPass();
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CColour colour = godRay->RayTracingPass(ray, depth, 1, background);
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	cv::imshow("output_RayTracer", picture);
	cv::imwrite("output_RayTracer.png", picture);
	cv::waitKey();
}

void CCamera::Render_RayTracer_GodRay_Antialiasing(CGodRay * godRay, CColour background = CColour(0, 0, 0))
{
	int horPixelNum = horRange * resolution;
	int verPixelNum = verRange * resolution;
	CPoint3D startPt = centerPt + left * (horRange / 2) + up * (verRange / 2);
	// Get the depth map and normal vector map
	cv::Mat depthMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat normMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat typeMap(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap1(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap2(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
	cv::Mat colourMap3(verPixelNum, horPixelNum, CV_64FC1, cv::Scalar(0));
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			// Generate a ray from view point to current point (on the picture)
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			CScene::CIntersection intersection = godRay->scene->FirstInterPt(ray, true);
			CPoint3D interPt = intersection.pt;
			CBaseObject * interObj = intersection.object;
			CPoint3D norm = interObj->Norm(interPt);
			double t = Dist(interPt, ray.beginPt());
			depthMap.at<double>(v, h) = t;
			normMap1.at<double>(v, h) = norm.x;
			normMap2.at<double>(v, h) = norm.y;
			normMap3.at<double>(v, h) = norm.z;
			typeMap.at<double>(v, h) = interObj->type();
			CColour colour0 = RayTracer(ray, depth, 1, *(godRay->scene), background);
			colourMap1.at<double>(v, h) = colour0.R();
			colourMap2.at<double>(v, h) = colour0.G();
			colourMap3.at<double>(v, h) = colour0.B();
		}
	}
	// Find out the margins of the depth map and normal vector map
	cv::Mat depth_grad = Laplace(depthMap);
	cv::Mat norm_grad1 = Laplace(normMap1);
	cv::Mat norm_grad2 = Laplace(normMap2);
	cv::Mat norm_grad3 = Laplace(normMap3);
	cv::Mat type_grad = Laplace(typeMap);
	cv::Mat colour_grad1 = Laplace(colourMap1);
	cv::Mat colour_grad2 = Laplace(colourMap2);
	cv::Mat colour_grad3 = Laplace(colourMap3);
	// Render the picture
	cv::Mat picture(verPixelNum, horPixelNum, CV_8UC3, cv::Scalar(0, 0, 0));
	godRay->PhotonMappingPass();
#pragma omp parallel for
	for (int h = 0; h < horPixelNum; h++)
	{
#pragma omp parallel for
		for (int v = 0; v < verPixelNum; v++)
		{
			CPoint3D curPt = startPt - left * (double(h) / double(resolution)) - up * (double(v) / double(resolution));
			CColour colour(0, 0, 0);
			// Generate a ray from view point to current point (on the picture)
			if (abs(depth_grad.at<double>(v, h)) > depth_grad_threshold ||
				abs(norm_grad1.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad2.at<double>(v, h)) > norm_grad_threshold ||
				abs(norm_grad3.at<double>(v, h)) > norm_grad_threshold ||
				abs(type_grad.at<double>(v, h)) > type_grad_threshold ||
				abs(colour_grad1.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad2.at<double>(v, h)) > colour_grad_threshold ||
				abs(colour_grad3.at<double>(v, h)) > colour_grad_threshold
				)
			{
				for (int j = 0; j < oversampleTimes; j++)
				{
					CPoint3D curPt1(curPt.x + random(r0, 1000), curPt.y + random(r0, 1000), curPt.z + random(r0, 1000));
					CRay ray1(viewPt, (curPt1 - viewPt).Unitize());
					colour += godRay->RayTracingPass(ray1, depth, 1, background) / oversampleTimes;
				}
				picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
				continue;
			}
			CRay ray(viewPt, (curPt - viewPt).Unitize());
			colour = godRay->RayTracingPass(ray, depth, 1, background);
			picture.at<cv::Vec3b>(v, h) = cv::Vec3b(colour.B(), colour.G(), colour.R());
		}
	}
	cv::imshow("output_RayTracer", picture);
	cv::imwrite("output_RayTracer.png", picture);
	cv::waitKey();
}

#endif