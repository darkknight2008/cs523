//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{

	auto sortRuleLambda = [](const CurvePoint& v1, const CurvePoint& v2) -> bool
	{
		return v1.time < v2.time;
	};
	std::sort(controlPoints.begin(), controlPoints.end(), sortRuleLambda);

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (!controlPoints.size() >= 2)
	{
		return false
	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{

	// Check if the end of the curve is reached, or the time is not valid
	if (time >= controlPoints.back().time)
	{
		return false
	}
	// Give a inital value to nextPoint
	if (!nextPoint > 0)
	{
		nextPoint = 1;
	}
	// Find nextPoint
	if (time >= controlPoints[nextPoint].time)
	{
		nextPoint += 1;
	}
	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime;

	p0 = controlPoints[nextPoint - 1].position;
	m0 = controlPoints[nextPoint - 1].tangent;
	p1 = controlPoints[nextPoint].position;
	m1 = controlPoints[nextPoint].tangent;

	// Calculate normalized t on Hermite curve
	normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);

	// Calculate position at t = time on Hermite curve
	newPosition =
		(2 * time * time * time - 3 * time * time + 1) * p0
		+ (time * time * time - 2 * time * time + time) * m0
		+ (-2 * time * time * time + 3 * time * time) * p1
		+ (time * time * time - time * time) * m1;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime;

	p0 = controlPoints[nextPoint - 1].position;
	p1 = controlPoints[nextPoint].position;

	// Calculate tangents based on postions.
	// ( It might be better to write a seperate function for it. since you want us to only work in this function so I will hard code it here. 

	switch (nextPoint-1) 
	{
	case 0 :
		m0 = 2(controlPoints[1].position - controlPoints[0].position) - (controlPoints[2].position - controlPoints[0].position) / 2;
		break;
	default:
		m0 = (controlPoints[nextPoint].position) - controlPoints[nextPoint - 2].position) / 2;
		break;
	}
	switch (nextPoint)
	{
	case controlPoints.size()-1:
		m1 = 2(controlPoints.end()[-1].position - controlPoints.end()[-2].position) - (controlPoints.end()[-1].position - controlPoints.end()[-3].position) / 2;
		break;
	default:
		m1 = (controlPoints[nextPoint + 1].position) - controlPoints[nextPoint - 1].position) / 2;
		break;
	}

	// Calculate normalized t on Hermite curve
	normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);

	// Calculate position at t = time on Hermite curve
	newPosition =
		(2 * time * time * time - 3 * time * time + 1) * p0
		+ (time * time * time - 2 * time * time + time) * m0
		+ (-2 * time * time * time + 3 * time * time) * p1
		+ (time * time * time - time * time) * m1;

	// Return result
	return newPosition;
}