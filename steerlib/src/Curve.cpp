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
	if (!checkRobust())
	{
		return;
	}
	
	Point start;
	Point end;

	// draw curve from controlPoints[i] to controlPoints[i+1], use 'window' as number of segments.
	for (int i = 0; i < controlPoints.size() - 1; i++)
	{
		end = controlPoints[i].position;
		for (int j = 0; j < window - 1; j++)
		{
			start = end;
			calculatePoint(end, controlPoints[i].time + (float)(j + 1) / (float)(window)* (controlPoints[i+1].time - controlPoints[i].time));
			DrawLib::drawLine(start, end, curveColor, curveThickness);
		}
		DrawLib::drawLine(end, controlPoints[i+1].position, curveColor, curveThickness);
	}
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
	if (! (controlPoints.size() >= 2))
	{
		return false;
	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	// Give a inital value to nextPoint
	nextPoint = 1;
	// Find nextPoint
	while (nextPoint < controlPoints.size() && time >= controlPoints[nextPoint].time)
	{
		nextPoint += 1;
	}
	if (nextPoint == controlPoints.size())
	{
		return false;
	}
	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime;

	Point p0;
	Point p1;
	Vector m0;
	Vector m1;

	p0 = controlPoints[nextPoint - 1].position;
	m0 = controlPoints[nextPoint - 1].tangent;
	p1 = controlPoints[nextPoint].position;
	m1 = controlPoints[nextPoint].tangent;

	// Calculate normalized t on Hermite curve
	normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);

	// Calculate position at t = time on Hermite curve
	newPosition =
		(2 * normalTime * normalTime * normalTime - 3 * normalTime * normalTime + 1) * p0
		+ (normalTime * normalTime * normalTime - 2 * normalTime * normalTime + normalTime) * m0
		+ (-2 * normalTime * normalTime * normalTime + 3 * normalTime * normalTime) * p1
		+ (normalTime * normalTime * normalTime - normalTime * normalTime) * m1;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime;

	Point p0;
	Point p1;
	Vector m0;
	Vector m1;

	p0 = controlPoints[nextPoint - 1].position;
	p1 = controlPoints[nextPoint].position;

	// Calculate tangents based on postions.
	// ( It might be better to write a seperate function for it. since you want us to only work in this function so I will hard code it here. 

	if (controlPoints.size() == 2)
	{
		m0 = (controlPoints[1].position - controlPoints[0].position) / 2;
		m1 = (controlPoints[1].position - controlPoints[0].position) / 2;
	}
	else
	{
		if (nextPoint - 1 == 0)
		{
			m0 = 2 * (controlPoints[1].position - controlPoints[0].position) - (controlPoints[2].position - controlPoints[0].position) / 2;
		}
		else
		{
			m0 = (controlPoints[nextPoint].position - controlPoints[nextPoint - 2].position) / 2;
		}
		if (nextPoint == controlPoints.size() - 1)
		{
			m1 = 2 * (controlPoints.end()[-1].position - controlPoints.end()[-2].position) - (controlPoints.end()[-1].position - controlPoints.end()[-3].position) / 2;
		}
		else
		{
			m1 = (controlPoints[nextPoint + 1].position - controlPoints[nextPoint - 1].position) / 2;
		}
	}

	// Calculate normalized t on Hermite curve
	normalTime = (time - controlPoints[nextPoint - 1].time) / (controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time);

	// Calculate position at t = time on Hermite curve
	newPosition =
		(2 * normalTime * normalTime * normalTime - 3 * normalTime * normalTime + 1) * p0
		+ (normalTime * normalTime * normalTime - 2 * normalTime * normalTime + normalTime) * m0
		+ (-2 * normalTime * normalTime * normalTime + 3 * normalTime * normalTime) * p1
		+ (normalTime * normalTime * normalTime - normalTime * normalTime) * m1;

	// Return result
	return newPosition;
}