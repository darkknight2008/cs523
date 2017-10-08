#include "obstacles/GJK_EPA.h"



void SteerLib::Simplex::pushPopNewVertex(Util::Vector new_vertices, Util::Vector direction)
{
	if (vertices.size() < 3)
	{
		vertices.push_back(new_vertices);
	}
	else
	{
		int i;
		i = (vertices[1] * direction < vertices[0] * direction) ? 1 : 0;
		i = (vertices[2] * direction < vertices[i] * direction) ? 2 : i;
		vertices[i] = new_vertices;
	}

	return;
}

//treat as origin in simplex for edge case
bool SteerLib::Simplex::nearestSimplex(Util::Vector& nearest)
{
	int length = vertices.size();
	switch (length) {
	case 0: 
		std::cerr << "ERROR>>>>Simplex is Empty!" << std::endl;
		return true;
	case 1: 
		nearest = vertices[0];
		return true;
	case 2: 
		nearestSegment(nearest, vertices[0], vertices[1]);
		return true;
	case 3:
		std::vector<bool> sides(3);
		std::vector<Util::Vector> nearests(3);
		for (int i = 0; i < 3; i++)
		{
			sides[i] = nearestSegment(nearests[i], vertices[0], vertices[1]);
		}
		if (sides[0] == sides[1] && sides[1] == sides[2])
		{
			return false;
		}
		else
		{
			int i;
			i = (nearests[1].lengthSquared() < nearests[0].lengthSquared()) ? 1 : 0;
			i = (nearests[2].lengthSquared() < nearests[i].lengthSquared()) ? 2 : i;
			nearest = nearests[i];
			if (nearest.lengthSquared() == 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}
	std::cerr << "ERROR>>>>Simplex is has too many endpoints!" << std::endl;
	return false;
}

bool SteerLib::Simplex::nearestSegment(Util::Vector& nearest, Util::Vector v1, Util::Vector v2)
{
	// line between v1 and v2: (1-t)v1 + t*v2. Need to find t s.t. ((1-t)v1 + t*v2) * (v2-v1) = 0 => t = v1*(v1-v2) / |v1-v2|^2
	float t = v1 * (v1 - v2) / (v1 - v2).lengthSquared();
	if (t < 0)
	{
		nearest =v1;
	}
	else if (t > 1)
	{
		nearest = v2;
	}
	else
	{
		nearest = (1 - t) * v1 + t * v2;
	}
	bool side = true ? (v2 - v1) * v2 > 0 : false;
	return side;
}

SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Simplex return_simplex;

	if (algorithmGJK(return_simplex, _shapeA, _shapeB))
	{
		algorithmEPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, return_simplex);
		return true;
	}
	else
	{
		return_penetration_depth = 0;
		return_penetration_vector = {};
		return false;
	}
	return false; // There is no collision
}

//float SteerLib::GJK_EPA::lineToOriginDistance(std::vector<Util::Vector>& endpoints)
//{
//	return 0.0;
//}

bool SteerLib::GJK_EPA::algorithmGJK(Simplex& return_simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Util::Vector direction = Util::Vector(1.0, 0.0, 0.0);
	Util::Vector maxA;
	Util::Vector maxB;
	support(maxA, direction, _shapeA);
	support(maxB, -direction, _shapeB);
	return_simplex.pushPopNewVertex(maxA - maxB, direction);
	direction = maxB - maxA;
	while (true)
	{
		support(maxA, direction, _shapeA);
		support(maxB, -direction, _shapeB);
		if ((maxA - maxB) * direction <= 0)
		{
			return false;
		}
		else
		{
			return_simplex.pushPopNewVertex(maxA - maxB, direction);
			if (!return_simplex.nearestSimplex(direction))
			{
				//If return_simplexs contains the origin
				return true;
			}
			else
			{
				//set the new direction to be the vector from the nearest point to the origin
				direction = -direction;
			}
		}
	}
	return false;
}

void SteerLib::GJK_EPA::support(Util::Vector& maximizer, const Util::Vector direction, const std::vector<Util::Vector>& _shape)
{	
	int index = 0;
	for (int i = 1; i < _shape.size(); i++)
	{
		if (direction * _shape[i] > direction * _shape[index])
		{
			index = i;
		}
	}
	maximizer = _shape[index];
	return;
}

void SteerLib::GJK_EPA::algorithmEPA(float& return_penetration_depth, Util::Vector& return_penetration_vector,
	const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB,
	const Simplex& s)
{
	if (s.vertices.size() < 3)
	{
		std::cerr << "ERROR>>>>Simplex dont have enough dimension for EPA!" << std::endl;
		return;
	}

	// We maintain a min heap which contains all edges of the ploygon and their nearest point during expanding, with format <nearest, v1, v2>
	std::priority_queue<std::vector<Util::Vector>, std::vector<std::vector<Util::Vector>>, Compare> edgeDistanceHeap;
	//std::vector<Util::Vector> popVector;

	float const minThresh = 0.0001;
	Util::Vector maxA;
	Util::Vector maxB;
	Util::Vector newVertex;

	// push all edges into the heap
	Util::Vector nearest;
	Simplex::nearestSegment(nearest, s.vertices[0], s.vertices[1]);
	edgeDistanceHeap.push({ nearest , s.vertices[0], s.vertices[1] });
	Simplex::nearestSegment(nearest, s.vertices[1], s.vertices[2]);
	edgeDistanceHeap.push({ nearest , s.vertices[1], s.vertices[2] });
	Simplex::nearestSegment(nearest, s.vertices[2], s.vertices[0]);
	edgeDistanceHeap.push({ nearest , s.vertices[2], s.vertices[0] });

	return_penetration_depth = 99999999999;
	// Do the expansion
	while (true)
	{
		std::vector<Util::Vector> popVector = edgeDistanceHeap.top();
		return_penetration_depth = popVector[0].length();

		//Deal the edge case
		if (return_penetration_depth == 0)
		{
			return_penetration_vector = Util::Vector(0, 0, 0);
		}

		// Push this edge on A-B
		support(maxA, popVector[0], _shapeA);
		support(maxB, -popVector[0], _shapeB);
		newVertex = maxA - maxB;
		if (newVertex * popVector[0] - return_penetration_depth * return_penetration_depth < minThresh)
		{
			//Finish the expansion. Recall we are caluculate the nearest point in A -B , so we need to reverse it and normalize it
			//The final return_penetration_vector will be a vector from A to B
			return_penetration_vector = -normalize(popVector[0]);
			return;
		}
		else
		{
			// add two new edges into the heap
			Util::Vector v1 = popVector[1];
			Util::Vector v2 = popVector[2];
			edgeDistanceHeap.pop();
			Simplex::nearestSegment(nearest, newVertex, v1);
			edgeDistanceHeap.push({ nearest, newVertex, v1 });
			Simplex::nearestSegment(nearest, newVertex, v2);
			edgeDistanceHeap.push({ nearest, newVertex, v2 });
		}
	}
	return;
}

