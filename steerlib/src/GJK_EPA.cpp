#include "obstacles/GJK_EPA.h"

void SteerLib::Simplex::pushPopNewVertex(Util::Vector v, Util::Vector& d)
{
	return;
}

bool SteerLib::Simplex::nearestSimplex(Util::Vector& e)
{
	return false;
}

SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	return false; // There is no collision
}

float SteerLib::GJK_EPA::lineToOriginDistance(std::vector<Util::Vector>& endpoints)
{
	return 0.0;
}

bool SteerLib::GJK_EPA::algorithmGJK(std::vector<Util::Vector>& return_simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	return false;
}

Util::Vector SteerLib::GJK_EPA::support(const Util::Vector& d, const std::vector<Util::Vector>& _shape)
{
	return Util::Vector(0.0, 0.0, 0.0);
}

void SteerLib::GJK_EPA::algorithmEPA(float& return_penetration_depth, Util::Vector& return_penetration_vector,
	const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB,
	const std::vector<Util::Vector>& S)
{
	return;
}


