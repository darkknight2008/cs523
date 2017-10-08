/*!
*
* \author VaHiD AzIzI
*
*/


#ifndef GJK_EPA_H_
#define GJK_EPA_H_


#include "util/Geometry.h"


#include <vector>
#include <math.h>
#include <limits>
#include <queue>


namespace SteerLib
{

	class STEERLIB_API Simplex
	{
	public:
		//All vertices, since we are working on 2D, there are at most 3 vertices
		std::vector<Util::Vector> vertices;

		Simplex() : vertices(std::vector<Util::Vector>()) { }

		Simplex(std::vector<Util::Vector> new_vertices) : vertices(new_vertices) { }

		//Push a new vertex v into the simplex, if the number of vertex exceed the limit, pop the one with minimum dot product with vector d
		void pushPopNewVertex(Util::Vector new_vertices, Util::Vector direction);

		//Calculate the point on the simplex S which is cloest to the origin and set it to e. Return false if simplex S contains the origin.
		bool nearestSimplex(Util::Vector& nearest);

		//nearest point for a segment, also calculate origin is on which side of the segement v1 v2, left: false, right, true
		static bool nearestSegment(Util::Vector& nearest, Util::Vector v1, Util::Vector v2);
	};

	class STEERLIB_API Compare
	{
	public:
		bool operator() (std::vector<Util::Vector> a, std::vector<Util::Vector> b)
		{
			//Recall we need the edge with smallest distance to the origin
			return (a[0].lengthSquared() > b[0].lengthSquared());
		}
	};

	class STEERLIB_API GJK_EPA
	{
	public:
		GJK_EPA();

		/*
		*
		*  DO NOT CHANGE THE FUNCTION DEFINITION FOR intersect()
		*  -----------------------------------------------------
		*  Return type:
		*  If there is a collision detected by the GJK algorithm the return value of intersect is TRUE, else FALSE
		*  -----------------------------------------------------
		*  Arguments:
		*  float& return_penetration_depth
		*  If there is a collision, this variable needs to be updated to the penetration depth calculated by the EPA algorithm
		*  TODO: YOU NEED TO POPULATE THIS VARIABLE
		*
		*  Util::Vector& return_penetration_vector
		*  If there is a collision, this variable needs to be updated to the penetration vector calculated by the EPA algorithm
		*  TODO: YOU NEED TO POPULATE THIS VARIABLE
		*
		*  NOTE: The above arguments are passed by reference, thereby their value change is reflected in the calling function.
		*  THIS IS CALLED PASS BY REFERENCE.
		*  This is way for functions to "return" multiple values
		*  ie.
		*  funct()
		*  {
		*      return (boolean, val)
		*  }
		*
		*  can be implemented as:
		*
		*  boolean funct(ref& value)
		*  {
		*      value = val;
		*  }
		*
		*  const std::vector<Util::Vector>& _shapeA
		*  The first input shape passed by the collisionAI. Every point p(x,y,z) is a "Util::Vector p(x,y,z);"
		*  std::vector<point> gives the list of points in the polygon as "std::vector<Util::Vector>"
		*  NOTE: ALL THE POINTS LIE ON THE XZ PLANE, ie. y=0 for all p. So all Polygon = { p1(x1,0,z2), p2(x2,0,z2), p3(x3,0,z3) ...}
		*  where Polygon is the std::vector and p1..pn are Util::Vector of (x,0,z)
		*
		*  const std::vector<Util::Vector>& _shapeB
		*  The second input shape passed by the collisionAI.
		*  ----------------------------------------------------
		*  Algorithms:
		*  Implement the GJK and EPA Algorithms in 2D since for all points (x,0,z), y=0.
		*  ----------------------------------------------------
		*  Pseudocode:
		*  GJK(Polygon A, Polygon B)
		*  {
		*      if (A collides with B)
		*          return (Simplex, true)
		*      else
		*          return (NULL, false)
		*  }
		*
		*  EPA(Polygon A, Polygon B, Simplex S)
		*  {
		*      Calculate penetration
		*      return (penetration_depth, penetration_vector)
		*  }
		*
		*  intersect(Polygon A, Polygon B)
		*  {
		*      (Simplex, is_colliding) = GJK(A, B)
		*      if ( is_colliding == true)
		*      {
		*          (penetration_depth, penetration_vector) = EPA(A, B, Simplex)
		*          return (true, penetration_depth, penetration_vector)
		*      }
		*      else
		*      {
		*          return (false, 0, NULL)
		*      }
		*  }
		*  ----------------------------------------------------
		*  Implementation:
		*  Write your own helper functions, as long as you do not change the function definition of intersect.
		*  ONLY MODIFY THIS CLASS.
		*  DO NOT LEAVE ANY DEBUG PRINTS IN YOUR CODE IN THIS CLASS.
		*  ----------------------------------------------------
		*  Testing:
		*  In order to test your code, execute the command, inside /build/bin
		*  ./steersim -testcase polygons_test -ai collisionAI
		*
		*  The output you see, should be:
		*
		*  $ ./steersim -testcase polygons_test -ai collisionAI
		*
		*  loaded module collisionAI
		*  loaded module testCasePlayer
		*  Initializing...
		*  Preprocessing...
		*   Collision detected between polygon No.0 and No.1 with a penetration depth of 2 and penetration vector of (-0,0,-1)
		*  Simulation is running...
		*  ----------------------------------------------------
		*  Additional Notes:
		*  Type of vertices are Vector which you can check Vector class in util/Geometry.h to get familier with overloaded operators
		*  intersection will be called for all polygons, for all unique pairs, ie. if we have 3 polygons it is called 3 times
		*  one between polygon No. 1 and No. 2
		*  one between polygon No. 1 and No. 3
		*  one between polygon No. 2 and No. 3
		*  For grading your results will be compared by our results with testcase polygons1, so please run your code with
		*  original version of polygon1
		*  DO NOT MODIFY polygon1.xml
		*/
		static bool intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);

	private:
		////Return the distance from origin to a hyperplane (line in 2D)
		//float lineToOriginDistance(std::vector<Util::Vector>& endpoints);

		//GJK algorithm, if _shapeA intersect _shapeB, return true and return_simplexwill be a simplex in the M sum that contains the origin, else return false
		static bool algorithmGJK(Simplex& return_simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);

		//Calculate the point on _shape which has the highest dot product with vector direction.
		static void support(Util::Vector& maximizer, const Util::Vector direction, const std::vector<Util::Vector>& _shape);

		//EPA, calculate the penetration depth and penetration vector
		static void algorithmEPA(float& return_penetration_depth, Util::Vector& return_penetration_vector,
			const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB,
			const Simplex& s);
	}; // class GJK_EPA

} // namespace SteerLib


#endif /* GJK_EPA_H_ */
