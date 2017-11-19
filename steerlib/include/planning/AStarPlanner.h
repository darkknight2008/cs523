//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>
#include <Windows.h>
#include "SteerLib.h"

namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		g : the cost from the start, for the node
		f : the f value of the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double g;
			double f;
			double rhs;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode() { };
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				g = _g;
				f = _f;
				point = _point;
				parent = _parent;
				rhs = 9999999999999999;
			}

			//bool operator<(AStarPlannerNode other) const
		 //   {
		 //       return this->f + 0.0001 * this->g > other.f + 0.0001 * other.g;
		 //   }
		 //   bool operator>(AStarPlannerNode other) const
		 //   {
			//	return this->f + 0.0001 * this->g < other.f + 0.0001 * other.g;
		 //   }
		    bool operator==(const AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			AStarPlanner(SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
			{
				gSpatialDatabase = _gSpatialDatabase;
			};
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id);
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, std::unordered_map <int, bool> new_wall = {}, std::unordered_map <int, bool> new_palce = {}, bool append_to_path = false);
		private:
			SteerLib::SpatialDataBaseInterface * dgSpatialDatabase;
			std::vector <AStarPlannerNode*> dOPEN;
			std::unordered_map <int, AStarPlannerNode*> dCLOSE;
			std::unordered_map <int, AStarPlannerNode*> dgVALUE;
			AStarPlannerNode* dStart;
			AStarPlannerNode dGoal;
			float dKey(AStarPlannerNode *p);
			void UpdateState(Util::Point u);
			bool dCompare(AStarPlannerNode *p, AStarPlannerNode *q);
			void dGeneratePath(AStarPlannerNode& Start, AStarPlannerNode& Goal, std::vector<Util::Point>& agent_path);

			float infty = 9999999999999999;

			float getH(Util::Point a, Util::Point b);

			bool weightedAstar(float epsilon, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			bool ARAstar(float time_limit, float init_epsilon, float decreaseRate, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			bool ADstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, std::unordered_map<int, bool>& new_wall, std::unordered_map <int, bool>& new_palce, bool append_to_path = false);
	
			void generatePath(AStarPlannerNode& Start, AStarPlannerNode& Goal, std::vector<Util::Point>& agent_path);
			std::list<Util::Point> getNeighoburs(Util::Point v, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase);
	};

	class STEERLIB_API MinHeap
	{
	private:
		std::vector<AStarPlannerNode*> _vector;
		void BubbleDown(int index);
		void BubbleUp(int index);

	public:
		int count = 0;
		void Insert(AStarPlannerNode* newNode);
		AStarPlannerNode* GetMin();
		void changeKey(AStarPlannerNode* changeNode);
		void DeleteMin();
	};
	


}


#endif
