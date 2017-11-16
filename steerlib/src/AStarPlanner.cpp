//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id , SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		_gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, _gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, _gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = _gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += _gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		////TODO
		//std::cout<<"\nIn A*";

		//return false;
		//agent_path.push_back(start);
		//agent_path.push_back(goal);
		//return true;

		float epsilon = 1;
		return weightedAstar(epsilon, agent_path, start, goal, _gSpatialDatabase, append_to_path);
	}

	bool AStarPlanner::weightedAstar(float epsilon, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		float infty = 9999999999999999;

		//std::priority_queue<AStarPlannerNode, std::vector<AStarPlannerNode>, Compare> OPEN;
		std::map <AStarPlannerNode, bool, Compare> OPEN;
		//std::multimap <float, AStarPlannerNode> OPEN;
		std::unordered_map <int, AStarPlannerNode> CLOSE;
		std::unordered_map <int, AStarPlannerNode> gVALUE;

		std::list<Util::Point> neighbours;
		//Util::Point* u;

		AStarPlannerNode Start = AStarPlannerNode(start, 0, epsilon * getH(start, goal), NULL);
		AStarPlannerNode Goal = AStarPlannerNode(goal, infty, infty + epsilon * getH(start, goal), NULL);

		gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Start.point)] = Start;
		gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Goal.point)] = Goal;

		////OPEN.push(Start);
		//OPEN.insert(std::pair<float, AStarPlannerNode>(Start.f, Start));
		//OPEN[Start.f] = Start;
		OPEN[Start] = true;

		while (!OPEN.empty())
		{
			AStarPlannerNode V = OPEN.begin()->first;

			OPEN.erase(V);

			//auto p = &*it;
			//AStarPlannerNode& V = p->first;

			//AStarPlannerNode* V = &OPEN.begin()->second;

			//auto pair = OPEN.erase(OPEN.begin());
			//AStarPlannerNode &V = pair->second;

			//V.g = -9999;

			//std::cerr << "well well" << V.g << " and " << Start.g << std::endl;

			//AStarPlannerNode = OPEN.begin()->first;
			if (V == Goal)
			{
				//std::cerr << "FUCK!" << V.f << "???" << Goal.f << std::endl;
				//std::cerr << "FUCK!" << V.point << V.parent->point << Goal.parent->point << std::endl;
				generatePath(Start, Goal, agent_path);
				return true;
			}
			//OPEN.pop();
			

			//std::cerr << V.point << std::endl;

			CLOSE[_gSpatialDatabase->getCellIndexFromLocation(V.point)] = V;
			neighbours = getNeighoburs(V.point, _gSpatialDatabase);
			//std::cerr << "-1" << std::endl;
			for (std::list<Util::Point>::iterator u = neighbours.begin(); u != neighbours.end(); ++u)
			{
				if (CLOSE.find(_gSpatialDatabase->getCellIndexFromLocation(*u)) == CLOSE.end())
				{
					// if u is not in CLOSE, else do nothing
					if (canBeTraversed(_gSpatialDatabase->getCellIndexFromLocation(*u), _gSpatialDatabase))
					{
						float g = V.g + Util::distanceBetween(V.point, *u);
						//std::cerr << "1" << std::endl;
						if (gVALUE.find(_gSpatialDatabase->getCellIndexFromLocation(*u)) == gVALUE.end())
						{
							// u is a new node
							//std::cerr << "2" << std::endl;
							AStarPlannerNode U = AStarPlannerNode(*u, g, g + epsilon * getH(*u, goal), &V);

							//std::cerr << U.point << "'s parent is: " << V.point <<  std::endl;

							//std::cerr <<*u << "is pointing to" << p->point << std::endl;
							gVALUE[_gSpatialDatabase->getCellIndexFromLocation(U.point)] = U;
							//OPEN.push(U);
							OPEN[U] = true;
							//OPEN.insert(std::pair<float, AStarPlannerNode>(U.f, U));
							if (U == Goal)
							{
								std::cerr << "You creat a second goal you idiot!" << std::endl;
							}
						}
						else
						{
							AStarPlannerNode &U = gVALUE.find(_gSpatialDatabase->getCellIndexFromLocation(*u))->second;
							if (g < U.g)
							{
								U.f += g - U.g;
								U.g = g;
								U.parent = &V;
								//std::cerr << U.point << "'s new parent is: " << V.point << std::endl;
								OPEN[U] = true;
							}
						}
					}
					else
					{
						//if not reachable
						AStarPlannerNode U = AStarPlannerNode(*u, infty, infty, NULL);
						CLOSE[_gSpatialDatabase->getCellIndexFromLocation(U.point)] = U;
					}
				}
			}
			std::cerr << "OHOH" << std::endl;
			for (std::map<AStarPlannerNode, bool>::iterator it = OPEN.begin(); it != OPEN.end(); ++it)
			{
				std::cerr << it->first.point << "parent" << it->first.parent->point << std::endl;
			}
		}

		return false;
	}

	bool AStarPlanner::ARAstar(float init_epsilon, float decreaseRate, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		//todo
		return false;
	}

	bool AStarPlanner::ADstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		//todo
		return false;
	}

	float AStarPlanner::getH(Util::Point a, Util::Point b)
	{
		return Util::distanceBetween(a, b);
	}
	
	void AStarPlanner::generatePath(AStarPlannerNode Start, AStarPlannerNode Goal, std::vector<Util::Point>& agent_path)
	{
		//return;
		//todo
		//std::vector<Util::Point> path;

		agent_path.push_back(Goal.point);
		AStarPlannerNode * parent = Goal.parent;
		std::cerr << "Pointer?" << parent << std::endl;

		while (!(*parent == Start))
		{
			agent_path.push_back(parent->point);
			std::cerr << "path" << parent->point << std::endl;
			parent = parent->parent;
		}

		agent_path.push_back(Start.point);

		std::reverse(agent_path.begin(), agent_path.end());
		return;
	}

	std::list<Util::Point> AStarPlanner::getNeighoburs(Util::Point v, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		std::list<Util::Point> neighbours;
		unsigned int x, z;
		_gSpatialDatabase->getGridCoordinatesFromIndex(_gSpatialDatabase->getCellIndexFromLocation(v), x, z);
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				if (i != 0 || j != 0)
				{
					Util::Point neighbour;
					unsigned int x0 = x + i;
					unsigned int z0 = z + j;
					_gSpatialDatabase->getLocationFromIndex(_gSpatialDatabase->getCellIndexFromGridCoords(x0, z0), neighbour);
					neighbours.push_back(neighbour);
				}
			}
		}
		return neighbours;
	}
}