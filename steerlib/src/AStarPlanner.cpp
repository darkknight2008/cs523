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
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;


		float epsilon = 1;
		return weightedAstar(epsilon, agent_path, start, goal, _gSpatialDatabase, append_to_path);
	}

	bool AStarPlanner::weightedAstar(float epsilon, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		float infty = 9999999999999999;

		std::vector <AStarPlannerNode*> OPEN;
		std::unordered_map <int, AStarPlannerNode*> CLOSE;
		std::unordered_map <int, AStarPlannerNode*> gVALUE;

		std::list<Util::Point> neighbours;

		AStarPlannerNode Start = AStarPlannerNode(start, 0, epsilon * getH(start, goal), NULL);
		AStarPlannerNode Goal = AStarPlannerNode(goal, infty, infty + epsilon * getH(start, goal), NULL);

		gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Start.point)] = &Start;
		gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Goal.point)] = &Goal;

		OPEN.push_back(&Goal);
		OPEN.push_back(&Start);

		AStarPlannerNode *Vp;
		AStarPlannerNode *Up;

		while (!OPEN.empty())
		{
			Vp = OPEN[OPEN.size() - 1];

			OPEN.pop_back();

			if (Vp->point == Goal.point)
			{
				if (!Vp->parent)
				{
					return false;
				}
				generatePath(Start, Goal, agent_path);
				return true;
			}

			//std::cerr << Vp->point << std::endl;

			CLOSE[_gSpatialDatabase->getCellIndexFromLocation(Vp->point)] = Vp;
			neighbours = getNeighoburs(Vp->point, _gSpatialDatabase);
			for (std::list<Util::Point>::iterator u = neighbours.begin(); u != neighbours.end(); ++u)
			{
				if (CLOSE.find(_gSpatialDatabase->getCellIndexFromLocation(*u)) == CLOSE.end())
				{
					// if u is not in CLOSE, else do nothing
					if (canBeTraversed(_gSpatialDatabase->getCellIndexFromLocation(*u)))
					{
						float g = Vp->g + Util::distanceBetween(Vp->point, *u);
						if (gVALUE.find(_gSpatialDatabase->getCellIndexFromLocation(*u)) == gVALUE.end())
						{
							// u is a new node
							AStarPlannerNode *p = Vp;
							AStarPlannerNode *Up = new AStarPlannerNode(*u, g, g + epsilon * getH(*u, goal), p);
							gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Up->point)] = Up;

							OPEN.push_back(Up);
							std::sort(OPEN.begin(), OPEN.end(), [](AStarPlannerNode *a, AStarPlannerNode *b) {return (a->f + 0.0000001 * a->g > b->f + 0.0000001 * b->g); });
						}
						else
						{
							Up = gVALUE.find(_gSpatialDatabase->getCellIndexFromLocation(*u))->second;
							if (g < Up->g)
							{
								Up->f += g - Up->g;
								Up->g = g;
								Up->parent = Vp;
								std::sort(OPEN.begin(), OPEN.end(), [](AStarPlannerNode *a, AStarPlannerNode *b) {return (a->f + 0.0000001 * a->g > b->f + 0.0000001 * b->g); });
							}
						}
					}
					else
					{
						//if not reachable
						AStarPlannerNode U = AStarPlannerNode(*u, infty, infty, NULL);
						CLOSE[_gSpatialDatabase->getCellIndexFromLocation(U.point)] = &U;
					}
				}
			}
		}
		return false;
	}

	bool AStarPlanner::ARAstar(float init_epsilon, float decreaseRate, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		float infty = 9999999999999999;
		float epsilon = init_epsilon;

		std::vector <AStarPlannerNode*> OPEN;
		std::unordered_map <int, AStarPlannerNode*> CLOSE;
		std::unordered_map <int, AStarPlannerNode*> INCONS;
		std::unordered_map <int, AStarPlannerNode*> gVALUE;

		std::list<Util::Point> neighbours;

		AStarPlannerNode Start = AStarPlannerNode(start, 0, epsilon * getH(start, goal), NULL);
		AStarPlannerNode Goal = AStarPlannerNode(goal, infty, infty + epsilon * getH(start, goal), NULL);

		gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Start.point)] = &Start;
		gVALUE[_gSpatialDatabase->getCellIndexFromLocation(Goal.point)] = &Goal;

		OPEN.push_back(&Goal);
		OPEN.push_back(&Start);

		while (true)
		{
			for (std::unordered_map <int, AStarPlannerNode*>::iterator income = INCONS.begin(); income != INCONS.end(); ++income)
			{

			}


			if (epsilon == 1)
			{
				break;
			}
			else
			{
				epsilon = MAX(1, epsilon / decreaseRate);
			}
		}


		return true;
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
	
	void AStarPlanner::generatePath(AStarPlannerNode& Start, AStarPlannerNode& Goal, std::vector<Util::Point>& agent_path)
	{
		AStarPlannerNode *p = &Goal;

		while (p->point != Start.point)
		{
			agent_path.push_back(p->point);
			p = p->parent;
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