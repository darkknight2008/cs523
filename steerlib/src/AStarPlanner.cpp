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



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, std::unordered_map <int, bool> new_wall, std::unordered_map <int, bool> new_palce, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		std::cerr << "\n test" << std::endl;

		return ADstar(agent_path, start, goal, _gSpatialDatabase, new_wall, new_palce);

		//return false;
		//float epsilon = 1;
		//return weightedAstar(epsilon, agent_path, start, goal, _gSpatialDatabase, append_to_path);

		//float init_epsilon = 8;
		//float decreaseRate = 8;
		//return ARAstar(init_epsilon, decreaseRate, agent_path, start, goal, _gSpatialDatabase, append_to_path);
	}

	bool AStarPlanner::weightedAstar(float epsilon, std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
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

		AStarPlannerNode *Vp;
		AStarPlannerNode *Up;

		while (true)
		{
			for (std::unordered_map <int, AStarPlannerNode*>::iterator income = INCONS.begin(); income != INCONS.end(); ++income)
			{
				OPEN.push_back(income->second);
			}
			INCONS.clear();
			for (std::vector <AStarPlannerNode*>::iterator p = OPEN.begin(); p != OPEN.end(); ++p)
			{
				AStarPlannerNode* q = *p;
				q->f = q->g + epsilon * getH(q->point, goal);
			}
			std::sort(OPEN.begin(), OPEN.end(), [](AStarPlannerNode *a, AStarPlannerNode *b) {return (a->f + 0.0000001 * a->g > b->f + 0.0000001 * b->g); });
			CLOSE.clear();

			while (!OPEN.empty())
			{
				Vp = OPEN[OPEN.size() - 1];

				//std::cerr << Vp->point << std::endl;
				//std::cerr << OPEN.size() << std::endl;
				//std::cerr << Vp->f << " abc " << Goal.f << std::endl;

				if (Vp->f >= Goal.f && !(Vp->point == Goal.point))
				{	
					break;
				}

				OPEN.pop_back();

				if (Vp->point == Goal.point)
				{
					if (!Vp->parent)
					{
						return false;
					}
					generatePath(Start, Goal, agent_path);
					break;
				}
				CLOSE[_gSpatialDatabase->getCellIndexFromLocation(Vp->point)] = Vp;
				neighbours = getNeighoburs(Vp->point, _gSpatialDatabase);
				for (std::list<Util::Point>::iterator u = neighbours.begin(); u != neighbours.end(); ++u)
				{
					//std::cerr << "check" << *u << std::endl;
					if (CLOSE.find(_gSpatialDatabase->getCellIndexFromLocation(*u)) == CLOSE.end())
					{
						// if u is not in CLOSE
						if (canBeTraversed(_gSpatialDatabase->getCellIndexFromLocation(*u)))
						{
							float g = Vp->g + epsilon * Util::distanceBetween(Vp->point, *u);
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

									OPEN.push_back(Up);
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
					else
					{
						if (canBeTraversed(_gSpatialDatabase->getCellIndexFromLocation(*u)))
						{
							//if u is already in closed set and it a valid position, insert it into INCONS
							INCONS[_gSpatialDatabase->getCellIndexFromLocation(*u)] = CLOSE[_gSpatialDatabase->getCellIndexFromLocation(*u)];
						}
					}
				}
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

	bool AStarPlanner::ADstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, std::unordered_map <int, bool>& new_wall, std::unordered_map <int, bool>& new_palce, bool append_to_path)
	{
		dgSpatialDatabase = _gSpatialDatabase;
		std::list<Util::Point> neighbours;
		AStarPlannerNode *Vp;
		AStarPlannerNode *Up;
		dStart = AStarPlannerNode(start, infty, infty + getH(start, start), NULL);
		if (new_wall.size() == 0 && new_palce.size() == 0)
		{
			std::cerr << "1 test" << std::endl;
			
			dGoal = AStarPlannerNode(goal, 0, getH(goal, start), NULL);
			dgVALUE[_gSpatialDatabase->getCellIndexFromLocation(dStart.point)] = &dStart;
			dgVALUE[_gSpatialDatabase->getCellIndexFromLocation(dGoal.point)] = &dGoal;
			dGoal.rhs = 0;

			dOPEN.push_back(&dStart);
			dOPEN.push_back(&dGoal);
		}
		else
		{
			for (std::unordered_map <int, bool>::iterator it = new_wall.begin(); it != new_wall.end(); ++it)
			{
				Util::Point n;
				_gSpatialDatabase->getLocationFromIndex(it->first, n);
				neighbours = getNeighoburs(n, _gSpatialDatabase);
				for (std::list<Util::Point>::iterator u = neighbours.begin(); u != neighbours.end(); ++u)
				{
					UpdateState(*u);
				}
			}
			for (std::unordered_map <int, bool>::iterator it = new_palce.begin(); it != new_palce.end(); ++it)
			{
				Util::Point n;
				_gSpatialDatabase->getLocationFromIndex(it->first, n);
				neighbours = getNeighoburs(n, _gSpatialDatabase);
				for (std::list<Util::Point>::iterator u = neighbours.begin(); u != neighbours.end(); ++u)
				{
					UpdateState(*u);
				}
			}
			
			//do the update
		}

		while (!dOPEN.empty() || dStart.rhs != dStart.g)
		{
			Vp = dOPEN[dOPEN.size() - 1];

			dOPEN.pop_back();

			std::cerr << "2 test" << std::endl;
			std::cerr << Vp->point << std::endl;

			if (Vp->point == dStart.point)
			{
				if (!Vp->parent)
				{
					return false;
				}
				std::cerr << "almost!" << std::endl;
				std::cerr << Vp->point << std::endl;
				generatePath(dGoal, dStart, agent_path);
				std::reverse(agent_path.begin(), agent_path.end());
				return true;
			}

			dCLOSE[dgSpatialDatabase->getCellIndexFromLocation(Vp->point)] = Vp;
			neighbours = getNeighoburs(Vp->point, dgSpatialDatabase);

			if (Vp->g > Vp->rhs)
			{
				Vp->g = Vp->rhs;
			}
			else
			{
				Vp->g = infty;
				std::cerr << "2.5 test" << std::endl;
				UpdateState(Vp->point);
			}
			for (std::list<Util::Point>::iterator u = neighbours.begin(); u != neighbours.end(); ++u)
			{
				std::cerr << "3 test" << *u << std::endl;
				UpdateState(*u);
			}
			
		}
		return false;
	}

	float AStarPlanner::dKey(AStarPlannerNode *p)
	{
		return MIN(p->g, p->rhs) + 0.9999 * getH(p->point, dStart.point);
	}

	bool  AStarPlanner::dCompare(AStarPlannerNode *p, AStarPlannerNode *q)
	{
		return dKey(p) > dKey(q);
	}

	void AStarPlanner::UpdateState(Util::Point u)
	{
		AStarPlannerNode *U;
		if (dgVALUE.find(dgSpatialDatabase->getCellIndexFromLocation(u)) == dgVALUE.end())
		{
			//new node
			U = new AStarPlannerNode(u, infty, infty + getH(u, dStart.point), NULL);
			dgVALUE[dgSpatialDatabase->getCellIndexFromLocation(U->point)] = U;
		}
		else
		{
			U = dgVALUE.find(dgSpatialDatabase->getCellIndexFromLocation(u))->second;
		}
		if (U->point != dGoal.point)
		{
			U->rhs = 10 * infty;
			std::list<Util::Point> neighbours = getNeighoburs(u, gSpatialDatabase);
			for (std::list<Util::Point>::iterator w = neighbours.begin(); w != neighbours.end(); ++w)
			{
				if (!(dgVALUE.find(gSpatialDatabase->getCellIndexFromLocation(*w)) == dgVALUE.end()))
				{
					//if we see that Node before
					if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(u)) && canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(*w)))
					{
						U->rhs = MIN(U->rhs, dgVALUE[gSpatialDatabase->getCellIndexFromLocation(*w)]->g + Util::distanceBetween(u, *w));
						std::cerr << "4 test" << u << *w << U->rhs << std::endl;
					}
				}
			}
		}
		//dOPEN.erase(std::find(dOPEN.begin(), dOPEN.end(), U));
		dOPEN.erase(std::remove(dOPEN.begin(), dOPEN.end(), U), dOPEN.end());
		if (U->g != U->rhs)
		{
			dOPEN.push_back(U);
			std::sort(dOPEN.begin(), dOPEN.end(), 
				[&](AStarPlannerNode *p, AStarPlannerNode *q) {return (MIN(p->g, p->rhs) + 0.9999 * getH(p->point, dStart.point) > MIN(q->g, q->rhs) + 0.9999 * getH(q->point, dStart.point)); }
				);
		}
		return;
	}

	float AStarPlanner::getH(Util::Point a, Util::Point b)
	{
		return Util::distanceBetween(a, b);
	}
	
	void AStarPlanner::generatePath(AStarPlannerNode& Start, AStarPlannerNode& Goal, std::vector<Util::Point>& agent_path)
	{
		agent_path.clear();

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