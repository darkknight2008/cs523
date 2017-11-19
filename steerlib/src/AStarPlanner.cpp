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
		////float epsilon = 1;
		////return weightedAstar(epsilon, agent_path, start, goal, _gSpatialDatabase, append_to_path);
		/*MinHeap minHeap;
		AStarPlannerNode p = AStarPlannerNode(Util::Point(1, 0, 1), 5, 3, NULL);
		minHeap.Insert(&p);
		AStarPlannerNode* q = new AStarPlannerNode(Util::Point(2, 0, 1), 0,5, NULL);
		minHeap.Insert(q);
		AStarPlannerNode* o = new AStarPlannerNode(Util::Point(1,0,1), 0, 10, NULL);
		minHeap.changeKey(o);
		Util::Point Vp = minHeap.GetMin();
		std::cerr << Vp << std::endl;*/
		float init_epsilon = 8;
		float decreaseRate = 8;
		return ARAstar(init_epsilon, decreaseRate, agent_path, start, goal, _gSpatialDatabase, append_to_path);
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

	bool AStarPlanner::ADstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		//todo
		return false;
	}
	void MinHeap::BubbleDown(int index)
	{
		int leftChildIndex = 2 * index + 1;
		int rightChildIndex = 2 * index + 2;

		if (leftChildIndex >= count)
			return; //index is a leaf

		int minIndex = index;

		if (_vector[index]->f > _vector[leftChildIndex]->f)
		{
			minIndex = leftChildIndex;
		}

		if ((_vector[index]->f == _vector[leftChildIndex]->f) && (_vector[index]->g > _vector[leftChildIndex]->g))
		{
			minIndex = leftChildIndex;
		}

		if ((rightChildIndex < count) && (_vector[minIndex]->f > _vector[rightChildIndex]->f))
		{
			minIndex = rightChildIndex;
		}
		if ((rightChildIndex < count) && (_vector[minIndex]->f == _vector[rightChildIndex]->f) && (_vector[minIndex]->g > _vector[rightChildIndex]->g))
		{
			minIndex = rightChildIndex;
		}

		if (minIndex != index)
		{
			//need to swap
			AStarPlannerNode* temp = _vector[index];
			_vector[index] = _vector[minIndex];
			_vector[minIndex] = temp;
			BubbleDown(minIndex);
		}
	}
	void MinHeap::BubbleUp(int index)
	{
		if (index == 0)
			return;

		int parentIndex = (index - 1) / 2;

		if (_vector[parentIndex]->f > _vector[index]->f)
		{
			AStarPlannerNode* temp = _vector[parentIndex];
			_vector[parentIndex] = _vector[index];
			_vector[index] = temp;
			BubbleUp(parentIndex);
		}
		else if ((_vector[parentIndex]->f == _vector[index]->f) && (_vector[parentIndex]->g > _vector[index]->g))
		{
			AStarPlannerNode* temp = _vector[parentIndex];
			_vector[parentIndex] = _vector[index];
			_vector[index] = temp;
			BubbleUp(parentIndex);
		}
	}

	void MinHeap::Insert(AStarPlannerNode* newNode)
	{
		_vector.push_back( newNode);
		count++;
		BubbleUp(count-1);
	}

	void MinHeap::changeKey(AStarPlannerNode* changeNode)
	{
		int m;
		for (int a = 0; a < count; a++)
		{
			if (_vector[a]->point == changeNode->point)
			{
				m = a;
				break;
			}
		}
		int oldF = _vector[m]->f;
		int oldG = _vector[m]->g;
		int newF = changeNode->f;
		int newG = changeNode->g;
		_vector[m] = changeNode;
		if (newF < oldF || (newF == oldF && newG < oldG))
		{
			BubbleUp(m);
		}
		else
		{
			BubbleDown(m);
		}
	}

	void MinHeap::DeleteMin()
	{
		//int length = _vector.size();
		if (count == 0)
		{
			return;
		}

		_vector[0] = _vector[count - 1];
		_vector.pop_back();
		count--;
		BubbleDown(0);
	}

	Util::Point MinHeap::GetMin()
	{
		return _vector[0]->point;
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
