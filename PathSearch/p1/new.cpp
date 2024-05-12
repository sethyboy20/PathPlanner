#include <chrono>
#include <iostream>
#include "PathSearch.h"

namespace ufl_cap4053
{
	namespace searches
	{
		// CLASS DEFINITION GOES HERE
		PathSearch::PathSearch() : open(greaterThan) {
			start = nullptr;
			goal = nullptr;
			tileRadius = 0.0;
			begin = false;
			solFound = false;
			hWeight = 1.2;
		}

		PathSearch::~PathSearch() {
			unload();
		}

		void PathSearch::load(TileMap* _tileMap) {
			// load map
			for (int i = 0; i < _tileMap->getRowCount(); i++) {
				std::vector<PlannerNode*> vec;
				map.push_back(vec);
				for (int j = 0; j < _tileMap->getColumnCount(); j++) {
					map[i].push_back(new PlannerNode(_tileMap->getTile(i, j)));
				}
			}
			
			// find/load adj. tiles for each map tile
			for (int i = 0; i < _tileMap->getRowCount(); i++) {
				for (int j = 0; j < _tileMap->getColumnCount(); j++) {
					adjTiles[map[i][j]->vertex] = findAdj(map[i][j]->vertex);
				}
			}
			
			// get tile radius
			tileRadius = _tileMap->getTileRadius();
		}

		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
			start = map[startRow][startCol]->vertex;
			goal = map[goalRow][goalCol]->vertex;
			solFound = false;
			begin = false;
			open.push(new PlannerNode(start));
			open.front()->givenCost = 0;
			open.front()->heurCost = estimate(start, goal);
			open.front()->finalCost = open.front()->givenCost + (open.front()->heurCost * hWeight);
			visited[start] = open.front();
		}

		void PathSearch::update(long timeslice) {
			typedef std::chrono::high_resolution_clock Time;
			typedef std::chrono::milliseconds ms;
			typedef std::chrono::duration<float> fs;
			long time = 0;
			fs diff;

			while (time <= timeslice) {
				Time::time_point startTime = Time::now();
				PlannerNode* curr = open.front();
				Tile* currTile = curr->vertex;

				// fill node as black
				currTile->setFill(0x000000FF);
				open.pop();

				if (curr->vertex == goal) {
					solFound = true;
					PlannerNode* node = curr;
					while (node != nullptr) {
						solution.push_back(node->vertex);
						node = node->parent;
					}
					return;
				}
				else {
					for (int i = 0; i < adjTiles[currTile].size(); i++) {
						Tile* successorTile = adjTiles[currTile][i];
						double tempGivenCost = curr->givenCost + getEdgeCost(successorTile);

						if (visited.find(successorTile) != visited.end()) {
							if (tempGivenCost < visited[successorTile]->givenCost) {
								PlannerNode* successor = visited[successorTile];
								open.remove(successor);
								successor->givenCost = tempGivenCost;
								successor->finalCost = successor->givenCost + (successor->heurCost * hWeight);
								successor->parent = curr;
								open.push(successor);
							}
						}
						else {
							PlannerNode* successor = map[successorTile->getRow()][successorTile->getColumn()];
							// set successor node as green
							successor->vertex->setFill(0xFF00FF00);
							successor->parent = curr;
							successor->givenCost = tempGivenCost;
							successor->heurCost = estimate(successorTile, goal);
							successor->finalCost = successor->givenCost + (successor->heurCost * hWeight);
							visited[successorTile] = successor;
							open.push(successor);
						}
					}
				}
				Time::time_point endTime = Time::now();
				diff = endTime - startTime;
				ms duration = std::chrono::duration_cast<ms>(diff);
				time += (long)duration.count();
				if (time >= timeslice)
					break;
			}
		}

		void PathSearch::shutdown() {
			// clear visited map
			for (auto it = visited.begin(); it != visited.end(); ) {
				it->second->parent = nullptr;
				it->second->heurCost = 0;
				it->second->givenCost = 0;
				it->second->finalCost = 0;
				it = visited.erase(it);
			}

			// clear PQ
			while (!open.empty()) {
				open.front()->parent = nullptr;
				open.front()->heurCost = 0;
				open.front()->givenCost = 0;
				open.front()->finalCost = 0;
				open.pop();
			}

			// clear solution
			solution.clear();
		}

		void PathSearch::unload() {
			for (int i = 0; i < map.size(); i++) {
				for (int j = 0; j < map[i].size(); j++) {
					delete map[i][j];
				}
			}
			map.clear();
			adjTiles.clear();
		}

		bool PathSearch::isDone() const {
			if (solFound || open.empty())
				return true;

			return false;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const {
			return solution;
		}

		std::vector<Tile*> PathSearch::findAdj(Tile* const& curr) {
			std::vector<Tile*> adj;
			int row = curr->getRow();
			int col = curr->getColumn();
			int i = row - 1, j = col - 1;
			int rowEnd = row + 1, colEnd = col + 1;
			if (i < 0)
				i = 0;
			if (j < 0)
				j = 0;
			if (rowEnd >= map.size())
				rowEnd = (int)map.size() - 1;
			if (colEnd >= map[0].size())
				colEnd = (int)map[0].size() - 1;
			int temp = j;

			for (; i <= rowEnd; i++) {
				for (j = temp; j <= colEnd; j++) {
					Tile* tile = map[i][j]->vertex;
					if (!(i == row && j == col) && tile->getWeight() > 0 && areAdjacent(curr, tile))
						adj.push_back(tile);
				}
			}
			return adj;
		}

		bool PathSearch::areAdjacent(Tile* const& lhs, Tile* const& rhs) {
			int rowDiff = -(lhs->getRow() - rhs->getRow());
			int colDiff = -(lhs->getColumn() - rhs->getColumn());
			if (abs(rowDiff) == 1 && colDiff == -1 && lhs->getRow() % 2 != 0)
				return false;
			else if (abs(rowDiff) == 1 && colDiff == 1 && lhs->getRow() % 2 == 0)
				return false;
			else
				return true;
		}

		bool PathSearch::greaterThan(PlannerNode* const& lhs, PlannerNode* const& rhs)
		{
			return lhs->finalCost > rhs->finalCost;
		}

		double PathSearch::estimate(Tile* const& lhs, Tile* const& rhs) {
			return sqrt(pow(lhs->getXCoordinate() - rhs->getXCoordinate(), 2) + pow(lhs->getYCoordinate() - rhs->getYCoordinate(), 2));
		}

		double PathSearch::getEdgeCost(Tile* const& tile) {
			return tileRadius * 2 * tile->getWeight();
		}
	}
}  // close namespace ufl_cap4053::searches
