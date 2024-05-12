#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../Framework/TileSystem/Tile.h"
#include "../Framework/TileSystem/TileMap.h"
#include "../PriorityQueue.h"

namespace ufl_cap4053
{
	namespace searches
	{
		class PathSearch
		{
		// CLASS DECLARATION GOES HERE
			class PlannerNode {
				friend class PathSearch;
				Tile* vertex;
				PlannerNode* parent;
				double heurCost;
				double givenCost;
				double finalCost;
				PlannerNode() {
					vertex = nullptr;
					parent = nullptr;
					heurCost = 0;
					givenCost = 0;
					finalCost = 0;
				}
				PlannerNode(Tile* vertex) {
					this->vertex = vertex;
					parent = nullptr;
					heurCost = 0;
					givenCost = 0;
					finalCost = 0;
				}
			};
			Tile* start;
			Tile* goal;
			bool solFound;
			bool begin;
			double tileRadius;
			double hWeight;
			std::vector<std::vector<PlannerNode*>> map;
			std::unordered_map<Tile*, std::vector<Tile*>> adjTiles;
			std::vector<Tile const*> solution;
			std::unordered_map<Tile*, PlannerNode*> visited;
			PriorityQueue<PlannerNode*> open;
			bool areAdjacent(Tile* const& lhs, Tile* const& rhs);
			static bool greaterThan(PlannerNode* const &lhs, PlannerNode* const &rhs);
			std::vector<Tile*> findAdj(Tile* const& curr);
			double estimate(Tile* const& lhs, Tile* const& rhs);
			double getEdgeCost(Tile* const& tile);
			public:
				DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h
				DLLEXPORT ~PathSearch();
				DLLEXPORT void load(TileMap* _tileMap);
				DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
				DLLEXPORT void update(long timeslice);
				DLLEXPORT void shutdown();
				DLLEXPORT void unload();
				DLLEXPORT bool isDone() const;
				DLLEXPORT std::vector<Tile const*> const getSolution() const;

		};
	}
}  // close namespace ufl_cap4053::searches
