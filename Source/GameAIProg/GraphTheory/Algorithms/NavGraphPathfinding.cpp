#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	FVector2D startOutPos{};
	FVector2D endOutPos{};
	TriPolygon::Triangle const* pStartTriangle = pNavGraph->GetNavPolygon()->GetClosestTriangleToPosition(startPos, startOutPos);
	TriPolygon::Triangle const* pEndTriangle = pNavGraph->GetNavPolygon()->GetClosestTriangleToPosition(endPos, endOutPos);

	//We have valid start/end triangles and they are not the same
	if (!pStartTriangle || !pEndTriangle)
		return finalPath;

	if (pStartTriangle == pEndTriangle)
	{
		finalPath.push_back(startOutPos);
		finalPath.push_back(endOutPos);
		return finalPath;
	}
	
	//=> Start looking for a path
	//Copy the graph
	std::unique_ptr<NavGraph> pClonedGraph = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position)
	int startNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(startOutPos, -1));
	for (TriPolygon::Edge const& Edge : pStartTriangle->GetEdges())
	{
		std::optional<int> EdgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(Edge);
		if (!EdgeIdx.has_value())
			continue;
		int neighborNodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx.value());
		if (neighborNodeId == Graphs::InvalidNodeId)
			continue;
		auto pConnection = std::make_unique<Connection>(startNodeId, neighborNodeId);
		pConnection->SetWeight(FVector2D{startOutPos - pClonedGraph->GetNode(neighborNodeId)->GetPosition()}.Length());
		pClonedGraph->AddConnection(std::move(pConnection));
	}

	//Create extra node for the endNode
	int endNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(endOutPos, -1));
	for (TriPolygon::Edge const& Edge : pEndTriangle->GetEdges())
	{
		std::optional<int> EdgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(Edge);
		if (!EdgeIdx.has_value())
			continue;
		int neighborNodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx.value());
		if (neighborNodeId == Graphs::InvalidNodeId)
			continue;
		auto pConnection = std::make_unique<Connection>(endNodeId, neighborNodeId);
		pConnection->SetWeight(FVector2D{endOutPos - pClonedGraph->GetNode(neighborNodeId)->GetPosition()}.Length());
		pClonedGraph->AddConnection(std::move(pConnection));
	}

	//Run A star on new graph
	AStar aStar(pClonedGraph.get(), HeuristicFunctions::Euclidean);
	std::vector<Node*> nodes = aStar.FindPath(pClonedGraph->GetNode(startNodeId).get(), pClonedGraph->GetNode(endNodeId).get());

	//Debug Visualisation
	for (Node* pNode : nodes)
	{
		debugNodePositions.push_back(pNode->GetPosition());
		finalPath.push_back(pNode->GetPosition());
	}

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}