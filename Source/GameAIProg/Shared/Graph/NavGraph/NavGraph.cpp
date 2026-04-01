#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigation mesh and create nodes
	auto const& Edges = pNavPoly->GetEdges();
	auto const& Triangles = pNavPoly->GetTriangles();

	for (int EdgeIdx = 0; EdgeIdx < static_cast<int>(Edges.size()); ++EdgeIdx)
	{
		TriPolygon::Edge const& Edge = Edges[EdgeIdx];

		int SharedCount = 0;
		for (TriPolygon::Triangle const& Triangle : Triangles)
		{
			if (Triangle.HasEdge(Edge))
				++SharedCount;
		}

		if (SharedCount < 2)
			continue;

		// Create node here
		FVector2D const P1{Edge.GetP1(*pNavPoly)};
		FVector2D const P2{Edge.GetP2(*pNavPoly)};
		AddNode(std::make_unique<NavGraphNode>((P1 + P2) / 2.f, EdgeIdx));
	}

	//2. Create connections now that every node is created	
	for (TriPolygon::Triangle const& Triangle : Triangles)
	{
		std::vector<int> NodeIds{};
		for (TriPolygon::Edge const& Edge : Triangle.GetEdges())
		{
			std::optional<int> EdgeIdx = pNavPoly->FindEdgeIndex(Edge);
			if (!EdgeIdx.has_value())
				continue;

			int NodeId = GetNodeIdFromEdgeIndex(EdgeIdx.value());
			if (NodeId != Graphs::InvalidNodeId)
				NodeIds.push_back(NodeId);
		}

		//2 valid nodes -> 1 connection
		//3 valid nodes -> 3 connections
		for (int i = 0; i < static_cast<int>(NodeIds.size()); ++i)
		{
			for (int j = i + 1; j < static_cast<int>(NodeIds.size()); ++j)
			{
				AddConnection(NodeIds[i], NodeIds[j]);
			}
		}
	}
		
	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}
