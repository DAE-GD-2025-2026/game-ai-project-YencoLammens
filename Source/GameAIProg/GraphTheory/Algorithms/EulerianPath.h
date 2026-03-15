#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		// TODO If the graph is not connected, there can be no Eulerian Trail
		if (!IsConnected())
			return Eulerianity::notEulerian;
		
		// TODO Count nodes with odd degree 
		int oddDegreeCount = 0;
		for (Node* pNode : m_pGraph->GetActiveNodes())
		{
			int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(pNode->GetId()).size());
			if (degree % 2 != 0)
				++oddDegreeCount;
		}

		// TODO A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddDegreeCount > 2)
			return Eulerianity::notEulerian;
		
		// TODO A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// TODO An Euler trail can be made, but only starting and ending in these 2 nodes
		if (oddDegreeCount == 2)
			return Eulerianity::semiEulerian;
		
		// TODO A connected graph with no odd nodes is Eulerian
		return Eulerianity::eulerian;
	}

	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		// TODO Check if there can be an Euler path
		eulerianity = IsEulerian();
		
		// TODO If this graph is not eulerian, return the empty path
		if (eulerianity == Eulerianity::notEulerian)
			return Path;
		
		// TODO Start algorithm loop
		if (eulerianity == Eulerianity::eulerian)
		{
			currentNodeId = Nodes[0]->GetId();
		}
		else
		{
			for (Node* pNode : Nodes)
			{
				if (graphCopy.FindConnectionsFrom(pNode->GetId()).size() % 2 != 0)
				{
					currentNodeId = pNode->GetId();
					break;
				}
			}
		}

		std::stack<int> nodeStack;

		while (!graphCopy.FindConnectionsFrom(currentNodeId).empty() || !nodeStack.empty())
		{
			if (!graphCopy.FindConnectionsFrom(currentNodeId).empty())
			{
				nodeStack.push(currentNodeId);
				int neighborId = graphCopy.FindConnectionsFrom(currentNodeId)[0]->GetToId();
				graphCopy.RemoveConnection(currentNodeId, neighborId);
				currentNodeId = neighborId;
			}
			else
			{
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}

		Path.push_back(m_pGraph->GetNode(currentNodeId).get());

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex ) const
	{
		// TODO Mark the visited node

		// TODO Ask the graph for the connections from that node
		// TODO recursively visit any valid connected nodes that were not visited before
		// TODO Tip: use an index-based for-loop to find the correct index
		
		visited[startIndex] = true;

		std::vector<Connection*> connections = m_pGraph->FindConnectionsFrom(Nodes[startIndex]->GetId());
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			for (Connection* pConnection : connections)
			{
				if (Nodes[i]->GetId() == pConnection->GetToId() && !visited[i])
				{
					VisitAllNodesDFS(Nodes, visited, i);
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;

		// TODO choose a starting node
		int startIndex = 0;
		// TODO start a depth-first-search traversal from the node that has at least one connection
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}
		std::vector<bool> visited(Nodes.size(), false);
		VisitAllNodesDFS(Nodes, visited, startIndex);
		
		// TODO if a node was never visited, this graph is not connected
		for (bool wasVisited : visited)
		{
			if (!wasVisited)
				return false;
		}
		return true;
		
		
	
		
	}
}