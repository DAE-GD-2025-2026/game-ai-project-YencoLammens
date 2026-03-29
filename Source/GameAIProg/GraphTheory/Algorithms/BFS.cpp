#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	std::queue<Node*> openList;
	std::map<Node*, Node*> closedList;

	openList.push(pStartNode);
	closedList[pStartNode] = nullptr;

	while (!openList.empty())
	{
		Node* pCurrentNode = openList.front();
		openList.pop();

		if (pCurrentNode == pDestinationNode)
			break;

		for (Connection* pConnection : pGraph->FindConnectionsFrom(pCurrentNode->GetId()))
		{
			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();

			if (closedList.find(pNextNode) == closedList.end())
			{
				closedList[pNextNode] = pCurrentNode;
				openList.push(pNextNode);
			}
		}
	}

	if (closedList.find(pDestinationNode) == closedList.end())
		return path;

	Node* pCurrentNode = pDestinationNode;
	while (pCurrentNode != nullptr)
	{
		path.push_back(pCurrentNode);
		pCurrentNode = closedList[pCurrentNode];
	}

	std::reverse(path.begin(), path.end());
	return path;
}
