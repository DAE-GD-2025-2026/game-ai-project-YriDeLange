#include "BFS.h"

#include <map>
#include <queue>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path{};
	std::queue<Node*> openList{};
	std::map<Node*, Node*> parent{};

	openList.push(pStartNode);
	parent[pStartNode] = nullptr;

	while (!openList.empty())
	{
		Node* pCurrent = openList.front();
		openList.pop();

		if (pCurrent == pDestinationNode)
		{
			Node* pStep = pDestinationNode;
			while (pStep != nullptr)
			{
				path.push_back(pStep);
				pStep = parent[pStep];
			}
			std::reverse(path.begin(), path.end());
			return path;
		}

		for (Connection* pConnection : pGraph->FindConnectionsFrom(pCurrent->GetId()))
		{
			Node* pNeighbor = pGraph->GetNode(pConnection->GetToId()).get();

			if (parent.find(pNeighbor) == parent.end())
			{
				parent[pNeighbor] = pCurrent;
				openList.push(pNeighbor);
			}
		}
	}

	return path;
}