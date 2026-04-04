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
	std::vector<FVector2D> finalPath{};

	TriPolygon const* pNavPoly = pNavGraph->GetNavPolygon();

	FVector2D startTriPos{};
	TriPolygon::Triangle const* pStartTriangle = pNavPoly->GetClosestTriangleToPosition(startPos, startTriPos);

	FVector2D endTriPos{};
	TriPolygon::Triangle const* pEndTriangle = pNavPoly->GetClosestTriangleToPosition(endPos, endTriPos);

	if (!pStartTriangle || !pEndTriangle)
		return finalPath;

	if (pStartTriangle == pEndTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	std::unique_ptr<NavGraph> pClonedGraph = pNavGraph->Clone();
	TriPolygon const* pClonedNavPoly = pClonedGraph->GetNavPolygon();

	int startNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(startPos, -1));

	for (TriPolygon::Edge const& Edge : pStartTriangle->GetEdges())
	{
		int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(Graphs::InvalidNodeId);
		int NodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx);
		if (NodeId != Graphs::InvalidNodeId)
		{
			float Cost = FVector2D::Distance(startPos, pClonedGraph->GetNode(NodeId)->GetPosition());
			auto NewConnection = std::make_unique<Connection>(startNodeId, NodeId);
			NewConnection->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(NewConnection));
		}
	}

	int endNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(endPos, -1));

	for (TriPolygon::Edge const& Edge : pEndTriangle->GetEdges())
	{
		int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(Graphs::InvalidNodeId);
		int NodeId = pClonedGraph->GetNodeIdFromEdgeIndex(EdgeIdx);
		if (NodeId != Graphs::InvalidNodeId)
		{
			float Cost = FVector2D::Distance(endPos, pClonedGraph->GetNode(NodeId)->GetPosition());
			auto NewConnection = std::make_unique<Connection>(endNodeId, NodeId);
			NewConnection->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(NewConnection));

			auto InverseConnection = std::make_unique<Connection>(NodeId, endNodeId);
			InverseConnection->SetWeight(Cost);
			pClonedGraph->AddConnection(std::move(InverseConnection));
		}
	}

	AStar pathfinder{ pClonedGraph.get(), HeuristicFunctions::Euclidean };
	Node* pStartNode = pClonedGraph->GetNode(startNodeId).get();
	Node* pEndNode = pClonedGraph->GetNode(endNodeId).get();

	std::vector<Node*> nodePath = pathfinder.FindPath(pStartNode, pEndNode);

	if (nodePath.empty())
		return finalPath;

	for (Node* pNode : nodePath)
	{
		finalPath.push_back(pNode->GetPosition());
		debugNodePositions.push_back(pNode->GetPosition());
	}

	// Run SSFA path optimiser, comment out to use the raw path
	debugPortals = SSFA::FindPortals(nodePath, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}