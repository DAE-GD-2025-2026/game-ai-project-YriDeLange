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
	std::vector<TriPolygon::Edge> const& Edges = pNavPoly->GetEdges();
	std::vector<TriPolygon::Triangle> const& Triangles = pNavPoly->GetTriangles();

	for (int EdgeIdx = 0; EdgeIdx < static_cast<int>(Edges.size()); ++EdgeIdx)
	{
		TriPolygon::Edge const& Edge = Edges[EdgeIdx];

		int TriangleCount = 0;
		for (TriPolygon::Triangle const& Triangle : Triangles)
		{
			if (Triangle.HasEdge(Edge))
				++TriangleCount;
		}

		if (TriangleCount < 2)
			continue;

		FVector const P1 = Edge.GetP1(*pNavPoly);
		FVector const P2 = Edge.GetP2(*pNavPoly);
		FVector2D const Midpoint = FVector2D{ (P1.X + P2.X) / 2.f, (P1.Y + P2.Y) / 2.f };

		AddNode(std::make_unique<NavGraphNode>(Midpoint, EdgeIdx));
	}

	for (TriPolygon::Triangle const& Triangle : Triangles)
	{
		std::array<TriPolygon::Edge, 3> TriEdges = Triangle.GetEdges();

		std::vector<int> NodeIds{};
		for (TriPolygon::Edge const& Edge : TriEdges)
		{
			int EdgeIdx = pNavPoly->FindEdgeIndex(Edge).value_or(Graphs::InvalidNodeId);
			int NodeId = GetNodeIdFromEdgeIndex(EdgeIdx);
			if (NodeId != Graphs::InvalidNodeId)
			{
				NodeIds.push_back(NodeId);
			}
		}

		for (int i = 0; i < static_cast<int>(NodeIds.size()); ++i)
		{
			for (int j = i + 1; j < static_cast<int>(NodeIds.size()); ++j)
			{
				AddConnection(NodeIds[i], NodeIds[j]);
			}
		}
	}

	SetConnectionCostsToDistances();
}
