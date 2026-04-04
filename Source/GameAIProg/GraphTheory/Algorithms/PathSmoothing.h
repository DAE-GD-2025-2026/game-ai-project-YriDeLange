#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		//=== SSFA Functions ===
		//--- References ---
		//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
		//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
		static std::vector<NavLine> FindPortals(std::vector<Node*> const& Path, TriPolygon const& NavPoly)
		{
			std::vector<NavLine> Portals = {};

			// Add degenerate start portal so OptimizePortals can use Portals[0] as the apex
			FVector2D StartPos = Path.front()->GetPosition();
			Portals.push_back(NavLine{ StartPos, StartPos });

			// Skip first (start) and last (end) nodes — they have lineIdx == -1
			// For each intermediate node, determine which vertex is right (P1) and which is left (P2)
			for (int i = 1; i < static_cast<int>(Path.size()) - 1; ++i)
			{
				NavGraphNode* pNavNode = reinterpret_cast<NavGraphNode*>(Path[i]);
				int EdgeIdx = pNavNode->GetEdgeIdx();
				if (EdgeIdx < 0) continue;

				TriPolygon::Edge const& Edge = NavPoly.GetEdges()[EdgeIdx];
				FVector2D P1 = FVector2D{ Edge.GetP1(NavPoly) };
				FVector2D P2 = FVector2D{ Edge.GetP2(NavPoly) };

				// Use travel direction (prev -> next) to determine orientation
				// Cross(TravelDir, ToP1): negative means P1 is to the right
				FVector2D PrevPos = Path[i - 1]->GetPosition();
				FVector2D NextPos = Path[i + 1]->GetPosition();
				FVector2D TravelDir = NextPos - PrevPos;
				FVector2D ToP1 = P1 - PrevPos;
				float Cross = TravelDir.X * ToP1.Y - TravelDir.Y * ToP1.X;

				NavLine Portal{};
				if (Cross <= 0.f)
				{
					// P1 is on the right side
					Portal.P1 = P1;
					Portal.P2 = P2;
				}
				else
				{
					// Swap — P2 becomes right, P1 becomes left
					Portal.P1 = P2;
					Portal.P2 = P1;
				}

				Portals.push_back(Portal);
			}

			// Add degenerate end portal to force final point evaluation
			FVector2D EndPos = Path.back()->GetPosition();
			Portals.push_back(NavLine{ EndPos, EndPos });

			return Portals;
		}

		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
		{
			std::vector<FVector2D> Path{};

			if (Portals.size() < 2) return Path;

			int amtPortals = static_cast<int>(Portals.size());

			// Apex starts at the degenerate start portal (== start position)
			FVector2D apexPoint = Portals[0].P1;

			// Initialize legs from the first real portal (index 1)
			FVector2D rightLeg = Portals[1].P1 - apexPoint;
			FVector2D leftLeg = Portals[1].P2 - apexPoint;

			int rightLegIndex = 1;
			int leftLegIndex = 1;

			// Add apex (start position) as the first path point
			Path.push_back(apexPoint);

			// Loop from the second real portal onwards
			for (int portalIdx = 2; portalIdx < amtPortals; ++portalIdx)
			{
				NavLine const& CurrentPortal = Portals[portalIdx];

				// --- RIGHT CHECK ---
				FVector2D newRightLeg = CurrentPortal.P1 - apexPoint;

				// Inwards for right side = CCW = Cross >= 0
				if (Cross2D(rightLeg, newRightLeg) >= 0.f)
				{
					// Check if it crosses over the left leg
					if (Cross2D(leftLeg, newRightLeg) > 0.f)
					{
						// Crossed left leg — left leg vertex becomes new apex
						apexPoint = apexPoint + leftLeg;

						portalIdx = leftLegIndex + 1;
						leftLegIndex = portalIdx;
						rightLegIndex = portalIdx;

						Path.push_back(apexPoint);

						if (portalIdx < amtPortals)
						{
							rightLeg = Portals[rightLegIndex].P1 - apexPoint;
							leftLeg = Portals[leftLegIndex].P2 - apexPoint;
							continue;
						}
					}
					else
					{
						// Tightening funnel on the right — update right leg
						rightLeg = newRightLeg;
						rightLegIndex = portalIdx;
					}
				}

				// --- LEFT CHECK ---
				FVector2D newLeftLeg = CurrentPortal.P2 - apexPoint;

				// Inwards for left side = CW = Cross <= 0
				if (Cross2D(leftLeg, newLeftLeg) <= 0.f)
				{
					// Check if it crosses over the right leg
					if (Cross2D(rightLeg, newLeftLeg) < 0.f)
					{
						// Crossed right leg — right leg vertex becomes new apex
						apexPoint = apexPoint + rightLeg;

						portalIdx = rightLegIndex + 1;
						rightLegIndex = portalIdx;
						leftLegIndex = portalIdx;

						Path.push_back(apexPoint);

						if (portalIdx < amtPortals)
						{
							rightLeg = Portals[rightLegIndex].P1 - apexPoint;
							leftLeg = Portals[leftLegIndex].P2 - apexPoint;
							continue;
						}
					}
					else
					{
						// Tightening funnel on the left — update left leg
						leftLeg = newLeftLeg;
						leftLegIndex = portalIdx;
					}
				}
			}

			// Push the final destination point
			Path.push_back(Portals.back().P1);

			return Path;
		}

	private:
		SSFA() {};
		~SSFA() {};

		// 2D cross product (Z component of 3D cross product)
		// > 0: CCW,  < 0: CW,  == 0: parallel
		static float Cross2D(FVector2D const& A, FVector2D const& B)
		{
			return A.X * B.Y - A.Y * B.X;
		}
	};
}