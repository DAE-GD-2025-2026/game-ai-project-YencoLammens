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
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		
		//For each node received, get it's corresponding line
		FVector2D startPoint = Path.front()->GetPosition();
        Portals.push_back(NavLine{startPoint, startPoint});
        
		for (int i = 1; i < static_cast<int>(Path.size()) - 1; ++i)
		{
			NavGraphNode* pNavNode = dynamic_cast<NavGraphNode*>(Path[i]);
			if (!pNavNode || pNavNode->GetEdgeIdx() < 0)
				continue;

			TriPolygon::Edge const& Edge = NavPoly.GetEdges()[pNavNode->GetEdgeIdx()];
			FVector2D P1{Edge.GetP1(NavPoly)};
			FVector2D P2{Edge.GetP2(NavPoly)};

			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
			FVector2D travelDir = Path[i + 1]->GetPosition() - Path[i - 1]->GetPosition();
			if (FVector2D::CrossProduct(travelDir, P2 - P1) <= 0)
			{
				std::swap(P1, P2);
			}

			//Store portal
			Portals.push_back(NavLine{P1, P2});
		}
		
		//Add degenerate portal to force end evaluation
		FVector2D endPos = Path.back()->GetPosition();
		Portals.push_back(NavLine{endPos, endPos});
		
		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};
		//P1 == right point of portal, P2 == left point of portal
		if (Portals.empty()) return Path;

		FVector2D apex = Portals[0].P1;
		int apexIndex = 0;
		FVector2D rightLeg{0.f, 0.f};
		FVector2D leftLeg{0.f, 0.f};
		int rightLegIndex = 0;
		int leftLegIndex = 0;
		int amtPortals = static_cast<int>(Portals.size());

		Path.push_back(apex);

		int portalIdx = 1;
		while (portalIdx < amtPortals)
		{
			NavLine const portal = Portals[portalIdx];

			//--- RIGHT CHECK ---
			//1. See if moving funnel inwards - RIGHT
			FVector2D newRightLeg = portal.P1 - apex;
			if (FVector2D::CrossProduct(rightLeg, newRightLeg) >= 0)
			{
				//2. See if new line degenerates a line segment - RIGHT
				if (FVector2D::CrossProduct(leftLeg, newRightLeg) > 0)
				{
					//Leftleg becomes new apex point
					apex = apex + leftLeg;
					apexIndex = leftLegIndex;
					portalIdx = leftLegIndex + 1;
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;
					Path.push_back(apex);

					//Calculate new legs (if not the end)
					if (portalIdx < amtPortals)
					{
						rightLeg = Portals[rightLegIndex].P1 - apex;
						leftLeg = Portals[leftLegIndex].P2 - apex;
						continue;
					}
					break;
				}
				else
				{
					rightLeg = newRightLeg;
					rightLegIndex = portalIdx;
				}
			}

			//--- LEFT CHECK ---
			//1. See if moving funnel inwards - LEFT
			FVector2D newLeftLeg = portal.P2 - apex;
			if (FVector2D::CrossProduct(leftLeg, newLeftLeg) <= 0)
			{
				//2. See if new line degenerates a line segment - LEFT
				if (FVector2D::CrossProduct(rightLeg, newLeftLeg) < 0)
				{
					//Rightleg becomes new apex point
					apex = apex + rightLeg;
					apexIndex = rightLegIndex;
					portalIdx = rightLegIndex + 1;
					leftLegIndex = portalIdx;
					rightLegIndex = portalIdx;
					Path.push_back(apex);

					//Calculate new legs (if not the end)
					if (portalIdx < amtPortals)
					{
						rightLeg = Portals[rightLegIndex].P1 - apex;
						leftLeg = Portals[leftLegIndex].P2 - apex;
						continue;
					}
					break;
				}
				else
				{
					leftLeg = newLeftLeg;
					leftLegIndex = portalIdx;
				}
			}

			++portalIdx;
		}

		// Add last path point
		Path.push_back(Portals.back().P1);

		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}
