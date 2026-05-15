#include "SearchState.h"
#include "AIController.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "Math/UnrealMathUtility.h"

static constexpr float SearchArrivalRadius = 80.f;
static constexpr float SearchWanderRadius = 400.f;

namespace GameAI::FSM
{
	SearchState::SearchState(GameAI::NavGraph* InNavGraph)
		: pPathFollow(std::make_unique<PathFollow>())
		, pNavGraph(InNavGraph)
	{}

	SearchState::~SearchState() = default;

	void SearchState::OnEnter(AAIController* Controller, Blackboard& BB)
	{
		ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
		if (!Agent) return;

		OriginalMaxSpeed = Agent->GetMaxLinearSpeed();
		bArrivedAtLastKnown = false;

		BB.Set<float>("SearchElapsedTime", 0.f);

		LastKnownPos = BB.Has("LastKnownPos") ? BB.Get<FVector2D>("LastKnownPos") : Agent->GetPosition();
		CurrentSearchTarget = LastKnownPos;

		Agent->SetSteeringBehavior(pPathFollow.get());
		NavigateToPosition(Agent, LastKnownPos);
	}

	void SearchState::OnExit(AAIController* Controller, Blackboard& BB)
	{
		if (ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn()))
			Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
	}

	void SearchState::Update(float DeltaTime, AAIController* Controller, Blackboard& BB)
	{
		ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
		if (!Agent) return;

		float Elapsed = BB.Get<float>("SearchElapsedTime") + DeltaTime;
		BB.Set<float>("SearchElapsedTime", Elapsed);

		float DistSq = FVector2D::DistSquared(Agent->GetPosition(), CurrentSearchTarget);
		if (DistSq < SearchArrivalRadius * SearchArrivalRadius)
		{
			if (!bArrivedAtLastKnown)
			{
				bArrivedAtLastKnown = true;
			}

			FVector2D WanderTarget = GetRandomNearbyPoint(LastKnownPos, SearchWanderRadius);
			CurrentSearchTarget = WanderTarget;
			NavigateToPosition(Agent, WanderTarget);
		}
	}

	void SearchState::NavigateToPosition(ASteeringAgent* Agent, FVector2D Target)
	{
		if (!pNavGraph) return;

		Agent->SetMaxLinearSpeed(OriginalMaxSpeed);

		GameAI::NavMeshPathfinding Pathfinder{};
		std::vector<FVector2D> Path = Pathfinder.FindPath(Agent->GetPosition(), Target, pNavGraph);

		if (!Path.empty())
			pPathFollow->SetPath(Path);
	}

	FVector2D SearchState::GetRandomNearbyPoint(FVector2D Center, float Radius) const
	{
		float Angle = FMath::RandRange(0.f, 2.f * PI);
		float Dist = FMath::RandRange(Radius * 0.3f, Radius);
		return Center + FVector2D(FMath::Cos(Angle) * Dist, FMath::Sin(Angle) * Dist);
	}
}