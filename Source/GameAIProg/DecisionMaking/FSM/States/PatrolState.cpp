#include "PatrolState.h"
#include "AIController.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"

static constexpr float PatrolWaypointLoopRadius = 60.f;

namespace GameAI::FSM
{
	PatrolState::PatrolState(GameAI::NavGraph* InNavGraph)
		: pPathFollow(std::make_unique<PathFollow>())
		, pNavGraph(InNavGraph)
	{}

	PatrolState::~PatrolState() = default;

	void PatrolState::OnEnter(AAIController* Controller, Blackboard& BB)
	{
		ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
		if (!Agent) return;

		OriginalMaxSpeed = Agent->GetMaxLinearSpeed();
		CurrentWaypointIdx = 0;
		PatrolWaypoints = BB.Get<std::vector<FVector2D>>("PatrolPath");

		Agent->SetSteeringBehavior(pPathFollow.get());
		NavigateToNextWaypoint(Agent);
	}

	void PatrolState::OnExit(AAIController* Controller, Blackboard& BB)
	{
		if (ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn()))
			Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
	}

	void PatrolState::Update(float DeltaTime, AAIController* Controller, Blackboard& BB)
	{
		if (PatrolWaypoints.empty()) return;

		ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
		if (!Agent) return;

		float DistSq = FVector2D::DistSquared(Agent->GetPosition(), PatrolWaypoints[CurrentWaypointIdx]);
		if (DistSq < PatrolWaypointLoopRadius * PatrolWaypointLoopRadius)
		{
			CurrentWaypointIdx = (CurrentWaypointIdx + 1) % static_cast<int>(PatrolWaypoints.size());
			NavigateToNextWaypoint(Agent);
		}
	}

	void PatrolState::NavigateToNextWaypoint(ASteeringAgent* Agent)
	{
		if (!pNavGraph || PatrolWaypoints.empty()) return;

		Agent->SetMaxLinearSpeed(OriginalMaxSpeed);

		GameAI::NavMeshPathfinding Pathfinder{};
		std::vector<FVector2D> Path = Pathfinder.FindPath(Agent->GetPosition(), PatrolWaypoints[CurrentWaypointIdx], pNavGraph);

		if (!Path.empty())
			pPathFollow->SetPath(Path);
	}
}