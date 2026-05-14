#pragma once

#include "GameAIProg/DecisionMaking/FSM/FSM.h"
#include <memory>
#include <vector>

class PathFollow;

namespace GameAI { class NavGraph; }

namespace GameAI::FSM
{
	class PatrolState final : public State
	{
	public:
		PatrolState(GameAI::NavGraph* InNavGraph);
		~PatrolState() override;

		void OnEnter(AAIController* Controller, Blackboard& BB) override;
		void OnExit(AAIController* Controller, Blackboard& BB) override;
		void Update(float DeltaTime, AAIController* Controller, Blackboard& BB) override;

	private:
		void NavigateToNextWaypoint(ASteeringAgent* Agent);

		std::unique_ptr<PathFollow> pPathFollow;
		std::vector<FVector2D> PatrolWaypoints;
		int CurrentWaypointIdx{0};
		float OriginalMaxSpeed{0.f};
		GameAI::NavGraph* pNavGraph{nullptr};
	};
}