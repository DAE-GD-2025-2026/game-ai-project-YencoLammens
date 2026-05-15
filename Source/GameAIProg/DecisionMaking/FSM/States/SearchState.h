#pragma once

#include "GameAIProg/DecisionMaking/FSM/FSM.h"
#include <memory>

class PathFollow;
namespace GameAI { class NavGraph; }

namespace GameAI::FSM
{
	class SearchState final : public State
	{
	public:
		SearchState(GameAI::NavGraph* InNavGraph);
		~SearchState() override;

		void OnEnter(AAIController* Controller, Blackboard& BB) override;
		void OnExit(AAIController* Controller, Blackboard& BB) override;
		void Update(float DeltaTime, AAIController* Controller, Blackboard& BB) override;

	private:
		void NavigateToPosition(ASteeringAgent* Agent, FVector2D Target);
		FVector2D GetRandomNearbyPoint(FVector2D Center, float Radius) const;

		std::unique_ptr<PathFollow> pPathFollow;
		GameAI::NavGraph* pNavGraph{nullptr};
		FVector2D LastKnownPos{};
		FVector2D CurrentSearchTarget{};
		float OriginalMaxSpeed{0.f};
		bool bArrivedAtLastKnown{false};
	};
}