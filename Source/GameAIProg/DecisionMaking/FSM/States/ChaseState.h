#pragma once

#include "GameAIProg/DecisionMaking/FSM/FSM.h"
#include <memory>

class Seek;

namespace GameAI::FSM
{
	class ChaseState final : public State
	{
	public:
		ChaseState();
		~ChaseState() override;

		void OnEnter(AAIController* Controller, Blackboard& BB) override;
		void OnExit(AAIController* Controller, Blackboard& BB) override;
		void Update(float DeltaTime, AAIController* Controller, Blackboard& BB) override;

	private:
		std::unique_ptr<Seek> pSeek;
	};
}