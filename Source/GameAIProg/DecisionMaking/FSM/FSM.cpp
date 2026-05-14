#include "FSM.h"

namespace GameAI::FSM
{
	void FSM::SetController(AAIController* InController)
	{
		Controller = InController;
	}

	Blackboard& FSM::GetBlackboard()
	{
		return BB;
	}

	State* FSM::AddState(std::unique_ptr<State>&& NewState)
	{
		State* RawPtr = NewState.get();
		States.push_back(std::move(NewState));
		return RawPtr;
	}

	void FSM::AddTransition(State* From, State* To, std::function<bool()> Condition)
	{
		Transitions.push_back({From, To, std::move(Condition)});
	}

	void FSM::SetInitialState(State* InitialState)
	{
		CurrentState = InitialState;
		if (CurrentState && Controller)
			CurrentState->OnEnter(Controller, BB);
	}

	void FSM::Update(float DeltaTime)
	{
		if (!CurrentState || !Controller) return;

		for (Transition& T : Transitions)
		{
			if (T.From == CurrentState && T.Condition())
			{
				CurrentState->OnExit(Controller, BB);
				CurrentState = T.To;
				CurrentState->OnEnter(Controller, BB);
				break;
			}
		}

		CurrentState->Update(DeltaTime, Controller, BB);
	}
}
