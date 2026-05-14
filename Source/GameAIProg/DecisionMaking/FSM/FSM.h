#pragma once
#include "CoreMinimal.h"
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

class AAIController;
class ASteeringAgent;

namespace GameAI::FSM
{
	using BlackboardField = std::variant<float, FVector2D, std::vector<FVector2D>, ASteeringAgent*>;

	class Blackboard final
	{
	public:
		template<typename T>
		void Set(const std::string& Key, T Value) { Data[Key] = std::move(Value); }

		template<typename T>
		T Get(const std::string& Key) const { return std::get<T>(Data.at(Key)); }

		bool Has(const std::string& Key) const { return Data.count(Key) > 0; }

	private:
		std::unordered_map<std::string, BlackboardField> Data;
	};

	class State
	{
	public:
		virtual ~State() = default;
		virtual void OnEnter(AAIController* Controller, Blackboard& BB) {}
		virtual void OnExit(AAIController* Controller, Blackboard& BB) {}
		virtual void Update(float DeltaTime, AAIController* Controller, Blackboard& BB) {}
	};

	class FSM final
	{
	public:
		void SetController(AAIController* InController);
		Blackboard& GetBlackboard();
		
		State* AddState(std::unique_ptr<State>&& NewState);
		void AddTransition(State* From, State* To, std::function<bool()> Condition);
		void SetInitialState(State* InitialState);
		void Update(float DeltaTime);

	private:
		struct Transition
		{
			State* From{nullptr};
			State* To{nullptr};
			std::function<bool()> Condition;
		};

		std::vector<std::unique_ptr<State>> States;
		std::vector<Transition> Transitions;
		State* CurrentState{nullptr};
		Blackboard BB;
		AAIController* Controller{nullptr};
	};
}