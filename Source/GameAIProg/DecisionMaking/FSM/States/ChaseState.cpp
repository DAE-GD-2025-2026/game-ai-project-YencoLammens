#include "ChaseState.h"
#include "AIController.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

namespace GameAI::FSM
{
    ChaseState::ChaseState()
        : pSeek(std::make_unique<Seek>())
    {}

    ChaseState::~ChaseState() = default;

    void ChaseState::OnEnter(AAIController* Controller, Blackboard& BB)
    {
        ASteeringAgent* Agent = Cast<ASteeringAgent>(Controller->GetPawn());
        if (Agent)
            Agent->SetSteeringBehavior(pSeek.get());
    }

    void ChaseState::OnExit(AAIController* Controller, Blackboard& BB) {}

    void ChaseState::Update(float DeltaTime, AAIController* Controller, Blackboard& BB)
    {
        ASteeringAgent* Thief = BB.Get<ASteeringAgent*>("Thief");
        if (!Thief) return;
        
        FTargetData Target;
        Target.Position = Thief->GetPosition();
        pSeek->SetTarget(Target);

        BB.Set("LastKnownPos", Thief->GetPosition());
    }
}