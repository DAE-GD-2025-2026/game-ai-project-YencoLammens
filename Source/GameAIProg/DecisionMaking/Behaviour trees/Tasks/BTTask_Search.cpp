#include "BTTask_Search.h"
#include "DecisionMaking/Behaviour trees/BTGuardController.h"
#include "DecisionMaking/Behaviour trees/BBKeys.h"
#include "BehaviorTree/BehaviorTreeComponent.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "Math/UnrealMathUtility.h"

UBTTask_Search::UBTTask_Search()
{
    NodeName = "Search";
    bNotifyTick = true;
}

EBTNodeResult::Type UBTTask_Search::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
    ABTGuardController* Controller = Cast<ABTGuardController>(OwnerComp.GetAIOwner());
    ASteeringAgent* Agent = Controller ? Cast<ASteeringAgent>(Controller->GetPawn()) : nullptr;
    if (!Agent) return EBTNodeResult::Failed;

    if (!OwnerComp.GetBlackboardComponent()->GetValueAsBool(BBKeys::bShouldSearch))
        return EBTNodeResult::Failed;

    if (!pPathFollow)
        pPathFollow = MakeUnique<PathFollow>();

    OriginalMaxSpeed = Agent->GetMaxLinearSpeed();
    SearchElapsed = 0.f;

    FVector LastKnown = OwnerComp.GetBlackboardComponent()->GetValueAsVector(BBKeys::LastKnownLocation);
    LastKnownPos = FVector2D(LastKnown.X, LastKnown.Y);
    CurrentTarget = LastKnownPos;

    Agent->SetSteeringBehavior(pPathFollow.Get());
    NavigateToPosition(Agent, Controller, LastKnownPos);

    return EBTNodeResult::InProgress;
}

void UBTTask_Search::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
    ABTGuardController* Controller = Cast<ABTGuardController>(OwnerComp.GetAIOwner());
    ASteeringAgent* Agent = Controller ? Cast<ASteeringAgent>(Controller->GetPawn()) : nullptr;
    if (!Agent) return;

    UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();

    if (BB->GetValueAsObject(BBKeys::TargetActor))
    {
        FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
        return;
    }

    SearchElapsed += DeltaSeconds;
    if (SearchElapsed >= Controller->SearchDuration)
    {
        OwnerComp.GetBlackboardComponent()->SetValueAsBool(BBKeys::bShouldSearch, false);
        FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
        return;
    }

    float DistSq = FVector2D::DistSquared(Agent->GetPosition(), CurrentTarget);
    if (DistSq < ArrivalRadius * ArrivalRadius)
    {
        CurrentTarget = GetRandomNearbyPoint(LastKnownPos, WanderRadius);
        NavigateToPosition(Agent, Controller, CurrentTarget);
    }
}

EBTNodeResult::Type UBTTask_Search::AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
    if (ASteeringAgent* Agent = Cast<ASteeringAgent>(OwnerComp.GetAIOwner()->GetPawn()))
        Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
    return EBTNodeResult::Aborted;
}

void UBTTask_Search::NavigateToPosition(ASteeringAgent* Agent, ABTGuardController* Controller, FVector2D Target)
{
    Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
    GameAI::NavMeshPathfinding Pathfinder{};
    std::vector<FVector2D> Path = Pathfinder.FindPath(Agent->GetPosition(), Target, Controller->GetNavGraph());
    if (!Path.empty())
        pPathFollow->SetPath(Path);
}

FVector2D UBTTask_Search::GetRandomNearbyPoint(FVector2D Center, float Radius) const
{
    float Angle = FMath::RandRange(0.f, 2.f * PI);
    float Dist = FMath::RandRange(Radius * 0.3f, Radius);
    return Center + FVector2D(FMath::Cos(Angle) * Dist, FMath::Sin(Angle) * Dist);
}