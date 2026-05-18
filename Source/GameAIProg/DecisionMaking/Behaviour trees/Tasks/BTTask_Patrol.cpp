#include "BTTask_Patrol.h"
#include "DecisionMaking/Behaviour trees/BTGuardController.h"
#include "BehaviorTree/BehaviorTreeComponent.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/NavGraph/NavGraph.h"

UBTTask_Patrol::UBTTask_Patrol()
{
    NodeName = "Patrol";
    bNotifyTick = true;
}

EBTNodeResult::Type UBTTask_Patrol::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
    ABTGuardController* Controller = Cast<ABTGuardController>(OwnerComp.GetAIOwner());
    ASteeringAgent* Agent = Controller ? Cast<ASteeringAgent>(Controller->GetPawn()) : nullptr;
    if (!Agent || Controller->GetPatrolPath().IsEmpty() || !Controller->GetNavGraph()) return EBTNodeResult::Failed;

    if (!pPathFollow)
        pPathFollow = MakeUnique<PathFollow>();

    OriginalMaxSpeed = Agent->GetMaxLinearSpeed();
    CurrentWaypointIdx = 0;
    Agent->SetSteeringBehavior(pPathFollow.Get());
    NavigateToCurrentWaypoint(Agent, Controller);

    return EBTNodeResult::InProgress;
}

void UBTTask_Patrol::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
    ABTGuardController* Controller = Cast<ABTGuardController>(OwnerComp.GetAIOwner());
    ASteeringAgent* Agent = Controller ? Cast<ASteeringAgent>(Controller->GetPawn()) : nullptr;
    if (!Agent) return;

    const TArray<FVector2D>& Path = Controller->GetPatrolPath();
    if (Path.IsEmpty()) return;

    float DistSq = FVector2D::DistSquared(Agent->GetPosition(), Path[CurrentWaypointIdx]);
    if (DistSq < ArrivalRadius * ArrivalRadius)
    {
        CurrentWaypointIdx = (CurrentWaypointIdx + 1) % Path.Num();
        NavigateToCurrentWaypoint(Agent, Controller);
    }
}

EBTNodeResult::Type UBTTask_Patrol::AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
    if (ASteeringAgent* Agent = Cast<ASteeringAgent>(OwnerComp.GetAIOwner()->GetPawn()))
        Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
    return EBTNodeResult::Aborted;
}

void UBTTask_Patrol::NavigateToCurrentWaypoint(ASteeringAgent* Agent, ABTGuardController* Controller)
{
    Agent->SetMaxLinearSpeed(OriginalMaxSpeed);

    FVector2D StartPos = Agent->GetPosition();
    FVector2D ValidStart = StartPos;

    TriPolygon const* NavPoly = Controller->GetNavGraph()->GetNavPolygon();
    if (NavPoly && !NavPoly->GetTriangleAtPosition(StartPos, true))
        NavPoly->GetClosestTriangleToPosition(StartPos, ValidStart);

    GameAI::NavMeshPathfinding Pathfinder{};
    std::vector<FVector2D> NavPath = Pathfinder.FindPath(
        ValidStart,
        Controller->GetPatrolPath()[CurrentWaypointIdx],
        Controller->GetNavGraph());

    if (!NavPath.empty())
    {
        pPathFollow->SetPath(NavPath);
    }
    else
    {
        std::vector<FVector2D> Fallback{Controller->GetPatrolPath()[CurrentWaypointIdx]};
        pPathFollow->SetPath(Fallback);
    }
}