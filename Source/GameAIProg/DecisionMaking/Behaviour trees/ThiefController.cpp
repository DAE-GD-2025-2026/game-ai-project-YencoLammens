#include "ThiefController.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "GraphTheory/Algorithms/NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Math/UnrealMathUtility.h"

AThiefController::AThiefController()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AThiefController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    ASteeringAgent* Agent = Cast<ASteeringAgent>(InPawn);
    if (!Agent) return;

    pPathFollow = MakeUnique<PathFollow>();
    pEvade = MakeUnique<Evade>();
    OriginalMaxSpeed = Agent->GetMaxLinearSpeed();
    Agent->SetSteeringBehavior(pPathFollow.Get());

    bHasActivePath = false;
    WanderRetryTimer = 0.f;
    WanderTarget = Agent->GetPosition();
}

void AThiefController::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ASteeringAgent* Agent = Cast<ASteeringAgent>(GetPawn());
    if (!Agent || !NavigationGraph) return;

    if (IsGuardNearby())
    {
        if (!bEvading)
        {
            bEvading = true;
            bHasActivePath = false;
            Agent->SetSteeringBehavior(pEvade.Get());
        }

        if (GuardAgent)
        {
            FTargetData TargetData;
            TargetData.Position = GuardAgent->GetPosition();
            TargetData.LinearVelocity = GuardAgent->GetLinearVelocity();
            pEvade->SetTarget(TargetData);
        }
        return;
    }

    if (bEvading)
    {
        bEvading = false;
        Agent->SetSteeringBehavior(pPathFollow.Get());
        WanderRetryTimer = 0.f;
        bHasActivePath = false;
    }

    WanderRetryTimer -= DeltaTime;

    if (bHasActivePath)
    {
        if (FVector2D::DistSquared(Agent->GetPosition(), WanderTarget) < ArrivalRadius * ArrivalRadius)
        {
            bHasActivePath = false;
            WanderRetryTimer = 0.f;
        }
    }
    else if (WanderRetryTimer <= 0.f)
    {
        PickNewWanderTarget();
    }
}

void AThiefController::PickNewWanderTarget()
{
    ASteeringAgent* Agent = Cast<ASteeringAgent>(GetPawn());
    if (!Agent) return;

    float Angle = FMath::RandRange(0.f, 2.f * PI);
    float Dist = FMath::RandRange(WanderRadius * 0.3f, WanderRadius);
    FVector2D Candidate = Agent->GetPosition() + FVector2D(FMath::Cos(Angle) * Dist, FMath::Sin(Angle) * Dist);
    FVector2D ValidCandidate = GetValidNavmeshPoint(Candidate);

    FVector2D StartPos = Agent->GetPosition();
    FVector2D ValidStart = StartPos;
    if (TriPolygon const* NavPoly = NavigationGraph->GetNavPolygon())
    {
        if (!NavPoly->GetTriangleAtPosition(StartPos, true))
            NavPoly->GetClosestTriangleToPosition(StartPos, ValidStart);
    }

    Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
    GameAI::NavMeshPathfinding Pathfinder{};
    std::vector<FVector2D> Path = Pathfinder.FindPath(ValidStart, ValidCandidate, NavigationGraph);

    if (!Path.empty())
    {
        pPathFollow->SetPath(Path);
        WanderTarget = ValidCandidate;
        bHasActivePath = true;
        WanderRetryTimer = 0.f;
    }
    else
    {
        bHasActivePath = false;
        WanderRetryTimer = WanderRetryDelay;
    }
}

FVector2D AThiefController::GetValidNavmeshPoint(FVector2D Candidate) const
{
    if (!NavigationGraph) return Candidate;

    TriPolygon const* NavPoly = NavigationGraph->GetNavPolygon();
    if (!NavPoly) return Candidate;

    if (NavPoly->GetTriangleAtPosition(Candidate, true))
        return Candidate;

    FVector2D OutPos = Candidate;
    NavPoly->GetClosestTriangleToPosition(Candidate, OutPos);
    return OutPos;
}

bool AThiefController::IsGuardNearby() const
{
    ASteeringAgent* Agent = Cast<ASteeringAgent>(GetPawn());
    if (!Agent || !GuardAgent) return false;
    return FVector2D::Distance(Agent->GetPosition(), GuardAgent->GetPosition()) < EvadeRadius;
}