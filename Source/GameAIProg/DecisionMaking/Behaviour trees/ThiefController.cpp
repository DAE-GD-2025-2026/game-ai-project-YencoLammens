#include "ThiefController.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

AThiefController::AThiefController()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AThiefController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);

    ASteeringAgent* Agent = Cast<ASteeringAgent>(InPawn);
    if (!Agent) return;

    pWander = MakeUnique<Wander>();
    pWander->SetWanderOffset(200.f);
    pWander->SetWanderRadius(150.f);
    pWander->SetMaxAngleChange(FMath::DegreesToRadians(45.f));

    pEvade = MakeUnique<Evade>();
    pEvade->SetEvadeRadius(EvadeRadius);

    OriginalMaxSpeed = Agent->GetMaxLinearSpeed();
    Agent->SetSteeringBehavior(pWander.Get());
}

void AThiefController::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ASteeringAgent* Agent = Cast<ASteeringAgent>(GetPawn());
    if (!Agent) return;

    if (IsGuardNearby())
    {
        if (!bEvading)
        {
            bEvading = true;
            Agent->SetMaxLinearSpeed(OriginalMaxSpeed * EvadeSpeedMultiplier);
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
        Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
        Agent->SetSteeringBehavior(pWander.Get());
    }
}

bool AThiefController::IsGuardNearby() const
{
    ASteeringAgent* Agent = Cast<ASteeringAgent>(GetPawn());
    if (!Agent || !GuardAgent) return false;
    return FVector2D::Distance(Agent->GetPosition(), GuardAgent->GetPosition()) < EvadeRadius;
}