#include "BTGuardController.h"
#include "BehaviorTree/BehaviorTree.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISenseConfig_Sight.h"
#include "BBKeys.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

ABTGuardController::ABTGuardController()
{
    UAIPerceptionComponent* Perception = CreateDefaultSubobject<UAIPerceptionComponent>(TEXT("AIPerception"));
    SetPerceptionComponent(*Perception);

    SightConfig = CreateDefaultSubobject<UAISenseConfig_Sight>(TEXT("SightConfig"));
    SightConfig->SightRadius = 400.f;
    SightConfig->LoseSightRadius = 500.f;
    SightConfig->PeripheralVisionAngleDegrees = 180.f;
    SightConfig->SetMaxAge(5.f);
    SightConfig->DetectionByAffiliation.bDetectEnemies = true;
    SightConfig->DetectionByAffiliation.bDetectNeutrals = true;
    SightConfig->DetectionByAffiliation.bDetectFriendlies = true;

    Perception->ConfigureSense(*SightConfig);
    Perception->SetDominantSense(SightConfig->GetSenseImplementation());
    Perception->OnTargetPerceptionUpdated.AddDynamic(this, &ABTGuardController::OnTargetPerceptionUpdated);
}

void ABTGuardController::OnPossess(APawn* InPawn)
{
    Super::OnPossess(InPawn);
    if (ASteeringAgent* Agent = Cast<ASteeringAgent>(InPawn))
        AgentMaxSpeed = Agent->GetMaxLinearSpeed();
    if (GuardBehaviorTree)
        RunBehaviorTree(GuardBehaviorTree);
}

void ABTGuardController::OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus)
{
    UBlackboardComponent* BB = GetBlackboardComponent();
    if (!BB) return;

    if (Stimulus.WasSuccessfullySensed())
    {
        BB->SetValueAsObject(BBKeys::TargetActor, Actor);
        BB->SetValueAsBool(BBKeys::bShouldSearch, false);
    }
    else
    {
        BB->SetValueAsObject(BBKeys::TargetActor, nullptr);
        BB->SetValueAsVector(BBKeys::LastKnownLocation, FVector(Actor->GetActorLocation().X, Actor->GetActorLocation().Y, 0.f));
        BB->SetValueAsBool(BBKeys::bShouldSearch, true);
    }
}

void ABTGuardController::SetDetectionRadius(float Radius)
{
    if (!SightConfig) return;
    SightConfig->SightRadius = Radius;
    SightConfig->LoseSightRadius = Radius + 100.f;
    GetPerceptionComponent()->ConfigureSense(*SightConfig);
}