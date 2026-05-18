#include "BTService_UpdateTarget.h"
#include "DecisionMaking/Behaviour trees/BBKeys.h"
#include "BehaviorTree/BehaviorTreeComponent.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

UBTService_UpdateTarget::UBTService_UpdateTarget()
{
	NodeName = "Update Target";
	Interval = 0.1f;
}

void UBTService_UpdateTarget::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
	if (!BB) return;

	ASteeringAgent* Target = Cast<ASteeringAgent>(BB->GetValueAsObject(BBKeys::TargetActor));
	if (!Target) return;

	BB->SetValueAsVector(BBKeys::LastKnownLocation, FVector(Target->GetPosition(), 0.f));
}