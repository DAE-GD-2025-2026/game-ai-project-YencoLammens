#include "BTTask_Chase.h"
#include "AIController.h"
#include "DecisionMaking/Behaviour trees/BBKeys.h"
#include "DecisionMaking/Behaviour trees/BTGuardController.h"
#include "BehaviorTree/BehaviorTreeComponent.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

UBTTask_Chase::UBTTask_Chase()
{
	NodeName = "Chase";
	bNotifyTick = true;
}

EBTNodeResult::Type UBTTask_Chase::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	ABTGuardController* Controller = Cast<ABTGuardController>(OwnerComp.GetAIOwner());
	ASteeringAgent* Agent = Controller ? Cast<ASteeringAgent>(Controller->GetPawn()) : nullptr;
	if (!Agent) return EBTNodeResult::Failed;

	UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
	if (!BB || !BB->GetValueAsObject(BBKeys::TargetActor)) return EBTNodeResult::Failed;

	if (!pSeek)
		pSeek = MakeUnique<Seek>();

	OriginalMaxSpeed = Controller->GetAgentMaxSpeed();
	Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
	Agent->SetSteeringBehavior(pSeek.Get());

	return EBTNodeResult::InProgress;
}

void UBTTask_Chase::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	UBlackboardComponent* BB = OwnerComp.GetBlackboardComponent();
	ASteeringAgent* Target = Cast<ASteeringAgent>(BB->GetValueAsObject(BBKeys::TargetActor));
	if (!Target)
	{
		if (ASteeringAgent* Agent = Cast<ASteeringAgent>(OwnerComp.GetAIOwner()->GetPawn()))
			Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	FTargetData TargetData;
	TargetData.Position = Target->GetPosition();
	pSeek->SetTarget(TargetData);

	BB->SetValueAsVector(BBKeys::LastKnownLocation, FVector(Target->GetPosition(), 0.f));
}

EBTNodeResult::Type UBTTask_Chase::AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	if (ASteeringAgent* Agent = Cast<ASteeringAgent>(OwnerComp.GetAIOwner()->GetPawn()))
		Agent->SetMaxLinearSpeed(OriginalMaxSpeed);
	return EBTNodeResult::Aborted;
}