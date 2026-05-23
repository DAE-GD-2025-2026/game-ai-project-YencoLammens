#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_Chase.generated.h"

class Seek;
class ASteeringAgent;

UCLASS()
class GAMEAIPROG_API UBTTask_Chase : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_Chase();

protected:
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual EBTNodeResult::Type AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;

private:
	TUniquePtr<Seek> pSeek;
	float OriginalMaxSpeed{0.f};
};