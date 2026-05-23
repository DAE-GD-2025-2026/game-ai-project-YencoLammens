#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_Patrol.generated.h"

class PathFollow;
class ASteeringAgent;
class ABTGuardController;

UCLASS()
class GAMEAIPROG_API UBTTask_Patrol : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_Patrol();

protected:
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual EBTNodeResult::Type AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;

private:
	void NavigateToCurrentWaypoint(ASteeringAgent* Agent, ABTGuardController* Controller);

	TUniquePtr<PathFollow> pPathFollow;
	int32 CurrentWaypointIdx{0};
	float OriginalMaxSpeed{0.f};

	static constexpr float ArrivalRadius{60.f};
};
