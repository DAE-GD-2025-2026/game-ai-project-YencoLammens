#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_Search.generated.h"

class PathFollow;
class ASteeringAgent;
class ABTGuardController;

UCLASS()
class GAMEAIPROG_API UBTTask_Search : public UBTTaskNode
{
	GENERATED_BODY()

public:
	UBTTask_Search();

protected:
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual EBTNodeResult::Type AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;

private:
	void NavigateToPosition(ASteeringAgent* Agent, ABTGuardController* Controller, FVector2D Target);
	FVector2D GetRandomNearbyPoint(FVector2D Center, float Radius) const;

	TUniquePtr<PathFollow> pPathFollow;
	FVector2D LastKnownPos{};
	FVector2D CurrentTarget{};
	float SearchElapsed{0.f};
	float OriginalMaxSpeed{0.f};

	static constexpr float ArrivalRadius{80.f};
	static constexpr float WanderRadius{400.f};
};