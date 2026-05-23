#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "Perception/AIPerceptionTypes.h"
#include "BTGuardController.generated.h"

class UAISenseConfig_Sight;
class UBehaviorTree;
namespace GameAI { class NavGraph; }

UCLASS()
class GAMEAIPROG_API ABTGuardController : public AAIController
{
	GENERATED_BODY()

public:
	ABTGuardController();
	virtual void OnPossess(APawn* InPawn) override;

	void SetNavGraph(GameAI::NavGraph* InNavGraph) { NavigationGraph = InNavGraph; }
	void SetPatrolPath(const TArray<FVector2D>& InPath) { PatrolPath = InPath; }
	void SetDetectionRadius(float Radius);

	GameAI::NavGraph* GetNavGraph() const { return NavigationGraph; }
	const TArray<FVector2D>& GetPatrolPath() const { return PatrolPath; }
	float GetAgentMaxSpeed() const { return AgentMaxSpeed; }

	UPROPERTY(EditDefaultsOnly, Category="BT")
	UBehaviorTree* GuardBehaviorTree{nullptr};

	UPROPERTY(EditDefaultsOnly, Category="BT|Search")
	float SearchDuration{5.f};

private:
	UFUNCTION()
	void OnTargetPerceptionUpdated(AActor* Actor, FAIStimulus Stimulus);

	UAISenseConfig_Sight* SightConfig{nullptr};
	GameAI::NavGraph* NavigationGraph{nullptr};
	TArray<FVector2D> PatrolPath;
	float AgentMaxSpeed{0.f};
};