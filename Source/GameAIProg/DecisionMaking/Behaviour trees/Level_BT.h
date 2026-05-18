#pragma once

#include "CoreMinimal.h"
#include <memory>
#include "Shared/Level_Base.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Level_BT.generated.h"

class ABTGuardController;
class AThiefController;
class UBehaviorTree;

UCLASS()
class GAMEAIPROG_API ALevel_BT : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_BT();
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, Category="BT")
	UBehaviorTree* GuardBehaviorTree{nullptr};

protected:
	virtual void BeginPlay() override;

private:
	void SetupNavGraph();
	void SetupGuard();
	void SetupThief();
	TArray<TArray<FVector>> ExtractNavMeshTris() const;

	UPROPERTY()
	ASteeringAgent* GuardAgent{nullptr};

	UPROPERTY()
	ASteeringAgent* ThiefAgent{nullptr};

	UPROPERTY()
	ABTGuardController* GuardController{nullptr};

	UPROPERTY()
	AThiefController* ThiefController{nullptr};

	std::unique_ptr<GameAI::NavGraph> NavigationGraph;

	UPROPERTY(EditAnywhere, Category="BT|Guard")
	float DetectionRadius{400.f};
};