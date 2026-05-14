// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <memory>
#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Level_FSM.generated.h"


class AGameAIController;
class Seek;

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_FSM();
	
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	ASteeringAgent* Agent{nullptr};

	UPROPERTY()
	ASteeringAgent* ThiefAgent{nullptr};

	UPROPERTY()
	AGameAIController* GuardController{nullptr};
	
	std::unique_ptr<Seek> ThiefSeek;
	std::unique_ptr<GameAI::NavGraph> NavigationGraph;

	UPROPERTY(EditAnywhere, Category="FSM|Guard")
	float DetectionRadius{400.f};

	UPROPERTY(EditAnywhere, Category="FSM|Guard")
	float SearchDuration{5.f};

	void SetupThief();
	void SetupGuard();
	TArray<TArray<FVector>> ExtractNavMeshTris() const;
};