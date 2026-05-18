#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include <memory>
#include <vector>
#include "ThiefController.generated.h"

class PathFollow;
class Evade;
class ASteeringAgent;
namespace GameAI { class NavGraph; }

UCLASS()
class GAMEAIPROG_API AThiefController : public AAIController
{
	GENERATED_BODY()

public:
	AThiefController();
	virtual void OnPossess(APawn* InPawn) override;
	virtual void Tick(float DeltaTime) override;

	void SetNavGraph(GameAI::NavGraph* InNavGraph) { NavigationGraph = InNavGraph; }
	void SetGuardAgent(ASteeringAgent* InGuard) { GuardAgent = InGuard; }

private:
	void PickNewWanderTarget();
	FVector2D GetValidNavmeshPoint(FVector2D Candidate) const;
	bool IsGuardNearby() const;

	TUniquePtr<PathFollow> pPathFollow;
	TUniquePtr<Evade> pEvade;
	GameAI::NavGraph* NavigationGraph{nullptr};
	ASteeringAgent* GuardAgent{nullptr};
	FVector2D WanderTarget{};
	float OriginalMaxSpeed{0.f};
	bool bEvading{false};
	bool bHasActivePath{false};
	float WanderRetryTimer{0.f};

	static constexpr float WanderRadius{600.f};
	static constexpr float ArrivalRadius{80.f};
	static constexpr float EvadeRadius{350.f};
	static constexpr float WanderRetryDelay{2.f};
};