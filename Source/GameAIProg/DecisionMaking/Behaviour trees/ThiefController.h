#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "ThiefController.generated.h"

class Wander;
class Evade;
class ASteeringAgent;

UCLASS()
class GAMEAIPROG_API AThiefController : public AAIController
{
	GENERATED_BODY()

public:
	AThiefController();
	virtual void OnPossess(APawn* InPawn) override;
	virtual void Tick(float DeltaTime) override;

	void SetGuardAgent(ASteeringAgent* InGuard) { GuardAgent = InGuard; }

private:
	bool IsGuardNearby() const;

	TUniquePtr<Wander> pWander;
	TUniquePtr<Evade> pEvade;
	ASteeringAgent* GuardAgent{nullptr};
	float OriginalMaxSpeed{0.f};
	bool bEvading{false};

	static constexpr float EvadeRadius{350.f};
};