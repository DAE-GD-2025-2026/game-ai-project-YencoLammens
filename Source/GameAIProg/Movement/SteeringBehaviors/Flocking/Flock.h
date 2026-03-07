#pragma once

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"
#include "../SpacePartitioning/SpacePartitioning.h"

class Flock final
{
public:
	Flock(
		UWorld* pWorld,
		TSubclassOf<ASteeringAgent> AgentClass,
		int FlockSize = 10,
		float WorldSize = 100.f,
		ASteeringAgent* const pAgentToEvade = nullptr,
		bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

	void RegisterNeighbors(ASteeringAgent* const Agent);
	int GetNrOfNeighbors() const;
	const TArray<ASteeringAgent*>& GetNeighbors() const;

	FVector2D GetAverageNeighborPos()      const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const& Target);

private:
	void RenderNeighborhood();

	UWorld* pWorld{nullptr};

	int FlockSize{0};
	TArray<ASteeringAgent*> Agents{};

	// Space partitioning
	std::unique_ptr<CellSpace> pPartitionedSpace{};
	int NrOfCellsX{10};
	TArray<FVector2D> OldPositions{};
	bool bUseSpacePartitioning{true};

	// Non-partitioned neighbors
	TArray<ASteeringAgent*> Neighbors{};
	int NrOfNeighbors{0};

	float NeighborhoodRadius{150.f};

	ASteeringAgent* pAgentToEvade{nullptr};

	float WorldSize{0.f};
	bool  bTrimWorld{false};
	TSubclassOf<ASteeringAgent> AgentClass{};

	std::unique_ptr<Separation>       pSeparationBehavior{};
	std::unique_ptr<Cohesion>         pCohesionBehavior{};
	std::unique_ptr<VelocityMatch>    pVelMatchBehavior{};
	std::unique_ptr<Seek>             pSeekBehavior{};
	std::unique_ptr<Wander>           pWanderBehavior{};
	std::unique_ptr<Evade>            pEvadeBehavior{};
	std::unique_ptr<BlendedSteering>  pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	float EvadeRadius{300.f};

	bool bDebugRenderSteering{false};
	bool bDebugRenderNeighborhood{false};
	bool bDebugRenderPartitions{true};
};