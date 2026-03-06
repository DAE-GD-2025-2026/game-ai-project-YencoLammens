#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "DrawDebugHelpers.h"

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
	, WorldSize{ WorldSize }
	, bTrimWorld{ bTrimWorld }
	, AgentClass{ AgentClass }
{
	Agents.SetNum(FlockSize);
	Neighbors.SetNum(FlockSize);

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	for (int i = 0; i < FlockSize; ++i)
	{
		float RandomX = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);
		float RandomY = FMath::RandRange(-WorldSize * 0.5f, WorldSize * 0.5f);

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
	AgentClass,
	FVector(RandomX, RandomY, 90.f), 
	FRotator::ZeroRotator,
	SpawnParams
);
	}

	pSeekBehavior       = std::make_unique<Seek>();
	pWanderBehavior     = std::make_unique<Wander>();
	pCohesionBehavior   = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior   = std::make_unique<VelocityMatch>(this);

	std::vector<BlendedSteering::WeightedBehavior> BlendedBehaviors;
	BlendedBehaviors.emplace_back(pSeekBehavior.get(),       0.0f);
	BlendedBehaviors.emplace_back(pWanderBehavior.get(),     0.5f);
	BlendedBehaviors.emplace_back(pCohesionBehavior.get(),   0.3f);
	BlendedBehaviors.emplace_back(pSeparationBehavior.get(), 0.8f);
	BlendedBehaviors.emplace_back(pVelMatchBehavior.get(),   0.3f);
	pBlendedSteering = std::make_unique<BlendedSteering>(BlendedBehaviors);

	for (ASteeringAgent* pAgent : Agents)
		if (pAgent)
			pAgent->SetSteeringBehavior(pBlendedSteering.get());
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world

	for (int i = 0; i < FlockSize; ++i)
	{
		ASteeringAgent* pAgent = Agents[i];
		if (!pAgent) continue;

		RegisterNeighbors(pAgent);

		if (bTrimWorld)
		{
			FVector2D Pos = pAgent->GetPosition();
			float Half = WorldSize * 0.5f;
			bool bClamped = false;
			if (Pos.X < -Half || Pos.X > Half || Pos.Y < -Half || Pos.Y > Half)
			{
				Pos.X = FMath::Clamp(Pos.X, -Half, Half);
				Pos.Y = FMath::Clamp(Pos.Y, -Half, Half);
				pAgent->SetActorLocation(FVector(Pos, 90.f));
				pAgent->GetCharacterMovement()->StopMovementImmediately();
			}
		}
	}
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
	if (bDebugRenderNeighborhood && Agents.Num() > 0)
		RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here
		bool bPrevDebugSteering = bDebugRenderSteering;
		ImGui::Checkbox("Debug Steering", &bDebugRenderSteering);
		if (bDebugRenderSteering != bPrevDebugSteering)
			for (ASteeringAgent* pAgent : Agents)
				if (pAgent) pAgent->SetDebugRenderingEnabled(bDebugRenderSteering);

		ImGui::Checkbox("Debug Neighborhood", &bDebugRenderNeighborhood);

		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
		auto& Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

		float SeekW = Behaviors[0].Weight;
		if (ImGui::SliderFloat("Seek",         &SeekW,  0.f, 1.f, "%.2f")) Behaviors[0].Weight = SeekW;

		float WanderW = Behaviors[1].Weight;
		if (ImGui::SliderFloat("Wander",       &WanderW, 0.f, 1.f, "%.2f")) Behaviors[1].Weight = WanderW;

		float CohesionW = Behaviors[2].Weight;
		if (ImGui::SliderFloat("Cohesion",     &CohesionW, 0.f, 1.f, "%.2f")) Behaviors[2].Weight = CohesionW;

		float SeparationW = Behaviors[3].Weight;
		if (ImGui::SliderFloat("Separation",   &SeparationW, 0.f, 1.f, "%.2f")) Behaviors[3].Weight = SeparationW;

		float VelMatchW = Behaviors[4].Weight;
		if (ImGui::SliderFloat("VelocityMatch",&VelMatchW, 0.f, 1.f, "%.2f")) Behaviors[4].Weight = VelMatchW;

		ImGui::Spacing();
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Neighborhood Radius",
			NeighborhoodRadius, 50.f, 500.f,
			[this](float InVal) { NeighborhoodRadius = InVal; }, "%.0f");

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
	if (Agents.Num() == 0 || !Agents[0]) return;

	ASteeringAgent* pFirst = Agents[0];
	FVector2D AgentPos = pFirst->GetPosition();

	DrawDebugCircle(pFirst->GetWorld(),
		FVector(AgentPos, 0.f),
		NeighborhoodRadius,
		32, FColor::Yellow, false, -1.f, 0, 2.f,
		FVector(0, 1, 0), FVector(1, 0, 0));

	const TArray<ASteeringAgent*>& Nbrs = GetNeighbors();
	for (int i = 0; i < GetNrOfNeighbors(); ++i)
	{
		if (!Nbrs[i]) continue;
		DrawDebugLine(pFirst->GetWorld(),
			FVector(AgentPos, 0.f),
			FVector(Nbrs[i]->GetPosition(), 0.f),
			FColor::Yellow, false, -1.f, 0, 1.5f);

		DrawDebugCircle(pFirst->GetWorld(),
			FVector(Nbrs[i]->GetPosition(), 0.f),
			15.f, 12, FColor::Orange, false, -1.f, 0, 2.f,
			FVector(0, 1, 0), FVector(1, 0, 0));
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;
	FVector2D AgentPos = pAgent->GetPosition();

	for (ASteeringAgent* pOther : Agents)
	{
		if (!pOther || pOther == pAgent) continue;
		if (FVector2D::Distance(AgentPos, pOther->GetPosition()) <= NeighborhoodRadius)
			Neighbors[NrOfNeighbors++] = pOther;
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;
	int Count = GetNrOfNeighbors();
	if (Count == 0) return avgPosition;

	const TArray<ASteeringAgent*>& Nbrs = GetNeighbors();
	for (int i = 0; i < Count; ++i)
		if (Nbrs[i]) avgPosition += Nbrs[i]->GetPosition();

	return avgPosition / static_cast<float>(Count);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;
	int Count = GetNrOfNeighbors();
	if (Count == 0) return avgVelocity;

	const TArray<ASteeringAgent*>& Nbrs = GetNeighbors();
	for (int i = 0; i < Count; ++i)
		if (Nbrs[i]) avgVelocity += FVector2D(Nbrs[i]->GetVelocity());

	return avgVelocity / static_cast<float>(Count);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}