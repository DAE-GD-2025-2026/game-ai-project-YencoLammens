#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"

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
			FVector(RandomX, RandomY, 0.f),
			FRotator::ZeroRotator,
			SpawnParams
		);
	}

	pSeekBehavior         = std::make_unique<Seek>();
	pWanderBehavior       = std::make_unique<Wander>();
	pCohesionBehavior     = std::make_unique<Cohesion>(this);
	pSeparationBehavior   = std::make_unique<Separation>(this);
	pVelMatchBehavior     = std::make_unique<VelocityMatch>(this);

	std::vector<BlendedSteering::WeightedBehavior> BlendedBehaviors;
	BlendedBehaviors.emplace_back(pSeekBehavior.get(),       0.0f);
	BlendedBehaviors.emplace_back(pWanderBehavior.get(),     0.2f);
	BlendedBehaviors.emplace_back(pCohesionBehavior.get(),   0.2f);
	BlendedBehaviors.emplace_back(pSeparationBehavior.get(), 0.2f);
	BlendedBehaviors.emplace_back(pVelMatchBehavior.get(),   0.2f);
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

		pAgent->Tick(DeltaTime);

		if (bTrimWorld)
		{
			FVector2D Pos = pAgent->GetPosition();
			float Half = WorldSize * 0.5f;
			Pos.X = FMath::Clamp(Pos.X, -Half, Half);
			Pos.Y = FMath::Clamp(Pos.Y, -Half, Half);
			pAgent->SetActorLocation(FVector(Pos, 0.f));
		}
	}
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
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

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
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