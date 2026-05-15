// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_FSM.h"

#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "NavMesh/RecastNavMesh.h"
#include "Runtime/Navmesh/Public/Detour/DetourNavMesh.h"
#include "Movement/Pathfinding/Navmesh//TriPolygon.h"

#include "FSMComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "States/PatrolState.h"
#include "States/ChaseState.h"
#include "States/SearchState.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"


ALevel_FSM::ALevel_FSM()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	auto NavPoly{std::make_unique<TriPolygon>()};
	for (TArray<FVector> const& Tri : ExtractNavMeshTris())
		NavPoly->AddTriangle(Tri);
	NavigationGraph = std::make_unique<GameAI::NavGraph>(std::move(NavPoly));

	SetupThief();
	SetupGuard();
}

void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (ThiefSeek)
	{
		FTargetData Target;
		Target.Position = FVector2D(LatestMouseWorldPos);
		ThiefSeek->SetTarget(Target);
	}
	
	if (Agent)
	{
		DrawDebugCircle(GetWorld(),
			FVector(Agent->GetPosition(), 90.f),
			DetectionRadius, 32,
			FColor::Yellow, false, -1.f, 0, 2.f,
			FVector(0,1,0), FVector(1,0,0));
	}
	
	ImGui::SetNextWindowPos(WindowPos);
	ImGui::SetNextWindowSize(WindowSize);
	ImGui::Begin("Game AI", nullptr,
		ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("Move mouse: move thief");
	ImGui::Text("WASD: move cam");
	ImGui::Text("Scrollwheel: zoom cam");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Guard Settings");
	ImGui::SliderFloat("Detection Radius", &DetectionRadius, 100.f, 1000.f, "%.0f");
	ImGui::SliderFloat("Search Duration",  &SearchDuration,  1.f,   20.f,  "%.1f");

	ImGui::End();
}

void ALevel_FSM::SetupThief()
{
	ThiefAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass, FVector{200.f, 0.f, 90.f}, FRotator::ZeroRotator);

	ThiefSeek = std::make_unique<Seek>();
	ThiefAgent->SetSteeringBehavior(ThiefSeek.get());
}

void ALevel_FSM::SetupGuard()
{
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass, FVector{0.f, 0.f, 90.f}, FRotator::ZeroRotator);
	Agent->SetDebugRenderingEnabled(false);

	GuardController = GetWorld()->SpawnActor<AGameAIController>();
	GuardController->Possess(Agent);

	std::vector<FVector2D> PatrolPath =
	{
		{-600.f, -600.f},
		{ 600.f, -600.f},
		{ 600.f,  600.f},
		{-600.f,  600.f},
	};

	UFSMComponent* FSMComp = Cast<UFSMComponent>(GuardController->GetBrainComponent());
	if (!ensure(FSMComp)) return;

	GameAI::FSM::Blackboard& BB = FSMComp->GetBlackboard();
	BB.Set("PatrolPath", PatrolPath);
	BB.Set<ASteeringAgent*>("Thief", ThiefAgent);

	// -- States --
	GameAI::FSM::State* pPatrol = FSMComp->AddState(std::make_unique<GameAI::FSM::PatrolState>(NavigationGraph.get()));
	GameAI::FSM::State* pChase  = FSMComp->AddState(std::make_unique<GameAI::FSM::ChaseState>());
	GameAI::FSM::State* pSearch = FSMComp->AddState(std::make_unique<GameAI::FSM::SearchState>(NavigationGraph.get()));

	auto IsTargetVisible = [this]() -> bool
	{
		if (!Agent || !ThiefAgent) return false;

		FVector2D GuardPos = Agent->GetPosition();
		FVector2D ThiefPos = ThiefAgent->GetPosition();

		if (FVector2D::Distance(GuardPos, ThiefPos) > DetectionRadius)
			return false;

		FVector Start(GuardPos, 90.f);
		FVector End(ThiefPos, 90.f);
		FHitResult Hit;
		FCollisionQueryParams Params;
		Params.AddIgnoredActor(Agent);
		Params.AddIgnoredActor(ThiefAgent);
		return !GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_WorldStatic, Params);
	};

	auto IsTargetNotVisible = [IsTargetVisible]() -> bool
	{
		return !IsTargetVisible();
	};

	auto IsSearchingTooLong = [this, FSMComp]() -> bool
	{
		if (!FSMComp) return false;
		GameAI::FSM::Blackboard& BB = FSMComp->GetBlackboard();
		if (!BB.Has("SearchElapsedTime")) return false;
		return BB.Get<float>("SearchElapsedTime") >= SearchDuration;
	};

	FSMComp->AddTransition(pPatrol, pChase,  IsTargetVisible);
	FSMComp->AddTransition(pChase,  pSearch, IsTargetNotVisible);
	FSMComp->AddTransition(pSearch, pChase,  IsTargetVisible);
	FSMComp->AddTransition(pSearch, pPatrol, IsSearchingTooLong);
	
	FSMComp->SetInitialState(pPatrol);
	GuardController->RunFiniteStateMachine();
	
	
}
TArray<TArray<FVector>> ALevel_FSM::ExtractNavMeshTris() const
{
	TArray<TArray<FVector>> Polys{};

	ANavigationData* NavData = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld())->GetDefaultNavDataInstance();
	if (dtNavMesh const* NavMesh = Cast<ARecastNavMesh>(NavData)->GetRecastMesh())
	{
		for (int TileIdx{0}; TileIdx < NavMesh->getMaxTiles(); ++TileIdx)
		{
			dtMeshTile const* Tile{NavMesh->getTile(TileIdx)};
			if (!Tile || !Tile->header || !Tile->polys) continue;

			for (int i = 0; i < Tile->header->detailMeshCount; ++i)
			{
				const dtPolyDetail* DetailMesh = &Tile->detailMeshes[i];
				const dtPoly* Poly = &Tile->polys[i];

				for (int triIdx = 0; triIdx < DetailMesh->triCount; ++triIdx)
				{
					const unsigned char* TriData = &Tile->detailTris[(DetailMesh->triBase + triIdx) * 4];
					TArray<FVector> TriVerts{};

					for (int corner = 0; corner < 3; ++corner)
					{
						unsigned char idx = TriData[corner];
						const double* Vert;

						if (idx < Poly->vertCount)
							Vert = &Tile->verts[Poly->verts[idx] * 3];
						else
						{
							int detailVertIdx = DetailMesh->vertBase + (idx - Poly->vertCount);
							Vert = &Tile->detailVerts[detailVertIdx * 3];
						}

						TriVerts.Add(FVector(
							static_cast<float>(-Vert[0]),
							static_cast<float>(-Vert[2]),
							static_cast<float>(Vert[1])
						));
					}
					Polys.Add(TriVerts);
				}
			}
		}
	}

	return Polys;
}