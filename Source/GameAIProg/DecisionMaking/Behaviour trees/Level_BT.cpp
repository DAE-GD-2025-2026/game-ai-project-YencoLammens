#include "Level_BT.h"
#include "BTGuardController.h"
#include "ThiefController.h"
#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "NavMesh/RecastNavMesh.h"
#include "Runtime/Navmesh/Public/Detour/DetourNavMesh.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Perception/AIPerceptionSystem.h"
#include "Perception/AISense_Sight.h"
#include "DrawDebugHelpers.h"
#include "imgui.h"

ALevel_BT::ALevel_BT()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ALevel_BT::BeginPlay()
{
    Super::BeginPlay();

    SetupNavGraph();
    SetupThief();
    SetupGuard();

    ThiefController->SetGuardAgent(GuardAgent);
}

void ALevel_BT::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (GuardAgent)
    {
        DrawDebugCircle(GetWorld(),
            FVector(GuardAgent->GetPosition(), 90.f),
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
    if (ImGui::SliderFloat("Detection Radius", &DetectionRadius, 100.f, 1000.f, "%.0f"))
    {
        if (GuardController)
            GuardController->SetDetectionRadius(DetectionRadius);
        if (ThiefController)
            ThiefController->SetEvadeDetectionRadius(DetectionRadius);
    }

    ImGui::End();
}

void ALevel_BT::SetupNavGraph()
{
    UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    if (!NavSys) return;

    if (!Cast<ARecastNavMesh>(NavSys->GetDefaultNavDataInstance())) return;

    auto NavPoly = std::make_unique<TriPolygon>();
    for (TArray<FVector> const& Tri : ExtractNavMeshTris())
        NavPoly->AddTriangle(Tri);
    NavigationGraph = std::make_unique<GameAI::NavGraph>(std::move(NavPoly));
}

void ALevel_BT::SetupThief()
{
    ThiefAgent = GetWorld()->SpawnActor<ASteeringAgent>(
        SteeringAgentClass, FVector{200.f, 200.f, 90.f}, FRotator::ZeroRotator);
    if (!ThiefAgent) return;

    UAIPerceptionSystem::RegisterPerceptionStimuliSource(
        GetWorld(), UAISense_Sight::StaticClass(), ThiefAgent);

    ThiefController = GetWorld()->SpawnActor<AThiefController>();
    if (!ThiefController) return;

    ThiefController->Possess(ThiefAgent);
}

void ALevel_BT::SetupGuard()
{
    GuardAgent = GetWorld()->SpawnActor<ASteeringAgent>(
        SteeringAgentClass, FVector{0.f, 0.f, 90.f}, FRotator::ZeroRotator);
    if (!GuardAgent) return;

    GuardAgent->SetDebugRenderingEnabled(false);

    GuardController = GetWorld()->SpawnActor<ABTGuardController>();
    if (!GuardController) return;

    GuardController->GuardBehaviorTree = GuardBehaviorTree;
    GuardController->SetNavGraph(NavigationGraph.get());
    GuardController->SetDetectionRadius(DetectionRadius);
    GuardController->SetPatrolPath({
        {-600.f, -600.f},
        { 600.f, -600.f},
        { 600.f,  600.f},
        {-600.f,  600.f},
    });
    GuardController->Possess(GuardAgent);
}

TArray<TArray<FVector>> ALevel_BT::ExtractNavMeshTris() const
{
    TArray<TArray<FVector>> Polys{};

    UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    if (!NavSys) return Polys;

    ARecastNavMesh* RecastNav = Cast<ARecastNavMesh>(NavSys->GetDefaultNavDataInstance());
    if (!RecastNav) return Polys;

    dtNavMesh const* NavMesh = RecastNav->GetRecastMesh();
    if (!NavMesh) return Polys;

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

    return Polys;
}