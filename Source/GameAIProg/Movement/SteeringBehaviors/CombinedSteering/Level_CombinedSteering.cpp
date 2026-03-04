#include "Level_CombinedSteering.h"

#include "imgui.h"


// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	pDrunkAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector(200.f, 0.f, 0.f),
		FRotator::ZeroRotator,
		SpawnParams
	);

	pSeekBehavior   = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();

	std::vector<BlendedSteering::WeightedBehavior> BlendedBehaviors;
	BlendedBehaviors.emplace_back(pSeekBehavior.get(),   0.5f);
	BlendedBehaviors.emplace_back(pWanderBehavior.get(), 0.5f);
	pBlendedSteering = std::make_unique<BlendedSteering>(BlendedBehaviors);

	if (pDrunkAgent)
		pDrunkAgent->SetSteeringBehavior(pBlendedSteering.get());

	pEvadingAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector(-200.f, 0.f, 0.f),
		FRotator::ZeroRotator,
		SpawnParams
	);

	pEvadeBehavior        = std::make_unique<Evade>();
	pEvaderWanderBehavior = std::make_unique<Wander>();

	if (pEvadingAgent)
		pEvadingAgent->SetSteeringBehavior(pEvaderWanderBehavior.get());
}

void ALevel_CombinedSteering::BeginDestroy()
{
	pDrunkAgent   = nullptr;
	pEvadingAgent = nullptr;
	Super::BeginDestroy();
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Combined Steering");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			if (pDrunkAgent)   pDrunkAgent->SetDebugRenderingEnabled(CanDebugRender);
			if (pEvadingAgent) pEvadingAgent->SetDebugRenderingEnabled(CanDebugRender);
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		auto& Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			Behaviors[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			Behaviors[1].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");

		ImGui::Spacing();
		ImGui::Text("EvadingAgent");
		ImGui::Spacing();

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Evade Radius",
			EvadeRadius, 50.f, 800.f,
			[this](float InVal) { EvadeRadius = InVal; }, "%.0f");
	
		//End
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
	if (UseMouseTarget && pSeekBehavior)
	{
		FTargetData SeekTarget;
		SeekTarget.Position = MouseTarget.Position;
		pSeekBehavior->SetTarget(SeekTarget);
	}

	if (pDrunkAgent && pEvadeBehavior && pEvadingAgent)
	{
		FVector2D DrunkPos  = FVector2D(pDrunkAgent->GetActorLocation());
		FVector2D EvaderPos = FVector2D(pEvadingAgent->GetActorLocation());
		float     Dist      = FVector2D::Distance(DrunkPos, EvaderPos);

		if (Dist <= EvadeRadius)
		{
			FTargetData EvadeTarget;
			EvadeTarget.Position       = DrunkPos;
			EvadeTarget.LinearVelocity = FVector2D(pDrunkAgent->GetVelocity());
			pEvadeBehavior->SetTarget(EvadeTarget);
			pEvadingAgent->SetSteeringBehavior(pEvadeBehavior.get());
		}
		else
		{
			pEvadingAgent->SetSteeringBehavior(pEvaderWanderBehavior.get());
		}
	}
}
