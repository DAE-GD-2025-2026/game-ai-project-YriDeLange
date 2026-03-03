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

	// Spawn the wanderer agent (just wanders freely)
	pWandererAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass, FVector{ 200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	pWanderBehavior = std::make_unique<Wander>();
	pWandererAgent->SetIsAutoOrienting(true);
	pWandererAgent->SetSteeringBehavior(pWanderBehavior.get());

	// Spawn the seeker agent — uses BlendedSteering (Seek + Wander)
	// wrapped in PrioritySteering with Evade taking priority
	pSeekerAgent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass, FVector{ -200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	pSeekerAgent->SetIsAutoOrienting(true);

	pSeekBehavior = std::make_unique<Seek>();
	pEvadeBehavior = std::make_unique<Evade>();

	// BlendedSteering: Seek toward mouse + a little Wander
	auto pSeekForBlend = std::make_unique<Seek>();
	auto pWanderForBlend = std::make_unique<Wander>();

	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{ pSeekBehavior.get(), 0.7f },
			{ pWanderBehavior.get(), 0.3f }
	});

	// PrioritySteering: Evade the wanderer first, fall back to BlendedSteering
	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() });

	pSeekerAgent->SetSteeringBehavior(pPrioritySteering.get());
}

void ALevel_CombinedSteering::BeginDestroy()
{
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
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
   // TODO: Handle the debug rendering of your agents here :)
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


		 ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
		 	pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
		 	[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
		 ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
		 pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
		 [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
	
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
	// Update seek target to mouse
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(MouseTarget);

	// Update evade target to the wanderer's current position
	if (pEvadeBehavior && pWandererAgent)
	{
		FTargetData WandererTarget;
		WandererTarget.Position = pWandererAgent->GetPosition();
		WandererTarget.LinearVelocity = pWandererAgent->GetLinearVelocity();
		pEvadeBehavior->SetTarget(WandererTarget);
	}
}
