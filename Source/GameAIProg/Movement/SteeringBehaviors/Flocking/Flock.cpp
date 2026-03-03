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
{
	Agents.SetNum(FlockSize);
	Neighbors.SetNum(FlockSize);

	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>();

	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{ pCohesionBehavior.get(), 0.3f },
			{ pSeparationBehavior.get(), 0.8f },
			{ pVelMatchBehavior.get(),   0.3f },
			{ pSeekBehavior.get(),       0.2f },
			{ pWanderBehavior.get(),     0.2f }
	});

	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() });

	for (int i = 0; i < FlockSize; ++i)
	{
		FVector SpawnPos{
			FMath::FRandRange(-WorldSize * 0.4f, WorldSize * 0.4f),
			FMath::FRandRange(-WorldSize * 0.4f, WorldSize * 0.4f),
			90.f };

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnPos, FRotator::ZeroRotator, SpawnParams);

		if (IsValid(Agents[i]))
		{
			Agents[i]->SetDebugRenderingEnabled(false);
			Agents[i]->SetIsAutoOrienting(true);
			Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
		}
	}
}

Flock::~Flock()
{
	for (ASteeringAgent* pAgent : Agents)
	{
		if (IsValid(pAgent))
			pAgent->Destroy();
	}
}

void Flock::Tick(float DeltaTime)
{
	if (pAgentToEvade && pEvadeBehavior)
	{
		FTargetData EvadeTarget;
		EvadeTarget.Position = pAgentToEvade->GetPosition();
		EvadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(EvadeTarget);
	}

	for (ASteeringAgent* pAgent : Agents)
	{
		if (!IsValid(pAgent)) continue;

		RegisterNeighbors(pAgent);

		if (pSeekBehavior)
			pSeekBehavior->SetTarget(FTargetData{});
	}
}

void Flock::RenderDebug()
{
	if (DebugRenderNeighborhood)
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

		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
		ImGui::Text("Behavior Weights");
		auto& Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();
		float w0 = Behaviors[0].Weight; if (ImGui::SliderFloat("Cohesion", &w0, 0.f, 1.f)) Behaviors[0].Weight = w0;
		float w1 = Behaviors[1].Weight; if (ImGui::SliderFloat("Separation", &w1, 0.f, 1.f)) Behaviors[1].Weight = w1;
		float w2 = Behaviors[2].Weight; if (ImGui::SliderFloat("Alignment", &w2, 0.f, 1.f)) Behaviors[2].Weight = w2;
		float w3 = Behaviors[3].Weight; if (ImGui::SliderFloat("Seek", &w3, 0.f, 1.f)) Behaviors[3].Weight = w3;
		float w4 = Behaviors[4].Weight; if (ImGui::SliderFloat("Wander", &w4, 0.f, 1.f)) Behaviors[4].Weight = w4;
		ImGui::Spacing();
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	ASteeringAgent* pFirstValid = nullptr;
	for (ASteeringAgent* pAgent : Agents)
	{
		if (IsValid(pAgent))
		{
			pFirstValid = pAgent;
			break;
		}
	}

	if (!pFirstValid) return;

	// Re-register neighbors for this specific agent so the data matches
	RegisterNeighbors(pFirstValid);

	FVector AgentLoc = FVector(pFirstValid->GetPosition(), 0.f);

	DrawDebugCircle(pWorld, AgentLoc, NeighborhoodRadius, 32,
		FColor::Yellow, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		FVector NeighborLoc = FVector(Neighbors[i]->GetPosition(), 0.f);
		DrawDebugLine(pWorld, AgentLoc, NeighborLoc, FColor::Yellow, false, -1.f, 0, 1.f);
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;
	for (ASteeringAgent* pOther : Agents)
	{
		if (pOther == pAgent || !IsValid(pOther)) continue;

		float Distance = FVector2D::Distance(pAgent->GetPosition(), pOther->GetPosition());
		if (Distance < NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = pOther;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D AvgPos = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return AvgPos;

	for (int i = 0; i < NrOfNeighbors; ++i)
		AvgPos += Neighbors[i]->GetPosition();

	return AvgPos / static_cast<float>(NrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D AvgVel = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return AvgVel;

	for (int i = 0; i < NrOfNeighbors; ++i)
		AvgVel += Neighbors[i]->GetLinearVelocity();

	return AvgVel / static_cast<float>(NrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}

