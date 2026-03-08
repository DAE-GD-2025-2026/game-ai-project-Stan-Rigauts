#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"

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
	
	Agents.Reserve(FlockSize);
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector SpawnLocation = FVector(
			FMath::RandRange(-WorldSize/2, WorldSize/2),
			FMath::RandRange(-WorldSize/2, WorldSize/2),
			0.f
		);

		ASteeringAgent* pAgent = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			SpawnLocation,
			FRotator::ZeroRotator
		);

		if (pAgent)
			Agents.Add(pAgent);
	}

	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld, WorldSize, WorldSize, NrOfCellsX, NrOfCellsX, FlockSize
	);

	OldPositions.SetNum(FlockSize);
	for (int i = 0; i < Agents.Num(); ++i)
	{
		if (Agents[i])
		{
			pPartitionedSpace->AddAgent(*Agents[i]);
			OldPositions[i] = FVector2D(Agents[i]->GetActorLocation());
		}
	}

	// Create behaviors
	pEvadeBehavior = std::make_unique<Evade>();
	pSeekBehavior = std::make_unique<Seek>();

	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pWanderBehavior = std::make_unique<Wander>();

	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>
	{
		{ pCohesionBehavior.get(), 0.5f },
		{ pSeparationBehavior.get(), 0.6f },
		{ pVelMatchBehavior.get(),   0.3f },
		{ pWanderBehavior.get(),   1.f },
		{ pSeekBehavior.get(),   0.1f },

	});


	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior*>{
		pEvadeBehavior.get()
	    ,pBlendedSteering.get()
		
	}

	);

	for (ASteeringAgent* pAgent : Agents)
		if (pAgent)
			pAgent->SetSteeringBehavior(pPrioritySteering.get());
}

Flock::~Flock()
{
	for (ASteeringAgent* pAgent : Agents)
		if (pAgent)
			pAgent->Destroy();
}

void Flock::Tick(float DeltaTime)
{
	if (pAgentToEvade)
	{
		FTargetData data;
		data.Position = FVector2D(pAgentToEvade->GetActorLocation());
		data.LinearVelocity = FVector2D(pAgentToEvade->GetVelocity());

		pEvadeBehavior->SetTarget(data);
	}

	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* pAgent = Agents[i];
		if (!pAgent) continue;

		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);

		pAgent->Tick(DeltaTime);

		pPartitionedSpace->UpdateAgentCell(*pAgent, OldPositions[i]);
		OldPositions[i] = FVector2D(pAgent->GetActorLocation());
	}

 
}

void Flock::RenderDebug()
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions)
		pPartitionedSpace->RenderCells();
#endif
	if (DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

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


		ImGui::Checkbox("Trim World", &bTrimWorld);

		

		// Neighborhood
		ImGui::Text("Neighborhood");
		ImGui::SliderFloat("Radius", &NeighborhoodRadius, 0.f, 500.f);

		ImGui::Checkbox("Show Neighbor Debug", &ShowDebugRendering);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();


		ImGui::Text("Behavior Weights");

		if (float* w = pBlendedSteering->GetWeight(pCohesionBehavior.get()))
			ImGui::SliderFloat("Cohesion", w, 0.f, 1.f);

		if (float* w = pBlendedSteering->GetWeight(pSeparationBehavior.get()))
			ImGui::SliderFloat("Separation", w, 0.f, 1.f);

		if (float* w = pBlendedSteering->GetWeight(pVelMatchBehavior.get()))
			ImGui::SliderFloat("Velocity Match", w, 0.f, 1.f);

		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (Agents.Num() == 0)
		return;

	ASteeringAgent* pAgent = Agents[0];
	if (!pAgent) return;

	// Register neighbors for this agent
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
	const TArray<ASteeringAgent*>& neighbors = pPartitionedSpace->GetNeighbors();
	int nrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
#else
	RegisterNeighbors(pAgent);
	const TArray<ASteeringAgent*>& neighbors = Neighbors;
	int nrOfNeighbors = NrOfNeighbors;
#endif

	
	for (ASteeringAgent* agent : Agents)
	{
		if (agent)
			agent->SetDebugRenderingEnabled(false);
	}

	if (ShowDebugRendering)
	{
		for (int i = 0; i < nrOfNeighbors; ++i)
		{
			if (neighbors[i])
				neighbors[i]->SetDebugRenderingEnabled(true);
		}

		DrawDebugCircle(
			pWorld,
			pAgent->GetActorLocation(),
			NeighborhoodRadius,
			32,
			FColor::Green,
			false,
			0.f,
			0,
			2.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
}




FVector2D Flock::GetAverageNeighborPos() const
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	const TArray<ASteeringAgent*>& neighbors = pPartitionedSpace->GetNeighbors();
	int nrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
#else
	const TArray<ASteeringAgent*>& neighbors = Neighbors;
	int nrOfNeighbors = NrOfNeighbors;
#endif

	FVector2D avgPosition = FVector2D::ZeroVector;
	if (nrOfNeighbors == 0) return avgPosition;

	for (int i = 0; i < nrOfNeighbors; ++i)
		avgPosition += FVector2D(neighbors[i]->GetActorLocation());

	return avgPosition / static_cast<float>(nrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	const TArray<ASteeringAgent*>& neighbors = pPartitionedSpace->GetNeighbors();
	int nrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
#else
	const TArray<ASteeringAgent*>& neighbors = Neighbors;
	int nrOfNeighbors = NrOfNeighbors;
#endif

	FVector2D avgVelocity = FVector2D::ZeroVector;
	if (nrOfNeighbors == 0) return avgVelocity;

	for (int i = 0; i < nrOfNeighbors; ++i)
		avgVelocity += FVector2D(neighbors[i]->GetVelocity());

	return avgVelocity / static_cast<float>(nrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
	{
		pSeekBehavior->SetTarget(Target);
	}

}

