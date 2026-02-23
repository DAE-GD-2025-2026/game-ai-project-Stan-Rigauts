#include "Level_CombinedSteering.h"

#include <format>
#include <string>
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

	AddAgent();
	CombinedAgents[0].Agent->SetDebugRenderingEnabled(true);
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
	ImGui::SetNextWindowPos(WindowPos);
	ImGui::SetNextWindowSize(WindowSize);
	ImGui::Begin("Game AI", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("WASD: move cam");
	ImGui::Text("Scrollwheel: zoom cam");
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

	ImGui::Text("Combined Steering Behaviors");
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
	if (TrimWorld->bShouldTrimWorld)
	{
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
			TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
			[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
	}
	ImGui::Spacing();

#pragma region PerAgentUI
	if (ImGui::Button("Add Agent"))
		AddAgent();
	ImGui::Separator();

	for (int i{ 0 }; i < CombinedAgents.size(); ++i)
	{
		ImGui::PushID(i);
		ImGui_CombinedAgent& a = CombinedAgents[i];

		std::string agentHeader{ std::format("Agent {}:", i) };
		if (ImGui::CollapsingHeader(agentHeader.c_str()))
		{
			ImGui::Indent();

			if (ImGui::CollapsingHeader("Properties"))
			{
				float v = a.Agent->GetMaxLinearSpeed();
				if (ImGui::SliderFloat("Lin", &v, 0.f, 600.f, "%.2f"))
					a.Agent->SetMaxLinearSpeed(v);

				v = a.Agent->GetMaxAngularSpeed();
				if (ImGui::SliderFloat("Ang", &v, 0.f, 360.f, "%.2f"))
					a.Agent->SetMaxAngularSpeed(v);

				v = a.Agent->GetMass();
				if (ImGui::SliderFloat("Mass ", &v, 0.f, 100.f, "%.2f"))
					a.Agent->SetMass(v);
			}

			bool bBehaviorModified = false;

			ImGui::Spacing();

			ImGui::Text("Combined Type: ");
			ImGui::SameLine();
			ImGui::PushItemWidth(100);
			int selectedType = static_cast<int>(a.BehaviorType);
			if (ImGui::Combo("##CombinedType", &selectedType, "Blended\0Priority\0", 2))
			{
				a.BehaviorType = static_cast<CombinedBehaviorType>(selectedType);
				bBehaviorModified = true;
			}
			ImGui::PopItemWidth();

			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();

			ImGui::PushID(i + 100);
			ImGui::Text("Target: ");
			ImGui::SameLine();
			ImGui::PushItemWidth(100);

			int selectedTargetOffset = a.SelectedTarget + 1;
			std::string const Label{ "" };
			std::string Targets{};
			for (auto const& Target : TargetLabels)
			{
				Targets += Target;
				Targets += '\0';
			}
			if (ImGui::Combo(Label.c_str(), &selectedTargetOffset, Targets.c_str()))
			{
				a.SelectedTarget = selectedTargetOffset - 1;
				bBehaviorModified = true;
			}

			ImGui::PopItemWidth();
			ImGui::PopID();
			ImGui::Spacing();

			ImGui::Text("Active Behaviors:");
			ImGui::Spacing();

			int enabledCount = 0;
			if (a.bUseSeek) enabledCount++;
			if (a.bUseFlee) enabledCount++;
			if (a.bUseWander) enabledCount++;
			if (a.bUseArrive) enabledCount++;
			if (a.bUseEvade) enabledCount++;
			if (a.bUsePursuit) enabledCount++;

			if (enabledCount == 0)
			{
				ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Warning: No behaviors enabled!");
				ImGui::Spacing();
			}

			if (a.BehaviorType == CombinedBehaviorType::Priority)
			{
				ImGui::Text("Priority Order (higher = checked first):");
				ImGui::Spacing();

				const char* behaviorNames[] = { "Seek", "Flee", "Wander", "Arrive", "Evade", "Pursuit" };
				bool* behaviorEnabled[] = { &a.bUseSeek, &a.bUseFlee, &a.bUseWander, &a.bUseArrive, &a.bUseEvade, &a.bUsePursuit };

				for (int priorityIdx = 0; priorityIdx < a.PriorityOrder.size(); ++priorityIdx)
				{
					int behaviorIdx = a.PriorityOrder[priorityIdx];

					ImGui::PushID(priorityIdx + 200);

					ImGui::Text("%d.", priorityIdx + 1);
					ImGui::SameLine();

					if (ImGui::Checkbox(behaviorNames[behaviorIdx], behaviorEnabled[behaviorIdx]))
						bBehaviorModified = true;

					ImGui::SameLine(200);
					if (priorityIdx > 0 && ImGui::Button("U"))
					{
						std::swap(a.PriorityOrder[priorityIdx], a.PriorityOrder[priorityIdx - 1]);
						bBehaviorModified = true;
					}

					ImGui::SameLine();
					if (priorityIdx < a.PriorityOrder.size() - 1 && ImGui::Button("D"))
					{
						std::swap(a.PriorityOrder[priorityIdx], a.PriorityOrder[priorityIdx + 1]);
						bBehaviorModified = true;
					}

					ImGui::PopID();
				}
			}
			else // Blended mode
			{
				if (ImGui::Checkbox("Seek", &a.bUseSeek))
					bBehaviorModified = true;
				if (a.bUseSeek)
				{
					ImGui::SameLine(150);
					ImGui::PushItemWidth(150);
					if (ImGui::SliderFloat("##SeekWeight", &a.SeekWeight, 0.f, 1.f, "%.2f"))
						bBehaviorModified = true;
					ImGui::PopItemWidth();
				}

				if (ImGui::Checkbox("Flee", &a.bUseFlee))
					bBehaviorModified = true;
				if (a.bUseFlee)
				{
					ImGui::SameLine(150);
					ImGui::PushItemWidth(150);
					if (ImGui::SliderFloat("##FleeWeight", &a.FleeWeight, 0.f, 1.f, "%.2f"))
						bBehaviorModified = true;
					ImGui::PopItemWidth();
				}

				if (ImGui::Checkbox("Wander", &a.bUseWander))
					bBehaviorModified = true;
				if (a.bUseWander)
				{
					ImGui::SameLine(150);
					ImGui::PushItemWidth(150);
					if (ImGui::SliderFloat("##WanderWeight", &a.WanderWeight, 0.f, 1.f, "%.2f"))
						bBehaviorModified = true;
					ImGui::PopItemWidth();
				}

				if (ImGui::Checkbox("Arrive", &a.bUseArrive))
					bBehaviorModified = true;
				if (a.bUseArrive)
				{
					ImGui::SameLine(150);
					ImGui::PushItemWidth(150);
					if (ImGui::SliderFloat("##ArriveWeight", &a.ArriveWeight, 0.f, 1.f, "%.2f"))
						bBehaviorModified = true;
					ImGui::PopItemWidth();
				}

				if (ImGui::Checkbox("Evade", &a.bUseEvade))
					bBehaviorModified = true;
				if (a.bUseEvade)
				{
					ImGui::SameLine(150);
					ImGui::PushItemWidth(150);
					if (ImGui::SliderFloat("##EvadeWeight", &a.EvadeWeight, 0.f, 1.f, "%.2f"))
						bBehaviorModified = true;
					ImGui::PopItemWidth();
				}

				if (ImGui::Checkbox("Pursuit", &a.bUsePursuit))
					bBehaviorModified = true;
				if (a.bUsePursuit)
				{
					ImGui::SameLine(150);
					ImGui::PushItemWidth(150);
					if (ImGui::SliderFloat("##PursuitWeight", &a.PursuitWeight, 0.f, 1.f, "%.2f"))
						bBehaviorModified = true;
					ImGui::PopItemWidth();
				}
			}

			ImGui::Spacing();
			ImGui::Spacing();

			if (bBehaviorModified)
				RebuildAgentBehavior(a);

			if (ImGui::Button("x"))
			{
				AgentIndexToRemove = i;
			}

			ImGui::SameLine(0, 20);

			bool isChecked = a.Agent->GetDebugRenderingEnabled();
			if (ImGui::Checkbox("Debug Rendering", &isChecked))
			{
				a.Agent->SetDebugRenderingEnabled(isChecked);
			}

			ImGui::Unindent();
		}
#pragma endregion 

		ImGui::PopID();
	}

	if (AgentIndexToRemove >= 0)
	{
		RemoveAgent(AgentIndexToRemove);
		AgentIndexToRemove = -1;
	}

	ImGui::End();
#pragma endregion

	for (ImGui_CombinedAgent& a : CombinedAgents)
	{
		if (a.Agent)
		{
			UpdateTargets(a);
		}
	}
}

bool ALevel_CombinedSteering::AddAgent()
{
	ImGui_CombinedAgent NewAgent = {};
	NewAgent.Agent = GetWorld()->SpawnActor<ASteeringAgent>(
		SteeringAgentClass,
		FVector{ 0, 0, 90 },
		FRotator::ZeroRotator
	);

	if (IsValid(NewAgent.Agent))
	{
		NewAgent.SeekBehavior = std::make_unique<Seek>();
		NewAgent.FleeBehavior = std::make_unique<Flee>();
		NewAgent.WanderBehavior = std::make_unique<Wander>();
		NewAgent.ArriveBehavior = std::make_unique<Arrive>();
		NewAgent.EvadeBehavior = std::make_unique<Evade>();
		NewAgent.PursuitBehavior = std::make_unique<Pursuit>();

		NewAgent.SelectedTarget = -1; 

		RebuildAgentBehavior(NewAgent);

		CombinedAgents.push_back(std::move(NewAgent));

		RefreshTargetLabels();

		return true;
	}

	return false;
}

void ALevel_CombinedSteering::RemoveAgent(unsigned int Index)
{
	CombinedAgents[Index].Agent->Destroy();
	CombinedAgents.erase(CombinedAgents.begin() + Index);

	RefreshTargetLabels();
	RefreshAgentTargets(Index);
}

void ALevel_CombinedSteering::RebuildAgentBehavior(ImGui_CombinedAgent& Agent)
{
	Agent.CombinedBehavior.reset();

	if (Agent.BehaviorType == CombinedBehaviorType::Blended) //blended steering
	{
		std::vector<BlendedSteering::WeightedBehavior> weightedBehaviors;

		if (Agent.bUseSeek)
			weightedBehaviors.push_back(BlendedSteering::WeightedBehavior(Agent.SeekBehavior.get(), Agent.SeekWeight));

		if (Agent.bUseFlee)
			weightedBehaviors.push_back(BlendedSteering::WeightedBehavior(Agent.FleeBehavior.get(), Agent.FleeWeight));

		if (Agent.bUseWander)
			weightedBehaviors.push_back(BlendedSteering::WeightedBehavior(Agent.WanderBehavior.get(), Agent.WanderWeight));

		if (Agent.bUseArrive)
			weightedBehaviors.push_back(BlendedSteering::WeightedBehavior(Agent.ArriveBehavior.get(), Agent.ArriveWeight));

		if (Agent.bUseEvade)
			weightedBehaviors.push_back(BlendedSteering::WeightedBehavior(Agent.EvadeBehavior.get(), Agent.EvadeWeight));

		if (Agent.bUsePursuit)
			weightedBehaviors.push_back(BlendedSteering::WeightedBehavior(Agent.PursuitBehavior.get(), Agent.PursuitWeight));

		if (!weightedBehaviors.empty())
		{
			Agent.CombinedBehavior = std::make_unique<BlendedSteering>(weightedBehaviors);
		}
	}
	else // Priority Steering
	{
		std::vector<ISteeringBehavior*> priorityBehaviors;

		ISteeringBehavior* behaviorPtrs[] = {
			Agent.SeekBehavior.get(),
			Agent.FleeBehavior.get(),
			Agent.WanderBehavior.get(),
			Agent.ArriveBehavior.get(),
			Agent.EvadeBehavior.get(),
			Agent.PursuitBehavior.get()
		};

		bool behaviorEnabled[] = {
			Agent.bUseSeek,
			Agent.bUseFlee,
			Agent.bUseWander,
			Agent.bUseArrive,
			Agent.bUseEvade,
			Agent.bUsePursuit
		};

		for (int behaviorIdx : Agent.PriorityOrder)
		{
			if (behaviorEnabled[behaviorIdx])
			{
				priorityBehaviors.push_back(behaviorPtrs[behaviorIdx]);
			}
		}

		if (!priorityBehaviors.empty())
		{
			Agent.CombinedBehavior = std::make_unique<PrioritySteering>(priorityBehaviors);
		}
	}

	if (Agent.CombinedBehavior)
	{
		Agent.Agent->SetSteeringBehavior(Agent.CombinedBehavior.get());
	}
	else
	{
	
		Agent.Agent->SetSteeringBehavior(nullptr);
	}

	UpdateTargets(Agent);
}

void ALevel_CombinedSteering::RefreshTargetLabels()
{
	TargetLabels.clear();

	TargetLabels.push_back("Mouse");
	for (int i{ 0 }; i < CombinedAgents.size(); ++i)
	{
		TargetLabels.push_back(std::format("Agent {}", i));
	}
}

void ALevel_CombinedSteering::UpdateTargets(ImGui_CombinedAgent& Agent)
{
	FTargetData targetData;

	bool const bUseMouseAsTarget = Agent.SelectedTarget < 0;
	if (!bUseMouseAsTarget)
	{
		ASteeringAgent* const TargetAgent = CombinedAgents[Agent.SelectedTarget].Agent;

		targetData.Position = TargetAgent->GetPosition();
		targetData.Orientation = TargetAgent->GetRotation();
		targetData.LinearVelocity = TargetAgent->GetLinearVelocity();
		targetData.AngularVelocity = TargetAgent->GetAngularVelocity();
	}
	else
	{
		targetData = MouseTarget;
	}

	if (Agent.SeekBehavior)
		Agent.SeekBehavior->SetTarget(targetData);

	if (Agent.FleeBehavior)
		Agent.FleeBehavior->SetTarget(targetData);

	if (Agent.WanderBehavior)
		Agent.WanderBehavior->SetTarget(targetData);

	if (Agent.ArriveBehavior)
		Agent.ArriveBehavior->SetTarget(targetData);

	if (Agent.EvadeBehavior)
		Agent.EvadeBehavior->SetTarget(targetData);

	if (Agent.PursuitBehavior)
		Agent.PursuitBehavior->SetTarget(targetData);
}

void ALevel_CombinedSteering::RefreshAgentTargets(unsigned int IndexRemoved)
{
	for (UINT i = 0; i < CombinedAgents.size(); ++i)
	{
		if (i >= IndexRemoved)
		{
			auto& Agent = CombinedAgents[i];
			if (Agent.SelectedTarget == IndexRemoved || i == Agent.SelectedTarget)
			{
				--Agent.SelectedTarget;
			}
		}
	}
}