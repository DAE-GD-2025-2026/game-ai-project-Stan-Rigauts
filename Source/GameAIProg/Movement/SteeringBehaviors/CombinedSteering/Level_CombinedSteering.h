// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "CombinedSteeringBehaviors.h"
#include "GameAIProg/Shared/Level_Base.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include <vector>
#include <memory>
#include <string>
#include "Level_CombinedSteering.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_CombinedSteering : public ALevel_Base
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ALevel_CombinedSteering();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void BeginDestroy() override;

private:
	enum class CombinedBehaviorType
	{
		Blended,
		Priority
	};

	struct ImGui_CombinedAgent final
	{
		ASteeringAgent* Agent{ nullptr };

		std::unique_ptr<Seek> SeekBehavior{ nullptr };
		std::unique_ptr<Flee> FleeBehavior{ nullptr };
		std::unique_ptr<Wander> WanderBehavior{ nullptr };
		std::unique_ptr<Arrive> ArriveBehavior{ nullptr };
		std::unique_ptr<Evade> EvadeBehavior{ nullptr };
		std::unique_ptr<Pursuit> PursuitBehavior{ nullptr };

		std::unique_ptr<ISteeringBehavior> CombinedBehavior{ nullptr };

		CombinedBehaviorType BehaviorType{ CombinedBehaviorType::Blended };

		bool bUseSeek{ true };
		bool bUseFlee{ false };
		bool bUseWander{ false };
		bool bUseArrive{ false };
		bool bUseEvade{ false };
		bool bUsePursuit{ false };

		float SeekWeight{ 0.5f };
		float FleeWeight{ 0.5f };
		float WanderWeight{ 0.5f };
		float ArriveWeight{ 0.5f };
		float EvadeWeight{ 0.5f };
		float PursuitWeight{ 0.5f };

		int SelectedTarget{ -1 }; 

		std::vector<int> PriorityOrder{ 0, 1, 2, 3, 4, 5 };
	};

	//Datamembers
	bool CanDebugRender = false;

	std::vector<ImGui_CombinedAgent> CombinedAgents{};
	std::vector<std::string> TargetLabels{};

	int AgentIndexToRemove = -1;

	// Agent 
	bool AddAgent();
	void RemoveAgent(unsigned int Index);
	void RebuildAgentBehavior(ImGui_CombinedAgent& Agent);

	// Target
	void RefreshTargetLabels();
	void UpdateTargets(ImGui_CombinedAgent& Agent);
	void RefreshAgentTargets(unsigned int IndexRemoved);
};