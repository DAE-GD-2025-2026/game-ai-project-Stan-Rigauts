#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"
#include "DrawDebugHelpers.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{
};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	float TotalWeight = 0.f;

	for (const WeightedBehavior& weightedBehavior : WeightedBehaviors)
	{
		if (weightedBehavior.pBehavior && weightedBehavior.Weight > 0.f)
		{
			SteeringOutput behaviorOutput = weightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent);

			BlendedSteering.LinearVelocity += behaviorOutput.LinearVelocity * weightedBehavior.Weight;
			BlendedSteering.AngularVelocity += behaviorOutput.AngularVelocity * weightedBehavior.Weight;

			TotalWeight += weightedBehavior.Weight;

			if (Agent.GetDebugRenderingEnabled())
			{
				FVector AgentPos3D = Agent.GetActorLocation();
				FVector2D AgentPos2D(AgentPos3D.X, AgentPos3D.Y);

				FVector2D WeightedVector = behaviorOutput.LinearVelocity * weightedBehavior.Weight * 100.f;
				FVector EndPos3D(AgentPos2D.X + WeightedVector.X, AgentPos2D.Y + WeightedVector.Y, AgentPos3D.Z);

				FColor DebugColor = FColor::Cyan;
				DrawDebugLine(Agent.GetWorld(), AgentPos3D, EndPos3D, DebugColor, false, -1.f, 0, 1.f);
			}
		}
	}

	if (TotalWeight > 0.f)
	{
		BlendedSteering.LinearVelocity /= TotalWeight;
		BlendedSteering.AngularVelocity /= TotalWeight;
	}

	BlendedSteering.IsValid = !BlendedSteering.LinearVelocity.IsNearlyZero();

	if (Agent.GetDebugRenderingEnabled() && BlendedSteering.IsValid)
	{
		FVector AgentPos3D = Agent.GetActorLocation();
		FVector2D AgentPos2D(AgentPos3D.X, AgentPos3D.Y);

		FVector2D FinalVector = BlendedSteering.LinearVelocity * 200.f;  
		FVector EndPos3D(AgentPos2D.X + FinalVector.X, AgentPos2D.Y + FinalVector.Y, AgentPos3D.Z);

		DrawDebugLine(Agent.GetWorld(), AgentPos3D, EndPos3D, FColor::Yellow, false, -1.f, 0, 3.f);
		DrawDebugPoint(Agent.GetWorld(), EndPos3D, 10.f, FColor::Yellow, false, -1.f, 0);
	}

	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if (it != WeightedBehaviors.end())
		return &it->Weight;

	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	return Steering;
}