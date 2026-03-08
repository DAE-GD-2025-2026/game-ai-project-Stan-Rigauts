#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	Steering.IsValid = true;
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	Steering.LinearVelocity.Normalize();
	Steering.IsValid = true; 
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	float arriveVelocity{ 1 };
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	float dist = Steering.LinearVelocity.Length();
	if (dist < minRad) arriveVelocity = 0;
	else if (dist > maxRad) arriveVelocity = 1;
	else arriveVelocity = (dist - minRad) / (maxRad - minRad);

	Steering.LinearVelocity.Normalize();
	Steering.LinearVelocity *= arriveVelocity;
	Steering.IsValid = true; 
	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();
	float TargetSpeed = Target.LinearVelocity.Length();
	float Time = (TargetSpeed > 0.0f) ? Distance / TargetSpeed : 0.0f;

	FVector2D PredictedPos = Target.Position + (Target.LinearVelocity * Time);

	Steering.LinearVelocity = PredictedPos - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	Steering.IsValid = true; 

	return Steering;
}


SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	if (!bHasTarget)
	{
		Steering.IsValid = false;
		return Steering;
	}

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();

	const float PanicDistance = 600.f;
	if (Distance < PanicDistance)
	{
		float TargetSpeed = Target.LinearVelocity.Length();
		float Time = (TargetSpeed > 0.0f) ? Distance / TargetSpeed : 0.0f;

		FVector2D PredictedPos = Target.Position + (Target.LinearVelocity * Time);

		Steering.LinearVelocity = Agent.GetPosition() - PredictedPos;
		Steering.LinearVelocity.Normalize();
		Steering.LinearVelocity *= Agent.GetMaxLinearSpeed();

		Steering.IsValid = true;
	}
	else
	{
		Steering.IsValid = false;
	}

	return Steering;
}
#include "DrawDebugHelpers.h"

SteeringOutput Wander::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	float RandomChange = FMath::FRandRange(-m_Maxanglechange, m_Maxanglechange);
	m_WanderAngle += RandomChange;

	FVector Velocity3D = Agent.GetVelocity();
	FVector2D VelocityDir(Velocity3D.X, Velocity3D.Y);
	if (VelocityDir.IsNearlyZero())
	{
		VelocityDir = FVector2D(1.f, 0.f);
	}
	else
	{
		VelocityDir.Normalize();
	}

	FVector AgentPos3D = Agent.GetActorLocation();
	FVector2D AgentPos2D(AgentPos3D.X, AgentPos3D.Y);
	FVector2D CircleCenter = AgentPos2D + VelocityDir * m_WanderOffset;

	FVector2D Displacement(FMath::Cos(m_WanderAngle), FMath::Sin(m_WanderAngle));
	Displacement *= m_WanderRadius;
	FVector2D WanderTarget = CircleCenter + Displacement;

	Target.Position = WanderTarget;
	Steering = Seek::CalculateSteering(DeltaTime, Agent);
	Steering.IsValid = true; 

	return Steering;
}