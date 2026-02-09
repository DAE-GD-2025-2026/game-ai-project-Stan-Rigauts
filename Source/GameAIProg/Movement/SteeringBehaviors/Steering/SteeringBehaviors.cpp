#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{}; 
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{}; 
	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	Steering.LinearVelocity.Normalize();
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{}; 
	float maxRad = 600;
	float minRad = 200;
	float arriveVelocity{1};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	float dist = Steering.LinearVelocity.Length();
	if (dist < minRad) arriveVelocity = 0;
	else if (dist > maxRad) arriveVelocity = 1;
	else arriveVelocity = (dist-minRad)/(maxRad - minRad);
	
	Steering.LinearVelocity.Normalize();
	Steering.LinearVelocity *= arriveVelocity;
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
    
	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();
	float TargetSpeed = Target.LinearVelocity.Length();
	float Time = (TargetSpeed > 0.0f) ? Distance / TargetSpeed : 0.0f;
    
	FVector2D PredictedPos = Target.Position + (Target.LinearVelocity * Time);
    if (Distance<600)
    {
	    Steering.LinearVelocity = -PredictedPos + Agent.GetPosition();
    	Steering.LinearVelocity.Normalize();
    }
    
	return Steering;
}




