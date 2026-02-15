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

    FVector CircleCenter3D(CircleCenter.X, CircleCenter.Y, AgentPos3D.Z);
    FVector WanderTarget3D(WanderTarget.X, WanderTarget.Y, AgentPos3D.Z);
    FVector AgentPos3D_Draw(AgentPos2D.X, AgentPos2D.Y, AgentPos3D.Z);

    DrawDebugCircle(Agent.GetWorld(),CircleCenter3D,m_WanderRadius,32,FColor::Green,false,-1.f,0,2.f,FVector(1, 0, 0),FVector(0, 1, 0),false);

    DrawDebugLine(Agent.GetWorld(),AgentPos3D_Draw,WanderTarget3D,FColor::Red,false,-1.f,0,2.f
    );

    DrawDebugPoint(Agent.GetWorld(),WanderTarget3D,10.f,FColor::Yellow,false,-1.f,0);


    Target.Position = WanderTarget;
    Steering = Seek::CalculateSteering(DeltaTime, Agent);

    return Steering;
}

