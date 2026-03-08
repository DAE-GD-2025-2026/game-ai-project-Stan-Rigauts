#include "FlockingSteeringBehaviors.h"
#include "Flock.h"


SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering{};
    if (pFlock->GetNrOfNeighbors() == 0) return steering;

    const FVector2D toCenter = pFlock->GetAverageNeighborPos() - FVector2D(pAgent.GetActorLocation());
    if (toCenter.IsNearlyZero()) return steering;

    steering.LinearVelocity = toCenter.GetSafeNormal();
    steering.IsValid = true;
    return steering;
}

SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering{};
    const int nrOfNeighbors = pFlock->GetNrOfNeighbors();
    if (nrOfNeighbors == 0) return steering;

    const FVector2D agentPos = FVector2D(pAgent.GetActorLocation());
    const TArray<ASteeringAgent*>& neighbors = pFlock->GetNeighbors();
    const float separationRadius = 110.f;
    FVector2D separationForce = FVector2D::ZeroVector;

    for (int i = 0; i < nrOfNeighbors; ++i)
    {
        if (!neighbors[i]) continue;
        const FVector2D toAgent = agentPos - FVector2D(neighbors[i]->GetActorLocation());
        const float distance = toAgent.Size();
        if (distance < KINDA_SMALL_NUMBER || distance > separationRadius) continue;
        separationForce += toAgent.GetSafeNormal() / distance;
    }

    if (separationForce.IsNearlyZero()) return steering;

    steering.LinearVelocity = separationForce.GetSafeNormal();
    steering.IsValid = true;

    return steering;
}

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering{};
    if (pFlock->GetNrOfNeighbors() == 0) return steering;

    const FVector2D avgVelocity = pFlock->GetAverageNeighborVelocity();
    if (avgVelocity.IsNearlyZero()) return steering;

    steering.LinearVelocity = avgVelocity.GetSafeNormal();
    steering.IsValid = true;
    return steering;
}