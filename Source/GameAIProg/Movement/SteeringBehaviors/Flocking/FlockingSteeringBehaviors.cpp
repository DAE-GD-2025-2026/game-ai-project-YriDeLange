#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput Steering{};
    if (pFlock->GetNrOfNeighbors() == 0)
        return Steering;

    // Seek toward average neighbor position
    Target.Position = pFlock->GetAverageNeighborPos();
    return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput Steering{};
    if (pFlock->GetNrOfNeighbors() == 0)
        return Steering;

    FVector2D SeparationVelocity = FVector2D::ZeroVector;
    for (int i = 0; i < pFlock->GetNrOfNeighbors(); ++i)
    {
        FVector2D ToNeighbor = pFlock->GetNeighbors()[i]->GetPosition() - pAgent.GetPosition();
        float Distance = ToNeighbor.Size();
        if (Distance > KINDA_SMALL_NUMBER)
        {
            // Inversely proportional to distance (y = 1/x)
            SeparationVelocity -= ToNeighbor.GetSafeNormal() * (1.f / Distance);
        }
    }

    Steering.LinearVelocity = SeparationVelocity;
    return Steering;
}
//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput Steering{};
    if (pFlock->GetNrOfNeighbors() == 0)
        return Steering;

    Steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
    return Steering;
}