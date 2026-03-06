#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	if (pFlock->GetNrOfNeighbors() == 0)
		return SteeringOutput{};

	SteeringOutput Steering{};
	FVector2D ToCenter = pFlock->GetAverageNeighborPos() - pAgent.GetPosition();
	Steering.LinearVelocity = ToCenter.GetSafeNormal();
	return Steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	int Count = pFlock->GetNrOfNeighbors();
	if (Count == 0) return Steering;

	const TArray<ASteeringAgent*>& Neighbors = pFlock->GetNeighbors();
	FVector2D TotalForce = FVector2D::ZeroVector;

	for (int i = 0; i < Count; ++i)
	{
		if (!Neighbors[i]) continue;
		FVector2D ToNeighbor = Neighbors[i]->GetPosition() - pAgent.GetPosition();
		float Distance = ToNeighbor.Length();
		if (Distance > 0.f)
		{
			float DistanceInMeters = Distance / 100.f;
			TotalForce += -ToNeighbor.GetSafeNormal() * (1.f / (DistanceInMeters * DistanceInMeters));
		}
	}

	Steering.LinearVelocity = TotalForce;
	return Steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	if (pFlock->GetNrOfNeighbors() == 0) return Steering;

	Steering.LinearVelocity = pFlock->GetAverageNeighborVelocity();
	return Steering;
}