
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	float TotalWeight = 0.f;
	for (const WeightedBehavior& WB : WeightedBehaviors)
	{
		if (!WB.pBehavior) continue;

		SteeringOutput Output = WB.pBehavior->CalculateSteering(DeltaT, Agent);
		BlendedSteering.LinearVelocity += Output.LinearVelocity * WB.Weight;
		BlendedSteering.AngularVelocity += Output.AngularVelocity * WB.Weight;
		TotalWeight += WB.Weight;
	}

	if (TotalWeight > 0.f)
	{
		BlendedSteering.LinearVelocity /= TotalWeight;
		BlendedSteering.AngularVelocity /= TotalWeight;
	}
	
	// TODO: Add debug drawing

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

	if(it!= WeightedBehaviors.end())
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

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}