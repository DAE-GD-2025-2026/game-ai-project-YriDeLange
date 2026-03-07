#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    Agent.SetMaxLinearSpeed(MaxLinearSpeed);
    SteeringOutput Steering{};
    Steering.LinearVelocity = Target.Position - Agent.GetPosition();

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc3D = FVector(Agent.GetPosition(), 0.f);
        const FVector TargetLoc3D = FVector(Target.Position, 0.f);

        DrawDebugSphere(Agent.GetWorld(), AgentLoc3D, 20.f, 12, FColor::Yellow, false, -1.f);

        DrawDebugSphere(Agent.GetWorld(), TargetLoc3D, 10.f, 12, FColor::Red, false, -1.f);

        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, TargetLoc3D, FColor::Green, false, -1.f, 0, 3.f);

        FVector Velocity3D = Agent.GetVelocity();
        FVector VelocityDir = Velocity3D.GetSafeNormal() * 100.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + VelocityDir, 50.f, FColor::Magenta, false, -1.f, 0, 4.f);

        float RotationRad = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);
        FVector FacingDir = FVector(FMath::Cos(RotationRad), FMath::Sin(RotationRad), 0.f) * 80.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + FacingDir, 40.f, FColor::Cyan, false, -1.f, 0, 3.f);
    }

    return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    Agent.SetMaxLinearSpeed(MaxLinearSpeed);
    SteeringOutput Steering{};
    Steering.LinearVelocity = Agent.GetPosition() - Target.Position;

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc3D = FVector(Agent.GetPosition(), 0.f);
        const FVector TargetLoc3D = FVector(Target.Position, 0.f);

        DrawDebugSphere(Agent.GetWorld(), AgentLoc3D, 20.f, 12, FColor::Yellow, false, -1.f);

        DrawDebugSphere(Agent.GetWorld(), TargetLoc3D, 10.f, 12, FColor::Red, false, -1.f);

        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, TargetLoc3D, FColor::Green, false, -1.f, 0, 3.f);

        FVector Velocity3D = Agent.GetVelocity();
        FVector VelocityDir = Velocity3D.GetSafeNormal() * 100.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + VelocityDir, 50.f, FColor::Magenta, false, -1.f, 0, 4.f);

        float RotationRad = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);
        FVector FacingDir = FVector(FMath::Cos(RotationRad), FMath::Sin(RotationRad), 0.f) * 80.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + FacingDir, 40.f, FColor::Cyan, false, -1.f, 0, 3.f);
    }

    return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    FVector2D ToTarget = Target.Position - Agent.GetPosition();
    float Distance = ToTarget.Size();
    if (Distance < TargetRadius)
    {
        Agent.SetMaxLinearSpeed(0.f);
        Steering.LinearVelocity = FVector2D::ZeroVector;
    }
    else
    {
        float TargetSpeed;

        if (Distance > SlowRadius)
        {
            TargetSpeed = MaxLinearSpeed;
        }
        else
        {
            TargetSpeed = MaxLinearSpeed * (Distance - TargetRadius) / (SlowRadius - TargetRadius);
        }
        Agent.SetMaxLinearSpeed(TargetSpeed);
        Steering.LinearVelocity = ToTarget;
    }

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc3D = FVector(Agent.GetPosition(), 0.f);
        const FVector TargetLoc3D = FVector(Target.Position, 0.f);

        DrawDebugSphere(Agent.GetWorld(), AgentLoc3D, 20.f, 12, FColor::Yellow, false, -1.f);

        DrawDebugSphere(Agent.GetWorld(), TargetLoc3D, 10.f, 12, FColor::Red, false, -1.f);

        FVector2D DesiredVel2D = Steering.LinearVelocity.GetSafeNormal() * 100.f;
        FVector DesiredVel3D = FVector(DesiredVel2D, 0.f);
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + DesiredVel3D, 50.f, FColor::Green, false, -1.f, 0, 4.f);

        FVector Velocity3D = Agent.GetVelocity();
        FVector VelocityDir = Velocity3D.GetSafeNormal() * 100.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + VelocityDir, 50.f, FColor::Magenta, false, -1.f, 0, 4.f);

        float RotationRad = FMath::DegreesToRadians(Agent.GetActorRotation().Yaw);
        FVector FacingDir = FVector(FMath::Cos(RotationRad), FMath::Sin(RotationRad), 0.f) * 80.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + FacingDir, 40.f, FColor::Cyan, false, -1.f, 0, 3.f);

        DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f), TargetRadius, 32, FColor::Red, false, -1.f, 0, 3.f, FVector(0, 1, 0), FVector(1, 0, 0));
        DrawDebugCircle(Agent.GetWorld(), FVector(Agent.GetPosition().X, Agent.GetPosition().Y, 0.f), SlowRadius, 64, FColor::Blue, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
    }

    return Steering;
}

void Arrive::SetTargetRadius(float radius)
{
    TargetRadius = radius;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    Agent.SetMaxAngularSpeed(360.f);

    FVector2D ToTarget = Target.Position - Agent.GetPosition();
    if (ToTarget.IsNearlyZero())
        return Steering;

    float DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
    float CurrentYaw = Agent.GetActorRotation().Yaw;

    float DeltaYaw = FMath::UnwindDegrees(DesiredYaw - CurrentYaw);

    Steering.AngularVelocity = FMath::Clamp(DeltaYaw / Agent.GetMaxAngularSpeed(), -1.f, 1.f);

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc3D = FVector(Agent.GetPosition(), 0.f);
        const FVector TargetLoc3D = FVector(Target.Position, 0.f);

        DrawDebugSphere(Agent.GetWorld(), AgentLoc3D, 20.f, 12, FColor::Yellow, false, -1.f);
        DrawDebugSphere(Agent.GetWorld(), TargetLoc3D, 10.f, 12, FColor::Red, false, -1.f);
        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, TargetLoc3D, FColor::Green, false, -1.f, 0, 3.f);

        float DesiredYawRad = FMath::DegreesToRadians(DesiredYaw);
        FVector DesiredFacingDir = FVector(FMath::Cos(DesiredYawRad), FMath::Sin(DesiredYawRad), 0.f) * 100.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + DesiredFacingDir, 40.f, FColor::Green, false, -1.f, 0, 3.f);

        float CurrentYawRad = FMath::DegreesToRadians(CurrentYaw);
        FVector CurrentFacingDir = FVector(FMath::Cos(CurrentYawRad), FMath::Sin(CurrentYawRad), 0.f) * 80.f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), AgentLoc3D, AgentLoc3D + CurrentFacingDir, 40.f, FColor::Cyan, false, -1.f, 0, 3.f);
    }

    return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    Agent.SetMaxLinearSpeed(MaxLinearSpeed);
    SteeringOutput Steering{};
    FVector2D ToTarget = Target.Position - Agent.GetPosition();
    float Distance = ToTarget.Size();
    float PursuerSpeed = Agent.GetMaxLinearSpeed();
    float TimeToReach = (PursuerSpeed > KINDA_SMALL_NUMBER)
        ? Distance / PursuerSpeed
        : 0.f;
    FVector2D PredictedPosition = Target.Position + (Target.LinearVelocity * TimeToReach);
    Steering.LinearVelocity = PredictedPosition - Agent.GetPosition();

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc3D = FVector(Agent.GetPosition(), 0.f);
        const FVector TargetLoc3D = FVector(Target.Position, 0.f);
        const FVector PredictedLoc3D = FVector(PredictedPosition, 0.f);

        DrawDebugSphere(Agent.GetWorld(), AgentLoc3D, 20.f, 12, FColor::Yellow, false, -1.f);
        DrawDebugSphere(Agent.GetWorld(), TargetLoc3D, 10.f, 12, FColor::Red, false, -1.f);
        DrawDebugSphere(Agent.GetWorld(), PredictedLoc3D, 10.f, 12, FColor::Orange, false, -1.f);
        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, TargetLoc3D, FColor::Red, false, -1.f, 0, 2.f);
        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, PredictedLoc3D, FColor::Green, false, -1.f, 0, 3.f);
        FVector TargetVel3D = FVector(Target.LinearVelocity, 0.f) * 0.5f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), TargetLoc3D, TargetLoc3D + TargetVel3D, 40.f, FColor::Orange, false, -1.f, 0, 3.f);
    }

    return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    Agent.SetMaxLinearSpeed(MaxLinearSpeed);
    SteeringOutput Steering{};
    FVector2D ToTarget = Target.Position - Agent.GetPosition();
    float Distance = ToTarget.Size();

    const float EvadeRadius = 1000.f;
    if (Distance > EvadeRadius)
    {
        Steering.IsValid = false;
        return Steering;
    }

    float PursuerSpeed = Agent.GetMaxLinearSpeed();
    float TimeToReach = (PursuerSpeed > KINDA_SMALL_NUMBER)
        ? Distance / PursuerSpeed
        : 0.f;
    FVector2D PredictedPosition = Target.Position + (Target.LinearVelocity * TimeToReach);
    Steering.LinearVelocity = Agent.GetPosition() - PredictedPosition;

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc3D = FVector(Agent.GetPosition(), 0.f);
        const FVector TargetLoc3D = FVector(Target.Position, 0.f);
        const FVector PredictedLoc3D = FVector(PredictedPosition, 0.f);

        DrawDebugSphere(Agent.GetWorld(), AgentLoc3D, 20.f, 12, FColor::Yellow, false, -1.f);
        DrawDebugSphere(Agent.GetWorld(), TargetLoc3D, 10.f, 12, FColor::Red, false, -1.f);
        DrawDebugSphere(Agent.GetWorld(), PredictedLoc3D, 10.f, 12, FColor::Orange, false, -1.f);
        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, TargetLoc3D, FColor::Red, false, -1.f, 0, 2.f);
        // Line from agent away from the predicted position
        DrawDebugLine(Agent.GetWorld(), AgentLoc3D, PredictedLoc3D, FColor::Green, false, -1.f, 0, 3.f);
        FVector TargetVel3D = FVector(Target.LinearVelocity, 0.f) * 0.5f;
        DrawDebugDirectionalArrow(Agent.GetWorld(), TargetLoc3D, TargetLoc3D + TargetVel3D, 40.f, FColor::Orange, false, -1.f, 0, 3.f);
    }

    return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    Agent.SetMaxLinearSpeed(MaxLinearSpeed);

    SteeringOutput Steering{};

    FVector Velocity3D = Agent.GetVelocity();
    FVector2D Forward = FVector2D(Velocity3D.X, Velocity3D.Y);

    float SpeedSq = Forward.SizeSquared();
    if (SpeedSq > 1.f * 1.f)
    {
        Forward /= FMath::Sqrt(SpeedSq);
    }
    else
    {
        Forward = FVector2D(1.f, 0.f);
    }

    FVector2D CircleCenter = Agent.GetPosition() + Forward * m_OffsetDistance;

    float AngleChange = FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);
    m_WanderAngle += AngleChange;


    FVector2D Displacement;
    Displacement.X = FMath::Cos(m_WanderAngle) * m_Radius;
    Displacement.Y = FMath::Sin(m_WanderAngle) * m_Radius;

    FVector2D Right = FVector2D(-Forward.Y, Forward.X);
    FVector2D LocalToWorldX = Forward;
    FVector2D LocalToWorldY = Right;

    FVector2D WorldDisplacement = LocalToWorldX * Displacement.X + LocalToWorldY * Displacement.Y;

    FVector2D TargetPosition = CircleCenter + WorldDisplacement;

    Steering.LinearVelocity = TargetPosition - Agent.GetPosition();

    if (Steering.LinearVelocity.SizeSquared() < 1.f)
    {
        Steering.LinearVelocity = FVector2D::ZeroVector;
    }

    if (Agent.GetDebugRenderingEnabled())
    {
        const FVector AgentLoc = FVector(Agent.GetPosition(), 0.f);
        const FVector CenterLoc = FVector(CircleCenter, 0.f);
        const FVector TargetLoc = FVector(TargetPosition, 0.f);

        DrawDebugLine(Agent.GetWorld(), AgentLoc, CenterLoc, FColor::Cyan, false, -1.f, 0, 2.f);

        DrawDebugLine(Agent.GetWorld(), CenterLoc, TargetLoc, FColor::Green, false, -1.f, 0, 3.f);

        DrawDebugCircle(
            Agent.GetWorld(),
            CenterLoc,
            m_Radius,
            32,
            FColor::Blue,
            false, -1.f, 0, 2.f,
            FVector(0, 0, 1)
        );

        DrawDebugSphere(Agent.GetWorld(), TargetLoc, 15.f, 12, FColor::Yellow, false, -1.f);

        FVector VelocityDisplay = FVector(Velocity3D.X, Velocity3D.Y, 0.f) * 0.5f;
        DrawDebugDirectionalArrow(
            Agent.GetWorld(),
            AgentLoc,
            AgentLoc + VelocityDisplay,
            80.f,
            FColor::Magenta,
            false, -1.f, 0, 4.f
        );
    }

    return Steering;
}
