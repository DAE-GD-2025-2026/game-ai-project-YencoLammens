#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    FVector2D velocity = Target.Position - Agent.GetPosition();
    Steering.LinearVelocity = velocity;
    
    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dir = velocity;
        dir.Normalize();
        float lineLength = 250.f;
        FVector2D endPoint = Agent.GetPosition() + (dir * lineLength);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(endPoint, 0), 
            FColor::Green,
            false,
            -1.f,
            0,
            2.f);
    }
    
    return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    FVector2D awayDirection = -toTarget;
    Steering.LinearVelocity = awayDirection;
    
    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dir = awayDirection;
        dir.Normalize();
        float lineLength = 250.f;
        FVector2D endPoint = Agent.GetPosition() + (dir * lineLength);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(endPoint, 0), 
            FColor::Red,
            false,
            -1.f,
            0,
            2.f);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(Target.Position, 0), 
            FColor::Yellow,
            false,
            -1.f,
            0,
            1.f);
    }
    
    return Steering;
}
Arrive::~Arrive()
{
    if (pAgent && OriginalMaxSpeed > 0.f)
    {
        pAgent->SetMaxLinearSpeed(OriginalMaxSpeed);
    }
}
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    if (OriginalMaxSpeed == 0.f)
    {
        OriginalMaxSpeed = Agent.GetMaxLinearSpeed();
        pAgent = &Agent;
    }
    
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();
    
    if (distance > SlowRadius)
    {
        Agent.SetMaxLinearSpeed(OriginalMaxSpeed);
        Steering.LinearVelocity = toTarget;
    }
    else if (distance < TargetRadius)
    {
        Agent.SetMaxLinearSpeed(0.f);
        Steering.LinearVelocity = FVector2D::ZeroVector;
    }
    else
    {
        float speedFactor = (distance - TargetRadius) / (SlowRadius - TargetRadius);
        float newSpeed = OriginalMaxSpeed * speedFactor;
        Agent.SetMaxLinearSpeed(newSpeed);
        Steering.LinearVelocity = toTarget;
    }
    
    if (Agent.GetDebugRenderingEnabled())
    {
        if (distance > TargetRadius)
        {
            FVector2D dir = toTarget;
            dir.Normalize();
            float debugLineLength = FMath::Min(distance, 300.f);
            FVector2D endPoint = Agent.GetPosition() + (dir * debugLineLength);
            
            DrawDebugLine(Agent.GetWorld(),
                FVector(Agent.GetPosition(), 0),
                FVector(endPoint, 0),
                FColor::Cyan,
                false,
                -1.f,
                0,
                2.f);
        }
        
        DrawDebugCircle(Agent.GetWorld(), 
            FVector(Target.Position, 0), 
            SlowRadius, 
            32, 
            FColor::Blue, 
            false, 
            -1.f, 
            0, 
            2.f, 
            FVector(0, 1, 0), 
            FVector(1, 0, 0));
        
        DrawDebugCircle(Agent.GetWorld(), 
            FVector(Target.Position, 0), 
            TargetRadius, 
            32, 
            FColor::Red, 
            false, 
            -1.f, 
            0, 
            2.f, 
            FVector(0, 1, 0), 
            FVector(1, 0, 0));
    }
    
    return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    
    if (toTarget.Length() > 0.01f)
    {
        float desiredAngle = FMath::Atan2(toTarget.Y, toTarget.X);
        float currentAngle = FMath::DegreesToRadians(Agent.GetRotation());
        float angleDiff = desiredAngle - currentAngle;
        
        while (angleDiff > PI) angleDiff -= 2.0f * PI;
        while (angleDiff < -PI) angleDiff += 2.0f * PI;
        
        Steering.AngularVelocity = FMath::RadiansToDegrees(angleDiff);
    }
    
    return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();
    
    float agentSpeed = Agent.GetMaxLinearSpeed();
    float timeToReach = (agentSpeed > 0.f) ? distance / agentSpeed : 0.f;
    
    FVector2D predictedPosition = Target.Position + (Target.LinearVelocity * timeToReach);
    
    Steering.LinearVelocity = predictedPosition - Agent.GetPosition();
    
    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D dirToCurrent = toTarget;
        dirToCurrent.Normalize();
        float currentLineLength = FMath::Min(distance, 300.f);
        FVector2D currentEndPoint = Agent.GetPosition() + (dirToCurrent * currentLineLength);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(currentEndPoint, 0), 
            FColor::Yellow, 
            false, 
            -1.f, 
            0, 
            1.f);
        
        FVector2D toPredicted = predictedPosition - Agent.GetPosition();
        FVector2D dirToPredicted = toPredicted;
        dirToPredicted.Normalize();
        float predictedLineLength = FMath::Min(toPredicted.Length(), 400.f);
        FVector2D predictedEndPoint = Agent.GetPosition() + (dirToPredicted * predictedLineLength);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(predictedEndPoint, 0), 
            FColor::Green, 
            false, 
            -1.f, 
            0, 
            2.f);
        
        DrawDebugSphere(Agent.GetWorld(), 
            FVector(predictedPosition, 0), 
            20.f, 
            8, 
            FColor::Green);
    }
    
    return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    FVector2D toTarget = Target.Position - Agent.GetPosition();
    float distance = toTarget.Length();
    
    float agentSpeed = Agent.GetMaxLinearSpeed();
    float timeToReach = (agentSpeed > 0.f) ? distance / agentSpeed : 0.f;
    
    FVector2D predictedPosition = Target.Position + (Target.LinearVelocity * timeToReach);
    
    Steering.LinearVelocity = -(predictedPosition - Agent.GetPosition());
    
    if (Agent.GetDebugRenderingEnabled())
    {
        FVector2D toPredicted = predictedPosition - Agent.GetPosition();
        FVector2D dirToPredicted = toPredicted;
        dirToPredicted.Normalize();
        float lineLength = FMath::Min(toPredicted.Length(), 400.f);
        FVector2D endPoint = Agent.GetPosition() + (dirToPredicted * lineLength);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(endPoint, 0), 
            FColor::Orange, 
            false, 
            -1.f, 
            0, 
            1.f);
        
        FVector2D fleeDir = -(toPredicted);
        fleeDir.Normalize();
        FVector2D fleeEndPoint = Agent.GetPosition() + (fleeDir * 250.f);
        
        DrawDebugLine(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            FVector(fleeEndPoint, 0),
            FColor::Red,
            false,
            -1.f,
            0,
            2.f);
        
        DrawDebugSphere(Agent.GetWorld(), 
            FVector(predictedPosition, 0), 
            20.f, 
            8, 
            FColor::Red);
    }
    
    return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering{};
    
    float agentAngle = FMath::DegreesToRadians(Agent.GetRotation());
    FVector2D agentForward(FMath::Cos(agentAngle), FMath::Sin(agentAngle));
    
    FVector2D circleCenter = Agent.GetPosition() + (agentForward * m_OffsetDistance);
    
    float angleChange = FMath::RandRange(-m_MaxAngleChange, m_MaxAngleChange);
    m_WanderAngle += angleChange;
    
    FVector2D circleOffset(
        FMath::Cos(m_WanderAngle) * m_Radius,
        FMath::Sin(m_WanderAngle) * m_Radius
    );
    
    FVector2D wanderTarget = circleCenter + circleOffset;
    
    FTargetData wanderTargetData;
    wanderTargetData.Position = wanderTarget;
    SetTarget(wanderTargetData);
    
    Steering = Seek::CalculateSteering(DeltaT, Agent);
    
    if (Agent.GetDebugRenderingEnabled())
    {
        DrawDebugCircle(Agent.GetWorld(), 
            FVector(circleCenter, 0), 
            m_Radius, 
            32, 
            FColor::Cyan, 
            false, 
            -1.f, 
            0, 
            1.f, 
            FVector(0, 1, 0), 
            FVector(1, 0, 0));
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(circleCenter, 0), 
            FColor::Cyan, 
            false, 
            -1.f, 
            0, 
            1.f);
        
        DrawDebugSphere(Agent.GetWorld(), 
            FVector(wanderTarget, 0), 
            15.f, 
            8, 
            FColor::Magenta);
        
        DrawDebugLine(Agent.GetWorld(), 
            FVector(Agent.GetPosition(), 0), 
            FVector(wanderTarget, 0), 
            FColor::Magenta, 
            false, 
            -1.f, 
            0, 
            2.f);
    }
    
    return Steering;
}