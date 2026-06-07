#pragma once
#include "CoreMinimal.h"
#include "FSMComponent.h"
#include "Movement/SteeringBehaviors/PathFollow/PathFollowSteeringBehavior.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

namespace GameAI::FSM
{
    class PatrolState : public State
    {
    public:
        PatrolState(ASteeringAgent* Cop)
            : CopAgent(Cop)
        {
            // define patrol waypoints
            PatrolPath = {
                FVector2D(150,  150),
                FVector2D(-150,  150),
                FVector2D(-150, -150),
                FVector2D(150, -150)
            };
        }

        virtual void OnEnter() override
        {
            UE_LOG(LogTemp, Warning, TEXT("Patrol: Enter"));
            if (!CopAgent) return;
            PathFollowBehavior.SetPath(PatrolPath, true); 
            CopAgent->SetSteeringBehavior(&PathFollowBehavior);
        }

        virtual void OnExit() override { UE_LOG(LogTemp, Warning, TEXT("Patrol: Exit")); }

        virtual void Update(float DeltaTime) override {}

    private:
        ASteeringAgent* CopAgent{ nullptr };
        PathFollow PathFollowBehavior{};
        std::vector<FVector2D> PatrolPath;
    };

    class ChaseState : public State
    {
    public:
        ChaseState(ASteeringAgent* Cop, ASteeringAgent* Thief)
            : CopAgent(Cop), ThiefAgent(Thief)
        {
            // nothing extra needed
        }

        virtual void OnEnter() override
        {
            UE_LOG(LogTemp, Warning, TEXT("Chase: Enter"));
            if (!CopAgent) return;
            CopAgent->SetSteeringBehavior(&SeekBehavior);
        }

        virtual void OnExit() override { UE_LOG(LogTemp, Warning, TEXT("Chase: Exit")); }

        virtual void Update(float DeltaTime) override
        {
            if (!ThiefAgent) return;

            // update seek target to thief every tick
            FSteeringParams Target;
            FVector Pos = ThiefAgent->GetActorLocation();
            Target.Position = FVector2D(Pos.X, Pos.Y);
            SeekBehavior.SetTarget(Target);
        }

    private:
        ASteeringAgent* CopAgent{ nullptr };
        ASteeringAgent* ThiefAgent{ nullptr };
        Seek SeekBehavior{};
    };

    class SearchState : public State
    {
    public:
        SearchState(ASteeringAgent* Cop)
            : CopAgent(Cop) {
        }

        virtual void OnEnter() override
        {
            UE_LOG(LogTemp, Warning, TEXT("Search: Enter"));
            SearchTimer = 0.f;
            if (!CopAgent) return;

            // seek around last known position
            FSteeringParams Target;
            FVector Pos = CopAgent->GetActorLocation();
            Target.Position = FVector2D(Pos.X + 200.f, Pos.Y);
            SeekBehavior.SetTarget(Target);
            CopAgent->SetSteeringBehavior(&SeekBehavior);
        }

        virtual void OnExit() override { UE_LOG(LogTemp, Warning, TEXT("Search: Exit")); }

        virtual void Update(float DeltaTime) override
        {
            SearchTimer += DeltaTime;
        }

        float SearchTimer{ 0.f };
        float SearchTooLongThreshold{ 5.f };

    private:
        ASteeringAgent* CopAgent{ nullptr };
        Seek SeekBehavior{};
    };
}