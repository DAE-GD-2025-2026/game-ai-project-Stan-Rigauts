#include "Level_FSM.h"
#include "FSMComponent.h"
#include "./States/States.h"
#include "DecisionMaking/GameAIController.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include <memory>
#include <Movement/SteeringBehaviors/SteeringHelpers.h>

ALevel_FSM::ALevel_FSM()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ALevel_FSM::BeginPlay()
{
    Super::BeginPlay();

    Cop = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 0,   0, 90 }, FRotator::ZeroRotator);
    Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 400, 0, 90 }, FRotator::ZeroRotator);
    Cop->SetDebugRenderingEnabled(false);
    Thief->SetDebugRenderingEnabled(false);

    // Thief follows mouse
    SeekBehavior = new Seek();
    Thief->SetSteeringBehavior(SeekBehavior);
    Cop->SetMaxLinearSpeed(200);
    // Cop is controlled by FSM
    if (AGameAIController* AIController = Cast<AGameAIController>(Cop->GetController()))
    {
        if (UFSMComponent* FSM = Cast<UFSMComponent>(AIController->GetBrainComponent()))
        {
            // states own their own steering behaviors now
            auto Patrol = std::make_unique<GameAI::FSM::PatrolState>(Cop);
            auto Chase = std::make_unique<GameAI::FSM::ChaseState>(Cop, Thief);
            auto Search = std::make_unique<GameAI::FSM::SearchState>(Cop);

            GameAI::FSM::PatrolState* PatrolPtr = Patrol.get();
            GameAI::FSM::ChaseState* ChasePtr = Chase.get();
            GameAI::FSM::SearchState* SearchPtr = Search.get();

            FSM->AddState(std::move(Patrol));
            FSM->AddState(std::move(Chase));
            FSM->AddState(std::move(Search));

            FSM->AddTransition(PatrolPtr, ChasePtr, [this]() { return IsTargetVisible(); });
            FSM->AddTransition(ChasePtr, SearchPtr, [this]() { return !IsTargetVisible(); });
            FSM->AddTransition(SearchPtr, ChasePtr, [this]() { return IsTargetVisible(); });
            FSM->AddTransition(SearchPtr, PatrolPtr, [SearchPtr]()
                {
                    return SearchPtr->SearchTimer >= SearchPtr->SearchTooLongThreshold;
                });

            AIController->RunFiniteStateMachine();
        }
    }
}

void ALevel_FSM::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // only thief is mouse controlled, cop is fully FSM driven
    if (SeekBehavior)
        SeekBehavior->SetTarget(MouseTarget);
}


bool ALevel_FSM::IsTargetVisible() const
{
    if (!Cop || !Thief) return false;

    float Distance = FVector::Dist(Cop->GetActorLocation(), Thief->GetActorLocation());
    return Distance <= VisibilityRange;
}