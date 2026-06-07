#pragma once
#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Level_FSM.generated.h"

class Seek;

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
    GENERATED_BODY()
public:
    ALevel_FSM();
    virtual void Tick(float DeltaTime) override;

protected:
    virtual void BeginPlay() override;

private:
    bool IsTargetVisible() const;

    UPROPERTY()
    ASteeringAgent* Cop{ nullptr };

    UPROPERTY()
    ASteeringAgent* Thief{ nullptr };

    Seek* SeekBehavior{ nullptr };    // Thief's seek (follows mouse)
   

    float VisibilityRange{ 400.f };
};