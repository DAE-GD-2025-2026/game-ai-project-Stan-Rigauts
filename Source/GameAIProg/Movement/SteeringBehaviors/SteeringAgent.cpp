// Fill out your copyright notice in the Description page of Project Settings.

#include "SteeringAgent.h"
#include "AIController.h"

// Sets default values
ASteeringAgent::ASteeringAgent()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASteeringAgent::BeginPlay()
{
	Super::BeginPlay();
}

void ASteeringAgent::BeginDestroy()
{
	Super::BeginDestroy();
}

void ASteeringAgent::Tick(float DeltaTime)
{
	if (!SteeringBehavior) return;
	SteeringOutput output = SteeringBehavior->CalculateSteering(DeltaTime, *this);
	if (output.IsValid)
	{
		AddMovementInput(FVector(output.LinearVelocity, 0.f));

		if (AAIController* AIController = Cast<AAIController>(GetController()))
		{
			float const DeltaYaw = FMath::Clamp(output.AngularVelocity, -1, 1) * GetMaxAngularSpeed() * DeltaTime;

			FRotator CurrentRot{GetActorForwardVector().ToOrientationRotator()};
			FRotator DeltaRot{0,DeltaYaw,0};
			FRotator DesiredRotation{CurrentRot+DeltaRot};

			if (!FMath::IsNearlyEqual(CurrentRot.Yaw, DesiredRotation.Yaw))
			{
				AIController->SetControlRotation(DesiredRotation);
				FaceRotation(DesiredRotation);
			}
		}
	}
}

// Called to bind functionality to input
void ASteeringAgent::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASteeringAgent::SetSteeringBehavior(ISteeringBehavior* NewSteeringBehavior)
{
	SteeringBehavior = NewSteeringBehavior;
}

