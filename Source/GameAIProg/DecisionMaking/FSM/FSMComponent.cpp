// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"
#include "FSM.h"
#include "AIController.h"


UFSMComponent::UFSMComponent()
{
	
	PrimaryComponentTick.bCanEverTick = true;

	// TODO Setup FSM
	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}

GameAI::FSM::State* UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	// TODO
	return FSMInstance->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const
{
	// TODO
	FSMInstance->AddTransition(From, To, std::move(EvalFunc));
}

void UFSMComponent::SetInitialState(GameAI::FSM::State* InitialState)
{
	FSMInstance->SetInitialState(InitialState);
}

GameAI::FSM::Blackboard& UFSMComponent::GetBlackboard()
{
	return FSMInstance->GetBlackboard();
}


void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
	FSMInstance->SetController(Cast<AAIController>(GetOwner()));
}


void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// TODO
	if (bIsRunning)
		FSMInstance->Update(DeltaTime);
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	// TODO
	bIsRunning = true;
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	// TODO
	bIsRunning = false;
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}