#pragma once
#include <functional>
#include <memory>
#include <vector>
#include "CoreMinimal.h"
#include "BrainComponent.h"
#include "FSMComponent.generated.h"

namespace GameAI::FSM
{
	class State
	{
	public:
		virtual ~State() = default;
		virtual void OnEnter() {}
		virtual void OnExit() {}
		virtual void Update(float DeltaTime) {}
	};

	struct Transition
	{
		State* From{ nullptr };
		State* To{ nullptr };
		std::function<bool()> EvalFunc;
	};

	class FSM
	{
	public:
		void AddState(std::unique_ptr<State>&& NewState)
		{
			States.push_back(std::move(NewState));
		}

		void AddTransition(State* From, State* To, std::function<bool()> EvalFunc)
		{
			Transitions.push_back({ From, To, EvalFunc });
		}

		void Start()
		{
			if (!States.empty())
			{
				CurrentState = States[0].get();
				CurrentState->OnEnter();
			}
		}

		void Update(float DeltaTime)
		{
			if (!CurrentState) return;

			for (auto& T : Transitions)
			{
				if (T.From == CurrentState && T.EvalFunc())
				{
					CurrentState->OnExit();
					CurrentState = T.To;
					CurrentState->OnEnter();
					break;
				}
			}

			CurrentState->Update(DeltaTime);
		}

		void Stop()
		{
			if (CurrentState)
			{
				CurrentState->OnExit();
				CurrentState = nullptr;
			}
		}

		State* GetCurrentState() const { return CurrentState; }

	private:
		std::vector<std::unique_ptr<State>> States;
		std::vector<Transition> Transitions;
		State* CurrentState{ nullptr };
	};
}

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class GAMEAIPROG_API UFSMComponent : public UBrainComponent
{
	GENERATED_BODY()
public:
	UFSMComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	virtual void StartLogic() override;
	virtual void StopLogic(const FString& Reason) override;
	virtual bool IsRunning() const override;

	void AddState(std::unique_ptr<GameAI::FSM::State>&& NewState);
	void AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const;

protected:
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::FSM::FSM> FSMInstance;
	bool bIsRunning{ false };
};