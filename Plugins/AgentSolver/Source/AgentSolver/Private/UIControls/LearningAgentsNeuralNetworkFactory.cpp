#include "UIControls/LearningAgentsNeuralNetworkFactory.h"

#if WITH_EDITOR

#include "LearningAgentsNeuralNetwork.h"

ULearningAgentsNeuralNetworkFactory::ULearningAgentsNeuralNetworkFactory()
{
	SupportedClass = ULearningAgentsNeuralNetwork::StaticClass();
	bCreateNew = true;
	bEditAfterNew = true;
}

UObject* ULearningAgentsNeuralNetworkFactory::FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	return NewObject<ULearningAgentsNeuralNetwork>(InParent, Class, Name, Flags);
}

#endif // WITH_EDITOR
