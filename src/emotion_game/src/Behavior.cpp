#include <Behavior.h>

Behavior::Behavior(std::string behaviorName) : name(behaviorName),
												actual("\0"),
												classification(-1)
{

}

Behavior::Behavior(std::string behaviorName, std::string actualBehaviorName) : name(behaviorName),
																			   actual(actualBehaviorName),
																				classification(-1)
{

}

Behavior::Behavior(std::string behaviorName, std::string actualBehaviorName, int behaviorClassification) :
																			name(behaviorName),
																			actual(actualBehaviorName),
																			classification(behaviorClassification)
{

}

const std::string& Behavior::getName() const
{
	return name;
}

const std::string& Behavior::getActualName() const
{
	return actual;
}

int Behavior::getClassification() const
{
	return classification;
}
