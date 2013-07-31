#include <Behavior.h>

Behavior::Behavior(std::string behaviorName, std::string actualBehaviorName) : name(behaviorName),
																			   actual(actualBehaviorName)
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
