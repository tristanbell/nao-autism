#include <Game.h>

void Game::introduction(void)
{
	//Perform start and instruction phrases/actions, etc
	std::vector<Phrase> phraseVector;
	if (getGameSettings().getPhraseVector(START_KEY, phraseVector))
		sayAny(phraseVector);

	if (getGameSettings().getPhraseVector(INSTRUCTION_KEY, phraseVector))
		sayAny(phraseVector);

	_currentState = PERFORM_EMOTION;
}

void Game::performEmotion(void)
{
	const std::vector<Behavior>& behaviorVector = _settings.getBehaviorVector();

	//Find a new random behavior to perform
	int index = 0;
	while (true){
		index = rand() % behaviorVector.size();
		const Behavior& ref = behaviorVector[index];

		if (_performedBehavior == NULL || ref.getActualName() != _performedBehavior->getActualName())
			break;
	}

	const Behavior& ref = behaviorVector[index];
	_naoControl.perform(ref.getName());

	if (_performedBehavior != NULL)
		delete _performedBehavior;

	_performedBehavior = new Behavior(ref);
}

void Game::sayRandParts(const Phrase& phrase, std::list<std::string> parts)
{
	int numParts = phrase.amountOfParts();

	if (phrase.amountOfParts() <= parts.size()){
		//Select random numParts to use as parts
		std::list<std::string> usedParts;

		while (numParts != 0){
			int rndIndex = rand() % parts.size();

			std::string val;

			int i = 0;
			std::list<std::string>::iterator it = parts.begin();
			while (it != parts.end()){
				if (i == rndIndex){
					val = *it;

					break;
				}

				i++;
				it++;
			}

			parts.remove(val);
			usedParts.push_back(val);

			numParts--;
		}

		_naoControl.say(phrase.getPhrase(usedParts));
	}
}

void Game::say(const Phrase& phrase)
{
	_naoControl.say(phrase.getPhrase());
	sleep(_settings.getWait());
}

void Game::say(const Phrase& phrase, const std::list<std::string> parts)
{
	if (parts.size() >= phrase.amountOfParts()){
		_naoControl.say(phrase.getPhrase(parts));
		sleep(_settings.getWait());
	}
}

void Game::sayRandParts(const Phrase& phrase, const std::string& required, std::list<std::string> parts)
{
	int numParts = phrase.amountOfParts();

	if (phrase.amountOfParts() <= parts.size()){
		int requiredIndex = rand() % numParts;

		//Select random numParts to use as parts
		std::list<std::string> usedParts;

		while (numParts != 0){
			// Index for the required string has been reached, insert
			if (phrase.amountOfParts() - numParts == requiredIndex){
				usedParts.push_back(required);
				numParts--;

				continue;
			}

			int rndIndex = rand() % parts.size();

			std::string val;

			int i = 0;
			std::list<std::string>::iterator it = parts.begin();
			while (it != parts.end()){
				if (i == rndIndex){
					val = *it;

					break;
				}

				i++;
				it++;
			}

			parts.remove(val);
			usedParts.push_back(val);

			numParts--;
		}

		_naoControl.say(phrase.getPhrase(usedParts));
		sleep(_settings.getWait());
	}
}

void Game::sayAny(const std::vector<Phrase>& phraseVector)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];
	if (actual.amountOfParts() == 0){
		_naoControl.say(actual.getPhrase());
		sleep(_settings.getWait());
	}
}

void Game::sayAny(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];

	if (actual.amountOfParts() <= parts.size()){
		_naoControl.say(actual.getPhrase(parts));
		sleep(_settings.getWait());
	}
}

void Game::sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];

	if (actual.amountOfParts() <= parts.size())
		sayRandParts(actual, parts);
}

void Game::sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::string& required, const std::list<std::string>& parts)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];

	if (actual.amountOfParts() <= parts.size())
		sayRandParts(actual, required, parts);
}
