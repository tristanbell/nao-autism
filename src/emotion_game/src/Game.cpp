#include <Game.h>

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

#define START_NAO_SPEECH_RECOGNITION_NAME "nao_speech/start_recognition"
#define STOP_NAO_SPEECH_RECOGNITION_NAME "nao_speech/stop_recognition"
#define START_PS_SPEECH_RECOGNITION_NAME "recognizer/start"
#define STOP_PS_SPEECH_RECOGNITION_NAME "recognizer/stop"

const Phrase Game::NULL_PHRASE = Phrase("");

void Game::introduction(void)
{
	//Perform start and instruction phrases/actions, etc
	std::vector<Phrase> phraseVector;
	if (getGameSettings().getPhraseVector(START_KEY, phraseVector)){
		const Phrase& phrase = sayAny(phraseVector);

		if (phrase.getNumberOfBehaviors() != 0) {
			std::string behavior = phrase.getRandomBehaviorName();

			if (!_naoControl.perform(behavior))
				ROS_ERROR("Could not perform behavior %s", behavior.c_str());
		}
	}
	sleep(_settings.getWait());

	if (getGameSettings().getPhraseVector(INSTRUCTION_KEY, phraseVector)){
		const Phrase& phrase = sayAny(phraseVector);

		if (phrase.getNumberOfBehaviors() != 0){
			std::string behavior = phrase.getRandomBehaviorName();

			if (!_naoControl.perform(behavior))
				ROS_ERROR("Could not perform behavior %s", behavior.c_str());
		}
	}
	sleep(_settings.getWait());
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
//	_naoControl.perform("init");

	if (_performedBehavior != NULL)
		delete _performedBehavior;

	_performedBehavior = new Behavior(ref);
}

/**
 * Asks user if they want to continue playing, starts speech recognition
 * and changes state to waiting for an answer.
 */
void Game::askToContinue(void)
{
	std::vector<Phrase> phrases;
	if (_settings.getPhraseVector(CONTINUE_GAME_QUESTION_KEY, phrases)){
		const Phrase& current = sayAny(phrases);

		if (current.getNumberOfBehaviors() == 0){
			std::string behavior = current.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}
	sleep(_settings.getWait());

	_currentState = START_SPEECH_RECOGNITION;
	_stateBuffer = WAITING_ANSWER_CONTINUE;

	sleep(_settings.getWait());
}

/**
 * Waits until yes/no is 'heard'. If yes, change state to perform emotion.
 * If no, signal that game is done.
 */
Game::Response Game::waitToContinue(void)
{
	std::list<std::string>::iterator it = _recognizedWords.begin();

	while (it != _recognizedWords.end()){
		std::string word = *it;

		if (word == "yes"){
			return POSITIVE;
		}else if (word == "no"){
			return NEGATIVE;
		}

		it++;
	}

	return NONE;
}

void Game::endGameSpeech()
{
	std::vector<Phrase> phraseVector;
	if (_settings.getPhraseVector(FINISH_KEY, phraseVector)){
		const Phrase& phrase = sayAny(phraseVector);

		if (phrase.getNumberOfBehaviors() == 0){
			std::string behavior = phrase.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}
}

bool Game::startSpeechRecognition()
{
	std::cout << "Attempting to start speech recognition.\n";

	bool naoSpeechRecognition = false;
	if (_startNaoSpeechService.exists() && _startNaoSpeechService.isValid()){
		std_srvs::Empty emptySrv;

		naoSpeechRecognition = _startNaoSpeechService.call(emptySrv);
	}

	bool psSpeechRecognition = false;
	if (_startPsSpeechService.exists() && _startPsSpeechService.isValid()){
		std_srvs::Empty emptySrv;

		psSpeechRecognition = _startPsSpeechService.call(emptySrv);
	}

	_recognizedWords.clear();

	return naoSpeechRecognition || psSpeechRecognition;
}

bool Game::stopSpeechRecognition()
{
	std::cout << "Attempting to stop speech recognition.\n";

	bool naoSpeechRecognition = false;
	if (_startNaoSpeechService.exists() && _startNaoSpeechService.isValid()){
		std_srvs::Empty emptySrv;

		naoSpeechRecognition = _stopNaoSpeechService.call(emptySrv);
	}

	bool psSpeechRecognition = false;
	if (_startPsSpeechService.exists() && _startPsSpeechService.isValid()){
		std_srvs::Empty emptySrv;

		psSpeechRecognition = _stopPsSpeechService.call(emptySrv);
	}

	_recognizedWords.clear();

	return naoSpeechRecognition || psSpeechRecognition;
}

const Phrase& Game::sayRandParts(const Phrase& phrase, std::list<std::string> parts)
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

		return phrase;
	}

	return NULL_PHRASE;
}

void Game::onSpeechRecognized(const nao_msgs::WordRecognized msg)
{
	//Check to see if speech is needed, if so push onto list
	if (!isDone && (_currentState == WAITING_ANSWER_CONTINUE || _currentState == WAITING_ANSWER_QUESTION)){
		for (int i=0;i<msg.words.size();i++){
			std::cout << "Recognized word: " << msg.words[i] << ", confidence: " << msg.confidence_values[i] << "\n";

			if (msg.confidence_values[i] >= _settings.getConfidenceThreshold())
				_recognizedWords.push_back(msg.words[i]);
		}
	}
}

void Game::onSpeech(const std_msgs::String msg) {
	//Check to see if speech is needed, if so push onto list
	if (!isDone && (_currentState == WAITING_ANSWER_CONTINUE || _currentState == WAITING_ANSWER_QUESTION)) {
		// Split all recognised words into a vector of strings
		std::vector<std::string> words;
		boost::split(words, msg.data, boost::is_any_of(" "));

		for (int i = 0; i < words.size(); i++) {
			std::cout << "Recognized word: " << words[i] << "\n";
			_recognizedWords.push_back(words[i]);
		}
	}
}

const Phrase& Game::say(const Phrase& phrase)
{
	_naoControl.say(phrase.getPhrase());
	return phrase;
}

const Phrase& Game::say(const Phrase& phrase, const std::list<std::string> parts)
{
	if (parts.size() >= phrase.amountOfParts()){
		_naoControl.say(phrase.getPhrase(parts));
	}

	return phrase;
}

const Phrase& Game::sayRandParts(const Phrase& phrase, const std::string& required, std::list<std::string> parts)
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
		return phrase;
	}

	return NULL_PHRASE;
}

const Phrase& Game::sayAny(const std::vector<Phrase>& phraseVector)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];
	if (actual.amountOfParts() == 0){
		_naoControl.say(actual.getPhrase());

		return actual;
	}

	return NULL_PHRASE;
}

const Phrase& Game::sayAny(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];

	if (actual.amountOfParts() <= parts.size()){
		_naoControl.say(actual.getPhrase(parts));

		return actual;
	}

	return NULL_PHRASE;
}

const Phrase& Game::sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];

	if (actual.amountOfParts() <= parts.size())
		return sayRandParts(actual, parts);

	return NULL_PHRASE;
}

const Phrase& Game::sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::string& required, const std::list<std::string>& parts)
{
	int index = rand() % phraseVector.size();

	const Phrase& actual = phraseVector[index];

	if (actual.amountOfParts() <= parts.size())
		return sayRandParts(actual, required, parts);

	return NULL_PHRASE;
}
