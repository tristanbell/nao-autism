#include <PhrasesTab.h>

const QString PhrasesTab::TAB_NAME = "Phrases";

void PhrasesTab::init()
{
	_generalPhraseTab = new GenericPhraseTab("General phrases");
	_guessGamePhraseTab = new GenericPhraseTab("Guess game phrases");
	_mimicGamePhraseTab = new GenericPhraseTab("Mimic game phrases");

	addTab(_generalPhraseTab, _generalPhraseTab->getTabName());
	addTab(_guessGamePhraseTab, _guessGamePhraseTab->getTabName());
	addTab(_mimicGamePhraseTab, _mimicGamePhraseTab->getTabName());
}

GenericPhraseTab* PhrasesTab::getGeneralPhraseTab()
{
	return _generalPhraseTab;
}

GenericPhraseTab* PhrasesTab::getGuessGamePhraseTab()
{
	return _guessGamePhraseTab;
}

GenericPhraseTab* PhrasesTab::getMimicGamePhraseTab()
{
	return _mimicGamePhraseTab;
}
