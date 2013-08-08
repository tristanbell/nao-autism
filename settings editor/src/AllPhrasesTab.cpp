#include <AllPhrasesTab.h>

const QString AllPhrasesTab::TAB_NAME = "Phrases";

void AllPhrasesTab::init()
{
	_generalPhraseTab = new GenericPhraseTab("General phrases");
	_guessGamePhraseTab = new GenericPhraseTab("Guess game phrases");
	_mimicGamePhraseTab = new GenericPhraseTab("Mimic game phrases");

	addTab(_generalPhraseTab, _generalPhraseTab->getTabName());
	addTab(_guessGamePhraseTab, _guessGamePhraseTab->getTabName());
	addTab(_mimicGamePhraseTab, _mimicGamePhraseTab->getTabName());
}

GenericPhraseTab* AllPhrasesTab::getGeneralPhraseTab()
{
	return _generalPhraseTab;
}

GenericPhraseTab* AllPhrasesTab::getGuessGamePhraseTab()
{
	return _guessGamePhraseTab;
}

GenericPhraseTab* AllPhrasesTab::getMimicGamePhraseTab()
{
	return _mimicGamePhraseTab;
}
