/*
 * PhrasesWidget.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef PHRASESWIDGET_H_
#define PHRASESWIDGET_H_

#include <AddBehaviorDialog.h>
#include <TextInputDialog.h>
#include <PhraseGroup.h>

#include <QTabWidget>
#include <QString>
#include <QComboBox>
#include <QListWidget>
#include <QPushButton>
#include <list>

#include <QWidget>

class PhrasesWidget : public QWidget
{

	Q_OBJECT

public:
	PhrasesWidget() : QWidget()
	{
		init();
	}

	void setPhraseGroup(std::map<QString, PhraseGroupData>&);

private:
	std::map<QString, PhraseGroupData> _phraseGroups;

//	AddBehaviorDialog* _addBehaviorDialog;
	TextInputDialog* _addPhraseDialog;
	TextInputDialog* _addBehaviorDialog;

	QComboBox* _phraseGroupBox;

	QListWidget* _phrasesList;
	QPushButton* _addPhraseBtn;
	QPushButton* _removePhraseBtn;

	QListWidget* _behaviorList;
	QPushButton* _addBehaviorBtn;
	QPushButton* _removeBehaviorBtn;

	void init();

private slots:
	void addPhraseButtonClicked();
	void removePhraseButtonClicked();

	void addBehaviorButtonClicked();
	void removeBehaviorButtonClicked();
};

#endif /* PHRASESWIDGET_H_ */
