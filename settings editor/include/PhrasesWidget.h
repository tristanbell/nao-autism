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
#include <PhraseGroupData.h>

#include <QTabWidget>
#include <QString>
#include <QComboBox>
#include <QListWidget>
#include <QPushButton>
#include <list>
#include <string>

#include <QWidget>

class PhrasesWidget : public QWidget
{

	Q_OBJECT

public:
	PhrasesWidget() : QWidget()
	{
		init();
	}

	void setPhraseGroup(std::map<std::string, PhraseGroupData>&);
	void setCurrentPhraseGroup(const PhraseGroupData&);

public slots:
	void phraseGroupBoxIndexChanged(const QString& text);

private:
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

	void loadIntoLists(const PhraseGroupData&);

private slots:
	void addPhraseButtonClicked();
	void removePhraseButtonClicked();

	void addBehaviorButtonClicked();
	void removeBehaviorButtonClicked();

signals:
	void currentPhraseGroupIndexChanged(const QString& text);

};

#endif /* PHRASESWIDGET_H_ */
