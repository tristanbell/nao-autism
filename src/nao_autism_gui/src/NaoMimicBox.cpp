#include <nao_gui/NaoMimicBox.h>

#include <QGridLayout>

#include <QPushButton>

void nao_gui::NaoMimicBox::init()
{
	QGridLayout* layout = new QGridLayout;

	QPushButton* promptBtn = new QPushButton("Prompt to mimic");
	promptBtn->connect(promptBtn, SIGNAL(clicked()),
			this, SLOT(promptButtonPressed()));

	QPushButton* correctBtn = new QPushButton("Correct answer");
	correctBtn->connect(correctBtn, SIGNAL(clicked()),
			this, SLOT(correctButtonPressed()));

	QPushButton* incorrectBtn = new QPushButton("Incorrect answer");
	incorrectBtn->connect(incorrectBtn, SIGNAL(clicked()),
			this, SLOT(incorrectButtonPressed()));

	layout->addWidget(promptBtn, 0, 0);
	layout->addWidget(correctBtn, 2, 0);
	layout->addWidget(incorrectBtn, 3, 0);

	setLayout(layout);
}

void nao_gui::NaoMimicBox::promptButtonPressed()
{
	naoControl->say(nao_gui::MIMIC_PROMPT);
}

void nao_gui::NaoMimicBox::correctButtonPressed()
{
	naoControl->say(nao_gui::MIMIC_CORRECT_ANSWER);
}

void nao_gui::NaoMimicBox::incorrectButtonPressed()
{
	naoControl->say(nao_gui::MIMIC_INCORRECT_ANSWER);
}
