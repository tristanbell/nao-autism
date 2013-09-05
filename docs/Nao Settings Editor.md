# Emotion Game Settings Editor

## Introduction
To run the emotion game, a .json file is required which contains the settings and phrases for both the guessing and the mimic game. This file will contain brief documentation on getting started and using the settings editor application.

## Starting the application

In order to run the application you must double click the run\_settings\_editor.sh file in the main nao-autism directory. When the application opens, you should see a screen like so:

<img src="resources/Settings Editor First View.png"/>

## Getting started

Once the application is opened, you will notice that you are unable to interact with anything as it will all be 'greyed' out. This is because either a new settings file must be created or an existing one opened.

If you want to create a new settings file you must do the following:

1. Press the 'File' menu button in the top-left corner of the screen. If you can't see it, then hover your mouse over the 'Settins Editor' label.
2. Click 'New' on the menu that pops up, this will create the settings file with the default values and populate the application with the new data.

If you want to open an existing settings file you must do the following:

1. Press the 'File' menu button in the top-left corner of the screen. If you can't see it, then hover your mouse over the 'Settins Editor' label.
2. Click 'Open' on the menu that pops up, this will show an 'Open file' window, you now must locate the file you wish to edit (it will be a file with a '.sjon' extension like: 'data.json'). This will now load the file into the application and populate the application with the new data.

Now that the data is loaded into the application, you will be able to interact with the application as intended now. Each part of the application is sorted into a seperate tab in order to make the UI cleaner and easier to use. The tabs are the following:

- Base settings
- Behaviors
- Phrases

The following sections of this documentation will explain each tab and what effect the changes will have on the emotion game.

## Base settings

The base settings tab looks like so:

<img src="resources/Settings Editor Base Settings.png"/>

This tab is responsible for 'game-wide' settings that will effect both the guessing and the mimicing game. In order to change a value you must, either, press the 'up' and 'down' arrows:

<img src="resources/Settings Editor UpDown.png"/>

or type and valid number into the box:

<img src="resources/Settings Editor TypeBox.png"/>


