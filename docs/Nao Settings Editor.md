# Emotion Game Settings Editor

## Introduction
To run the emotion game, a json file is required which contains the settings and phrases for both the guessing and the mimic game. This file will contain brief documentation on getting started and using the settings editor application.

## Starting the application

In order to run the application you must double click the run\_settings\_editor.sh file in the main nao-autism directory. When the application opens, you should see a screen like so:

<img src="resources/Settings Editor First View.png"/>

## Getting started

Once the application is opened, you will notice that you are unable to interact with anything as it will all be 'greyed' out. This is because either a new settings file must be created or an existing one opened.

If you want to create a new settings file you must do the following:

1. Press the 'File' menu button in the top-left corner of the screen. If you can't see it, then hover your mouse over the 'Settings Editor' label.
2. Click 'New' on the menu that pops up, this will create the settings file with the default values and populate the application with the new data.

If you want to open an existing settings file you must do the following:

1. Press the 'File' menu button in the top-left corner of the screen. If you can't see it, then hover your mouse over the 'Settings Editor' label.
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

Once the value has been changed, there is no further need to press any buttons. The value will automatically replace the old value and once the file is saved the new value will be saved too.

#### Value meanings

###### Wait after speech

This value changes the amount of time the Nao will wait after it says something. This is measured in seconds. So, for example, if the value was changed to '5' then the Nao will wait 5 seconds after it says something before continueing.

###### Wait before question timeout

This value changes the amount of time the Nao will wait before asking the question again. If the Nao has already asked the question enough (this is defined as 'Tries before failure', which will be explained next) then the Nao will assume the child got the question wrong and it will continue with the next emotion. If the value '10' is provided then the Nao will wait 10 seconds before asking the question again, assuming it hasn't already recieved the correct answer.

###### Tries before failure

This value changes the amount of 'timeouts' the Nao allows before assuming that the question is incorrect. For example, if the value '3' is provided then the Nao will ask the child the same question 3 times before assuming that the child is wrong.

###### Number of emotions before ask to continue question

This value changes the amount of 'emotions' the Nao will perform in the game before asking the child if they want to continue playing that specific game. For example, if the value '5' is used then the Nao will go through 5 different emotions ('rounds') before asking the child if they want to continue playing.

###### Speech recognition confidence

This value changes the confidence in the results from the Nao's speech recognition software. Each time a 'word' is heard by the Nao it assigns a rating to it. This rating describes how confident it is that the word was the actual word spoken, the value supplied for speech recognition confidence in the settings editor will filter anything that has a rating below the value given. For example, if the value '50' (%) is given then any word that has a rating below 50% will be discarded.

## Behaviors

The Behaviors tab looks like this:

<img src="resources/Behaviors Tab View.png"/>

This tab is used to edit the data for the behaviors. It is a bit more complex then the Base settings tab. There are more tabs present, as these edit different data. Below is an explaination of each tab and how to use it.

###### Game behaviors tab

This tab is for editing behaviors that relate to the games themselves. There are 4 behavior names to choose from the drop-down box:

<img src="resources/Behaviors Dropdown.png"/>

When you select a behavior, the list below will update to reflect all the possible behaviors that may be performed by the Nao, like so:

List before 'scared' behavior has been selected:

<img src="resources/Behaviors List1.png"/>

List after 'scared' behavior has been selected:

<img src="resources/Behaviors List2.png"/>

By default, there will only be one behavior in each list. However, when more are added the Nao will randomly select a behavior out of the list to perform when it attempts to perform the selected behavior.

_Adding behaviors_

In order to add new behaviors, you must click the 'Add behavior button'. This will open a seperate dialog like so:

<img src="resources/Behaviors BehaviorDialog.png"/>

Simple type the behavior name (as it appears in the 'Choregraphe' application) and click the submit button. If successful the following should appear:

<img src="resources/Behaviors Success.png"/>

If nothing was entered or the action was cancelled then one of the following should appear:

<img src="resources/Behaviors Nothing.png"/>
<img src="resources/Behaviors Cancel.png"/>

_Editing behaviors_

If you wish to edit a behavior then you simply click on the behavior you wish to edit in the list, like so:

<img src="resources/Behaviors Click.png"/>

This will now enable the 'Edit behaviors' and 'Remove behaviors' button. Press the 'Edit behavior' button. This will open the same dialog as the 'Add behavior' button, except this time the original name will be present in the text box. Simply press submit when the name has been edited and the changes should now appear in the list.

_Removing behaviors_

If you wish to remove a behavior then, again, simple click on the behavior you wish to delete in the list. Once it has been highlighted (see image above) press the 'Remove behavior' button. When this button has been clicked, the following dialog should appear:

<img src="resources/Behaviors RemoveConfirm.png"/>

If yes is clicked then the behavior will be removed from the list and the data. If no is clicked then the behavior will not be removed and will remain in the list. No additional dialogs will appear.

###### Reward behaviors tab

This tab is for editing behaviors that relate to the rewards the children recieve when a game has ended. This tab should look like the following:

<img src="resources/Reward Behaviors View.png"/>

When a reward is given to the child, the Nao will randomly select a behavior from this list to perform. By default, there should be 3.

The process of adding/editing/removing the behaviors is the same as in the previous section, so refer to that for specific instructions.

## Phrases

TODO

## Finishing up

TODO
