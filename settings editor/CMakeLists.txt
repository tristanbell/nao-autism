cmake_minimum_required(VERSION 2.8.3)

project(settings_editor)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
include_directories(include ${QT_INCLUDE_DIRS})

set(settings_editor_SOURCES
	src/main.cpp
	src/Window.cpp
	src/BaseSettingsTab.cpp
	src/BehaviorTab.cpp
	src/AllPhrasesTab.cpp
	src/GenericPhraseTab.cpp
	src/GuessGameTab.cpp
	src/MimicGameTab.cpp	
	src/AddBehaviorDialog.cpp
	src/TextInputDialog.cpp
	src/PhrasesWidget.cpp
	src/Model.cpp
	src/Controller.cpp
	src/jsoncpp.cpp
)

set(settings_editor_HEADERS
	include/Window.h
	include/BaseSettingsTab.h
	include/BehaviorTab.h
	include/AllPhrasesTab.h
	include/GenericPhraseTab.h
	include/GuessGameTab.h
	include/MimicGameTab.h
	include/AddBehaviorDialog.h
	include/TextInputDialog.h
	include/PhrasesWidget.h
	include/PhraseGroupData.h
	include/BehaviorData.h
	include/Controller.h
	include/Model.h
	include/Keys.h
	include/json/json.h
)

include_directories(
	include/
)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

qt4_wrap_cpp(settings_editor_HEADERS_MOC ${settings_editor_HEADERS})

add_executable(settings_editor ${settings_editor_SOURCES} ${settings_editor_HEADERS_MOC})
target_link_libraries(settings_editor ${QT_LIBRARIES})

install(
	TARGETS settings_editor
	DESTINATION bin
)

install(
	FILES ${settings_editor_HEADERS}
	DESTINATION include
)
