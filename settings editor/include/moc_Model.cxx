/****************************************************************************
** Meta object code from reading C++ file 'Model.h'
**
** Created: Wed Aug 7 17:52:50 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Model.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Model.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Model[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,
      26,    6,    6,    6, 0x05,
      50,    6,    6,    6, 0x05,
      76,    6,    6,    6, 0x05,
     115,  102,    6,    6, 0x05,
     172,  156,    6,    6, 0x05,
     218,    6,    6,    6, 0x05,
     283,    6,    6,    6, 0x05,
     350,    6,    6,    6, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Model[] = {
    "Model\0\0behaviorsCleared()\0"
    "generalPhrasesCleared()\0"
    "guessGamePhrasesCleared()\0"
    "mimicGamePhrasesCleared()\0behaviorData\0"
    "behaviorsLoaded(std::list<BehaviorData>)\0"
    "rewardBehaviors\0"
    "rewardBehaviorsLoaded(std::list<std::string>)\0"
    "generalPhraseGroupLoaded(std::map<std::string,PhraseGroupData>&)\0"
    "guessGamePhraseGroupLoaded(std::map<std::string,PhraseGroupData>&)\0"
    "mimicGamePhraseGroupLoaded(std::map<std::string,PhraseGroupData>&)\0"
};

void Model::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Model *_t = static_cast<Model *>(_o);
        switch (_id) {
        case 0: _t->behaviorsCleared(); break;
        case 1: _t->generalPhrasesCleared(); break;
        case 2: _t->guessGamePhrasesCleared(); break;
        case 3: _t->mimicGamePhrasesCleared(); break;
        case 4: _t->behaviorsLoaded((*reinterpret_cast< std::list<BehaviorData>(*)>(_a[1]))); break;
        case 5: _t->rewardBehaviorsLoaded((*reinterpret_cast< std::list<std::string>(*)>(_a[1]))); break;
        case 6: _t->generalPhraseGroupLoaded((*reinterpret_cast< std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 7: _t->guessGamePhraseGroupLoaded((*reinterpret_cast< std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 8: _t->mimicGamePhraseGroupLoaded((*reinterpret_cast< std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Model::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Model::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Model,
      qt_meta_data_Model, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Model::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Model::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Model::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Model))
        return static_cast<void*>(const_cast< Model*>(this));
    return QObject::qt_metacast(_clname);
}

int Model::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void Model::behaviorsCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Model::generalPhrasesCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void Model::guessGamePhrasesCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void Model::mimicGamePhrasesCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void Model::behaviorsLoaded(std::list<BehaviorData> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Model::rewardBehaviorsLoaded(std::list<std::string> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Model::generalPhraseGroupLoaded(std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Model::guessGamePhraseGroupLoaded(std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Model::mimicGamePhraseGroupLoaded(std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_END_MOC_NAMESPACE
