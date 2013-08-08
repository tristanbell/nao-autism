/****************************************************************************
** Meta object code from reading C++ file 'Model.h'
**
** Created: Thu Aug 8 15:43:45 2013
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
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      13,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,
      26,    6,    6,    6, 0x05,
      50,    6,    6,    6, 0x05,
      76,    6,    6,    6, 0x05,
     115,  102,    6,    6, 0x05,
     160,    6,    6,    6, 0x05,
     212,  196,    6,    6, 0x05,
     258,    6,    6,    6, 0x05,
     322,    6,    6,    6, 0x05,
     388,    6,    6,    6, 0x05,
     454,    6,    6,    6, 0x05,
     499,    6,    6,    6, 0x05,
     546,    6,    6,    6, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Model[] = {
    "Model\0\0behaviorsCleared()\0"
    "generalPhrasesCleared()\0"
    "guessGamePhrasesCleared()\0"
    "mimicGamePhrasesCleared()\0behaviorData\0"
    "gameBehaviorsLoaded(std::list<BehaviorData>)\0"
    "gameBehaviorRetrieved(BehaviorData)\0"
    "rewardBehaviors\0"
    "rewardBehaviorsLoaded(std::list<std::string>)\0"
    "generalPhraseGroupLoaded(std::map<std::string,PhraseGroupData>)\0"
    "guessGamePhraseGroupLoaded(std::map<std::string,PhraseGroupData>)\0"
    "mimicGamePhraseGroupLoaded(std::map<std::string,PhraseGroupData>)\0"
    "generalPhraseGroupRetrieved(PhraseGroupData)\0"
    "guessGamePhraseGroupRetrieved(PhraseGroupData)\0"
    "mimicGamePhraseGroupRetrieved(PhraseGroupData)\0"
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
        case 4: _t->gameBehaviorsLoaded((*reinterpret_cast< const std::list<BehaviorData>(*)>(_a[1]))); break;
        case 5: _t->gameBehaviorRetrieved((*reinterpret_cast< const BehaviorData(*)>(_a[1]))); break;
        case 6: _t->rewardBehaviorsLoaded((*reinterpret_cast< const std::list<std::string>(*)>(_a[1]))); break;
        case 7: _t->generalPhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 8: _t->guessGamePhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 9: _t->mimicGamePhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 10: _t->generalPhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        case 11: _t->guessGamePhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        case 12: _t->mimicGamePhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
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
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
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
void Model::gameBehaviorsLoaded(const std::list<BehaviorData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Model::gameBehaviorRetrieved(const BehaviorData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Model::rewardBehaviorsLoaded(const std::list<std::string> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Model::generalPhraseGroupLoaded(const std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Model::guessGamePhraseGroupLoaded(const std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void Model::mimicGamePhraseGroupLoaded(const std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void Model::generalPhraseGroupRetrieved(const PhraseGroupData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 10, _a);
}

// SIGNAL 11
void Model::guessGamePhraseGroupRetrieved(const PhraseGroupData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 11, _a);
}

// SIGNAL 12
void Model::mimicGamePhraseGroupRetrieved(const PhraseGroupData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 12, _a);
}
QT_END_MOC_NAMESPACE
