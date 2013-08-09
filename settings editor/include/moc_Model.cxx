/****************************************************************************
** Meta object code from reading C++ file 'Model.h'
**
** Created: Fri Aug 9 10:57:03 2013
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
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      17,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,    7,    6,    6, 0x05,
      51,   44,    6,    6, 0x05,
      81,    7,    6,    6, 0x05,
     109,   44,    6,    6, 0x05,
     139,    6,    6,    6, 0x05,
     158,    6,    6,    6, 0x05,
     182,    6,    6,    6, 0x05,
     208,    6,    6,    6, 0x05,
     247,  234,    6,    6, 0x05,
     292,    6,    6,    6, 0x05,
     344,  328,    6,    6, 0x05,
     390,    6,    6,    6, 0x05,
     454,    6,    6,    6, 0x05,
     520,    6,    6,    6, 0x05,
     586,    6,    6,    6, 0x05,
     631,    6,    6,    6, 0x05,
     678,    6,    6,    6, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Model[] = {
    "Model\0\0location\0successfulSave(std::string)\0"
    "reason\0unsuccessfulSave(std::string)\0"
    "successfulOpen(std::string)\0"
    "unsuccessfulOpen(std::string)\0"
    "behaviorsCleared()\0generalPhrasesCleared()\0"
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
        case 0: _t->successfulSave((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->unsuccessfulSave((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 2: _t->successfulOpen((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 3: _t->unsuccessfulOpen((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 4: _t->behaviorsCleared(); break;
        case 5: _t->generalPhrasesCleared(); break;
        case 6: _t->guessGamePhrasesCleared(); break;
        case 7: _t->mimicGamePhrasesCleared(); break;
        case 8: _t->gameBehaviorsLoaded((*reinterpret_cast< const std::list<BehaviorData>(*)>(_a[1]))); break;
        case 9: _t->gameBehaviorRetrieved((*reinterpret_cast< const BehaviorData(*)>(_a[1]))); break;
        case 10: _t->rewardBehaviorsLoaded((*reinterpret_cast< const std::list<std::string>(*)>(_a[1]))); break;
        case 11: _t->generalPhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 12: _t->guessGamePhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 13: _t->mimicGamePhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 14: _t->generalPhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        case 15: _t->guessGamePhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        case 16: _t->mimicGamePhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
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
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void Model::successfulSave(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Model::unsuccessfulSave(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Model::successfulOpen(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Model::unsuccessfulOpen(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Model::behaviorsCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void Model::generalPhrasesCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 5, 0);
}

// SIGNAL 6
void Model::guessGamePhrasesCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}

// SIGNAL 7
void Model::mimicGamePhrasesCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 7, 0);
}

// SIGNAL 8
void Model::gameBehaviorsLoaded(const std::list<BehaviorData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void Model::gameBehaviorRetrieved(const BehaviorData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 9, _a);
}

// SIGNAL 10
void Model::rewardBehaviorsLoaded(const std::list<std::string> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void Model::generalPhraseGroupLoaded(const std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void Model::guessGamePhraseGroupLoaded(const std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void Model::mimicGamePhraseGroupLoaded(const std::map<std::string,PhraseGroupData> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void Model::generalPhraseGroupRetrieved(const PhraseGroupData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 14, _a);
}

// SIGNAL 15
void Model::guessGamePhraseGroupRetrieved(const PhraseGroupData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 15, _a);
}

// SIGNAL 16
void Model::mimicGamePhraseGroupRetrieved(const PhraseGroupData & _t1)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(const_cast< Model *>(this), &staticMetaObject, 16, _a);
}
QT_END_MOC_NAMESPACE
