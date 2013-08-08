/****************************************************************************
** Meta object code from reading C++ file 'GameBehaviorsTab.h'
**
** Created: Thu Aug 8 14:54:10 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "GameBehaviorsTab.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GameBehaviorsTab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GameBehaviorsTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      35,   18,   17,   17, 0x05,
      83,   78,   17,   17, 0x05,

 // slots: signature, parameters, type, tag, flags
     117,   17,   17,   17, 0x0a,
     152,   17,   17,   17, 0x0a,
     181,   17,   17,   17, 0x0a,
     227,   17,   17,   17, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GameBehaviorsTab[] = {
    "GameBehaviorsTab\0\0key,behaviorName\0"
    "onBehaviorCreated(std::string,std::string)\0"
    "name\0behaviorDataRequired(std::string)\0"
    "onBehaviorBoxIndexChanged(QString)\0"
    "onCreateBehaviorBtnClicked()\0"
    "onBehaviorListLoaded(std::list<BehaviorData>)\0"
    "onBehaviorDataRetrieved(BehaviorData)\0"
};

void GameBehaviorsTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GameBehaviorsTab *_t = static_cast<GameBehaviorsTab *>(_o);
        switch (_id) {
        case 0: _t->onBehaviorCreated((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const std::string(*)>(_a[2]))); break;
        case 1: _t->behaviorDataRequired((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 2: _t->onBehaviorBoxIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->onCreateBehaviorBtnClicked(); break;
        case 4: _t->onBehaviorListLoaded((*reinterpret_cast< const std::list<BehaviorData>(*)>(_a[1]))); break;
        case 5: _t->onBehaviorDataRetrieved((*reinterpret_cast< const BehaviorData(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GameBehaviorsTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GameBehaviorsTab::staticMetaObject = {
    { &QTabWidget::staticMetaObject, qt_meta_stringdata_GameBehaviorsTab,
      qt_meta_data_GameBehaviorsTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GameBehaviorsTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GameBehaviorsTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GameBehaviorsTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GameBehaviorsTab))
        return static_cast<void*>(const_cast< GameBehaviorsTab*>(this));
    return QTabWidget::qt_metacast(_clname);
}

int GameBehaviorsTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTabWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void GameBehaviorsTab::onBehaviorCreated(const std::string & _t1, const std::string & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GameBehaviorsTab::behaviorDataRequired(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
