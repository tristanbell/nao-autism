/****************************************************************************
** Meta object code from reading C++ file 'GuessGameTab.h'
**
** Created: Wed Aug 7 19:06:44 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "GuessGameTab.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GuessGameTab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GuessGameTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      18,   14,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      53,   13,   13,   13, 0x0a,
     117,  112,   13,   13, 0x0a,
     155,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GuessGameTab[] = {
    "GuessGameTab\0\0key\0onPhraseGroupRequired(std::string)\0"
    "onPhraseGroupLoaded(std::map<std::string,PhraseGroupData>)\0"
    "text\0onPhraseGroupBoxIndexChanged(QString)\0"
    "onPhraseGroupRetrieved(PhraseGroupData)\0"
};

void GuessGameTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GuessGameTab *_t = static_cast<GuessGameTab *>(_o);
        switch (_id) {
        case 0: _t->onPhraseGroupRequired((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->onPhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 2: _t->onPhraseGroupBoxIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->onPhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GuessGameTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GuessGameTab::staticMetaObject = {
    { &QTabWidget::staticMetaObject, qt_meta_stringdata_GuessGameTab,
      qt_meta_data_GuessGameTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GuessGameTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GuessGameTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GuessGameTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GuessGameTab))
        return static_cast<void*>(const_cast< GuessGameTab*>(this));
    return QTabWidget::qt_metacast(_clname);
}

int GuessGameTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTabWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void GuessGameTab::onPhraseGroupRequired(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
