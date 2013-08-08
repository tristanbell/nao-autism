/****************************************************************************
** Meta object code from reading C++ file 'PhraseTab.h'
**
** Created: Thu Aug 8 13:15:29 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "PhraseTab.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PhraseTab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PhraseTab[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   11,   10,   10, 0x05,
      61,   50,   10,   10, 0x05,
     104,   50,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
     155,   50,   10,   10, 0x0a,
     196,   50,   10,   10, 0x0a,
     245,   10,   10,   10, 0x0a,
     309,  304,   10,   10, 0x0a,
     347,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PhraseTab[] = {
    "PhraseTab\0\0key\0onPhraseGroupRequired(std::string)\0"
    "key,phrase\0onPhraseCreated(std::string&,std::string&)\0"
    "onPhraseBehaviorCreated(std::string&,std::string&)\0"
    "phraseCreated(std::string&,std::string&)\0"
    "phraseBehaviorCreated(std::string&,std::string&)\0"
    "onPhraseGroupLoaded(std::map<std::string,PhraseGroupData>)\0"
    "text\0onPhraseGroupBoxIndexChanged(QString)\0"
    "onPhraseGroupRetrieved(PhraseGroupData)\0"
};

void PhraseTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PhraseTab *_t = static_cast<PhraseTab *>(_o);
        switch (_id) {
        case 0: _t->onPhraseGroupRequired((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->onPhraseCreated((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 2: _t->onPhraseBehaviorCreated((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 3: _t->phraseCreated((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 4: _t->phraseBehaviorCreated((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 5: _t->onPhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 6: _t->onPhraseGroupBoxIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->onPhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PhraseTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PhraseTab::staticMetaObject = {
    { &QTabWidget::staticMetaObject, qt_meta_stringdata_PhraseTab,
      qt_meta_data_PhraseTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PhraseTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PhraseTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PhraseTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PhraseTab))
        return static_cast<void*>(const_cast< PhraseTab*>(this));
    return QTabWidget::qt_metacast(_clname);
}

int PhraseTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTabWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void PhraseTab::onPhraseGroupRequired(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PhraseTab::onPhraseCreated(std::string & _t1, std::string & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PhraseTab::onPhraseBehaviorCreated(std::string & _t1, std::string & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
