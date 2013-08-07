/****************************************************************************
** Meta object code from reading C++ file 'MimicGameTab.h'
**
** Created: Wed Aug 7 19:06:44 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "MimicGameTab.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MimicGameTab.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MimicGameTab[] = {

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

static const char qt_meta_stringdata_MimicGameTab[] = {
    "MimicGameTab\0\0key\0onPhraseGroupRequired(std::string)\0"
    "onPhraseGroupLoaded(std::map<std::string,PhraseGroupData>)\0"
    "text\0onPhraseGroupBoxIndexChanged(QString)\0"
    "onPhraseGroupRetrieved(PhraseGroupData)\0"
};

void MimicGameTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MimicGameTab *_t = static_cast<MimicGameTab *>(_o);
        switch (_id) {
        case 0: _t->onPhraseGroupRequired((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->onPhraseGroupLoaded((*reinterpret_cast< const std::map<std::string,PhraseGroupData>(*)>(_a[1]))); break;
        case 2: _t->onPhraseGroupBoxIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->onPhraseGroupRetrieved((*reinterpret_cast< const PhraseGroupData(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MimicGameTab::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MimicGameTab::staticMetaObject = {
    { &QTabWidget::staticMetaObject, qt_meta_stringdata_MimicGameTab,
      qt_meta_data_MimicGameTab, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MimicGameTab::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MimicGameTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MimicGameTab::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MimicGameTab))
        return static_cast<void*>(const_cast< MimicGameTab*>(this));
    return QTabWidget::qt_metacast(_clname);
}

int MimicGameTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void MimicGameTab::onPhraseGroupRequired(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
