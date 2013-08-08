/****************************************************************************
** Meta object code from reading C++ file 'PhrasesWidget.h'
**
** Created: Thu Aug 8 11:33:18 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "PhrasesWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PhrasesWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PhrasesWidget[] = {

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
      20,   15,   14,   14, 0x05,
      71,   60,   14,   14, 0x05,
     132,  115,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
     183,   15,   14,   14, 0x0a,
     219,   14,   14,   14, 0x08,
     244,   14,   14,   14, 0x08,
     272,   14,   14,   14, 0x08,
     299,   14,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PhrasesWidget[] = {
    "PhrasesWidget\0\0text\0"
    "currentPhraseGroupIndexChanged(QString)\0"
    "key,phrase\0newPhraseCreated(std::string&,std::string&)\0"
    "key,behaviorName\0"
    "newPhraseBehaviorCreated(std::string&,std::string)\0"
    "phraseGroupBoxIndexChanged(QString)\0"
    "addPhraseButtonClicked()\0"
    "removePhraseButtonClicked()\0"
    "addBehaviorButtonClicked()\0"
    "removeBehaviorButtonClicked()\0"
};

void PhrasesWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PhrasesWidget *_t = static_cast<PhrasesWidget *>(_o);
        switch (_id) {
        case 0: _t->currentPhraseGroupIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->newPhraseCreated((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 2: _t->newPhraseBehaviorCreated((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 3: _t->phraseGroupBoxIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->addPhraseButtonClicked(); break;
        case 5: _t->removePhraseButtonClicked(); break;
        case 6: _t->addBehaviorButtonClicked(); break;
        case 7: _t->removeBehaviorButtonClicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PhrasesWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PhrasesWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PhrasesWidget,
      qt_meta_data_PhrasesWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PhrasesWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PhrasesWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PhrasesWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PhrasesWidget))
        return static_cast<void*>(const_cast< PhrasesWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PhrasesWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
void PhrasesWidget::currentPhraseGroupIndexChanged(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PhrasesWidget::newPhraseCreated(std::string & _t1, std::string & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PhrasesWidget::newPhraseBehaviorCreated(std::string & _t1, std::string _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
