/****************************************************************************
** Meta object code from reading C++ file 'PhrasesWidget.h'
**
** Created: Wed Aug 7 18:58:31 2013
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
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      20,   15,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   15,   14,   14, 0x0a,
      96,   14,   14,   14, 0x08,
     121,   14,   14,   14, 0x08,
     149,   14,   14,   14, 0x08,
     176,   14,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PhrasesWidget[] = {
    "PhrasesWidget\0\0text\0"
    "currentPhraseGroupIndexChanged(QString)\0"
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
        case 1: _t->phraseGroupBoxIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->addPhraseButtonClicked(); break;
        case 3: _t->removePhraseButtonClicked(); break;
        case 4: _t->addBehaviorButtonClicked(); break;
        case 5: _t->removeBehaviorButtonClicked(); break;
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
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void PhrasesWidget::currentPhraseGroupIndexChanged(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
