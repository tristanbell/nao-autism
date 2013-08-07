/****************************************************************************
** Meta object code from reading C++ file 'PhrasesWidget.h'
**
** Created: Wed Aug 7 13:28:31 2013
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
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x08,
      40,   14,   14,   14, 0x08,
      68,   14,   14,   14, 0x08,
      95,   14,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PhrasesWidget[] = {
    "PhrasesWidget\0\0addPhraseButtonClicked()\0"
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
        case 0: _t->addPhraseButtonClicked(); break;
        case 1: _t->removePhraseButtonClicked(); break;
        case 2: _t->addBehaviorButtonClicked(); break;
        case 3: _t->removeBehaviorButtonClicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
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
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
