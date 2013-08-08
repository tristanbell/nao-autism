/****************************************************************************
** Meta object code from reading C++ file 'FileMenu.h'
**
** Created: Thu Aug 8 15:36:07 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "FileMenu.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FileMenu.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_FileMenu[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   10,    9,    9, 0x05,
      48,    9,    9,    9, 0x05,
      66,   10,    9,    9, 0x05,

 // slots: signature, parameters, type, tag, flags
      97,    9,    9,    9, 0x08,
     113,    9,    9,    9, 0x08,
     129,    9,    9,    9, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_FileMenu[] = {
    "FileMenu\0\0location\0onOpenRequested(std::string)\0"
    "onSaveRequested()\0onSaveAsRequested(std::string)\0"
    "openTriggered()\0saveTriggered()\0"
    "saveAsTriggered()\0"
};

void FileMenu::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        FileMenu *_t = static_cast<FileMenu *>(_o);
        switch (_id) {
        case 0: _t->onOpenRequested((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 1: _t->onSaveRequested(); break;
        case 2: _t->onSaveAsRequested((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 3: _t->openTriggered(); break;
        case 4: _t->saveTriggered(); break;
        case 5: _t->saveAsTriggered(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData FileMenu::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject FileMenu::staticMetaObject = {
    { &QMenu::staticMetaObject, qt_meta_stringdata_FileMenu,
      qt_meta_data_FileMenu, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &FileMenu::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *FileMenu::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *FileMenu::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_FileMenu))
        return static_cast<void*>(const_cast< FileMenu*>(this));
    return QMenu::qt_metacast(_clname);
}

int FileMenu::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMenu::qt_metacall(_c, _id, _a);
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
void FileMenu::onOpenRequested(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void FileMenu::onSaveRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void FileMenu::onSaveAsRequested(const std::string & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
