/****************************************************************************
** Meta object code from reading C++ file 'commandwidget.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/commandwidget.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'commandwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_CommandWidget_t {
    uint offsetsAndSizes[36];
    char stringdata0[14];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[16];
    char stringdata5[7];
    char stringdata6[18];
    char stringdata7[8];
    char stringdata8[21];
    char stringdata9[12];
    char stringdata10[21];
    char stringdata11[11];
    char stringdata12[19];
    char stringdata13[15];
    char stringdata14[17];
    char stringdata15[15];
    char stringdata16[15];
    char stringdata17[18];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CommandWidget_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CommandWidget_t qt_meta_stringdata_CommandWidget = {
    {
        QT_MOC_LITERAL(0, 13),  // "CommandWidget"
        QT_MOC_LITERAL(14, 12),  // "CommandReady"
        QT_MOC_LITERAL(27, 0),  // ""
        QT_MOC_LITERAL(28, 7),  // "command"
        QT_MOC_LITERAL(36, 15),  // "CommandExecuted"
        QT_MOC_LITERAL(52, 6),  // "output"
        QT_MOC_LITERAL(59, 17),  // "OnPackageSelected"
        QT_MOC_LITERAL(77, 7),  // "package"
        QT_MOC_LITERAL(85, 20),  // "OnLaunchFileSelected"
        QT_MOC_LITERAL(106, 11),  // "launch_file"
        QT_MOC_LITERAL(118, 20),  // "OnExecutableSelected"
        QT_MOC_LITERAL(139, 10),  // "executable"
        QT_MOC_LITERAL(150, 18),  // "OnParameterChanged"
        QT_MOC_LITERAL(169, 14),  // "OnBuildCommand"
        QT_MOC_LITERAL(184, 16),  // "OnExecuteCommand"
        QT_MOC_LITERAL(201, 14),  // "OnSaveTemplate"
        QT_MOC_LITERAL(216, 14),  // "OnLoadTemplate"
        QT_MOC_LITERAL(231, 17)   // "OnClearParameters"
    },
    "CommandWidget",
    "CommandReady",
    "",
    "command",
    "CommandExecuted",
    "output",
    "OnPackageSelected",
    "package",
    "OnLaunchFileSelected",
    "launch_file",
    "OnExecutableSelected",
    "executable",
    "OnParameterChanged",
    "OnBuildCommand",
    "OnExecuteCommand",
    "OnSaveTemplate",
    "OnLoadTemplate",
    "OnClearParameters"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CommandWidget[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   80,    2, 0x06,    1 /* Public */,
       4,    2,   83,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       6,    1,   88,    2, 0x08,    6 /* Private */,
       8,    1,   91,    2, 0x08,    8 /* Private */,
      10,    1,   94,    2, 0x08,   10 /* Private */,
      12,    0,   97,    2, 0x08,   12 /* Private */,
      13,    0,   98,    2, 0x08,   13 /* Private */,
      14,    0,   99,    2, 0x08,   14 /* Private */,
      15,    0,  100,    2, 0x08,   15 /* Private */,
      16,    0,  101,    2, 0x08,   16 /* Private */,
      17,    0,  102,    2, 0x08,   17 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    3,    5,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void, QMetaType::QString,    9,
    QMetaType::Void, QMetaType::QString,   11,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject CommandWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_CommandWidget.offsetsAndSizes,
    qt_meta_data_CommandWidget,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CommandWidget_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<CommandWidget, std::true_type>,
        // method 'CommandReady'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'CommandExecuted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'OnPackageSelected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'OnLaunchFileSelected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'OnExecutableSelected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'OnParameterChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnBuildCommand'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnExecuteCommand'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnSaveTemplate'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnLoadTemplate'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnClearParameters'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void CommandWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CommandWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->CommandReady((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->CommandExecuted((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 2: _t->OnPackageSelected((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->OnLaunchFileSelected((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->OnExecutableSelected((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 5: _t->OnParameterChanged(); break;
        case 6: _t->OnBuildCommand(); break;
        case 7: _t->OnExecuteCommand(); break;
        case 8: _t->OnSaveTemplate(); break;
        case 9: _t->OnLoadTemplate(); break;
        case 10: _t->OnClearParameters(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CommandWidget::*)(const QString & );
            if (_t _q_method = &CommandWidget::CommandReady; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CommandWidget::*)(const QString & , const QString & );
            if (_t _q_method = &CommandWidget::CommandExecuted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject *CommandWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CommandWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CommandWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int CommandWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void CommandWidget::CommandReady(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CommandWidget::CommandExecuted(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
