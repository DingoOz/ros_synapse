/****************************************************************************
** Meta object code from reading C++ file 'ros2executor.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/ros2executor.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ros2executor.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_ROS2Executor_t {
    uint offsetsAndSizes[76];
    char stringdata0[13];
    char stringdata1[15];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[11];
    char stringdata5[14];
    char stringdata6[7];
    char stringdata7[16];
    char stringdata8[10];
    char stringdata9[13];
    char stringdata10[6];
    char stringdata11[19];
    char stringdata12[19];
    char stringdata13[18];
    char stringdata14[10];
    char stringdata15[15];
    char stringdata16[18];
    char stringdata17[8];
    char stringdata18[12];
    char stringdata19[22];
    char stringdata20[11];
    char stringdata21[12];
    char stringdata22[11];
    char stringdata23[10];
    char stringdata24[12];
    char stringdata25[17];
    char stringdata26[14];
    char stringdata27[16];
    char stringdata28[18];
    char stringdata29[21];
    char stringdata30[12];
    char stringdata31[15];
    char stringdata32[23];
    char stringdata33[17];
    char stringdata34[26];
    char stringdata35[25];
    char stringdata36[18];
    char stringdata37[16];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_ROS2Executor_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_ROS2Executor_t qt_meta_stringdata_ROS2Executor = {
    {
        QT_MOC_LITERAL(0, 12),  // "ROS2Executor"
        QT_MOC_LITERAL(13, 14),  // "CommandStarted"
        QT_MOC_LITERAL(28, 0),  // ""
        QT_MOC_LITERAL(29, 7),  // "command"
        QT_MOC_LITERAL(37, 10),  // "process_id"
        QT_MOC_LITERAL(48, 13),  // "CommandOutput"
        QT_MOC_LITERAL(62, 6),  // "output"
        QT_MOC_LITERAL(69, 15),  // "CommandFinished"
        QT_MOC_LITERAL(85, 9),  // "exit_code"
        QT_MOC_LITERAL(95, 12),  // "CommandError"
        QT_MOC_LITERAL(108, 5),  // "error"
        QT_MOC_LITERAL(114, 18),  // "ProcessListChanged"
        QT_MOC_LITERAL(133, 18),  // "PackageListChanged"
        QT_MOC_LITERAL(152, 17),  // "ROS2StatusChanged"
        QT_MOC_LITERAL(170, 9),  // "available"
        QT_MOC_LITERAL(180, 14),  // "ExecuteCommand"
        QT_MOC_LITERAL(195, 17),  // "ExecuteLaunchFile"
        QT_MOC_LITERAL(213, 7),  // "package"
        QT_MOC_LITERAL(221, 11),  // "launch_file"
        QT_MOC_LITERAL(233, 21),  // "QMap<QString,QString>"
        QT_MOC_LITERAL(255, 10),  // "parameters"
        QT_MOC_LITERAL(266, 11),  // "ExecuteNode"
        QT_MOC_LITERAL(278, 10),  // "executable"
        QT_MOC_LITERAL(289, 9),  // "arguments"
        QT_MOC_LITERAL(299, 11),  // "KillProcess"
        QT_MOC_LITERAL(311, 16),  // "KillAllProcesses"
        QT_MOC_LITERAL(328, 13),  // "EmergencyStop"
        QT_MOC_LITERAL(342, 15),  // "RefreshPackages"
        QT_MOC_LITERAL(358, 17),  // "OnProcessFinished"
        QT_MOC_LITERAL(376, 20),  // "QProcess::ExitStatus"
        QT_MOC_LITERAL(397, 11),  // "exit_status"
        QT_MOC_LITERAL(409, 14),  // "OnProcessError"
        QT_MOC_LITERAL(424, 22),  // "QProcess::ProcessError"
        QT_MOC_LITERAL(447, 16),  // "OnProcessStarted"
        QT_MOC_LITERAL(464, 25),  // "OnReadyReadStandardOutput"
        QT_MOC_LITERAL(490, 24),  // "OnReadyReadStandardError"
        QT_MOC_LITERAL(515, 17),  // "UpdateProcessList"
        QT_MOC_LITERAL(533, 15)   // "CheckROS2Status"
    },
    "ROS2Executor",
    "CommandStarted",
    "",
    "command",
    "process_id",
    "CommandOutput",
    "output",
    "CommandFinished",
    "exit_code",
    "CommandError",
    "error",
    "ProcessListChanged",
    "PackageListChanged",
    "ROS2StatusChanged",
    "available",
    "ExecuteCommand",
    "ExecuteLaunchFile",
    "package",
    "launch_file",
    "QMap<QString,QString>",
    "parameters",
    "ExecuteNode",
    "executable",
    "arguments",
    "KillProcess",
    "KillAllProcesses",
    "EmergencyStop",
    "RefreshPackages",
    "OnProcessFinished",
    "QProcess::ExitStatus",
    "exit_status",
    "OnProcessError",
    "QProcess::ProcessError",
    "OnProcessStarted",
    "OnReadyReadStandardOutput",
    "OnReadyReadStandardError",
    "UpdateProcessList",
    "CheckROS2Status"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_ROS2Executor[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,  152,    2, 0x06,    1 /* Public */,
       5,    2,  157,    2, 0x06,    4 /* Public */,
       7,    2,  162,    2, 0x06,    7 /* Public */,
       9,    2,  167,    2, 0x06,   10 /* Public */,
      11,    0,  172,    2, 0x06,   13 /* Public */,
      12,    0,  173,    2, 0x06,   14 /* Public */,
      13,    1,  174,    2, 0x06,   15 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      15,    1,  177,    2, 0x0a,   17 /* Public */,
      16,    3,  180,    2, 0x0a,   19 /* Public */,
      16,    2,  187,    2, 0x2a,   23 /* Public | MethodCloned */,
      21,    3,  192,    2, 0x0a,   26 /* Public */,
      21,    2,  199,    2, 0x2a,   30 /* Public | MethodCloned */,
      24,    1,  204,    2, 0x0a,   33 /* Public */,
      25,    0,  207,    2, 0x0a,   35 /* Public */,
      26,    0,  208,    2, 0x0a,   36 /* Public */,
      27,    0,  209,    2, 0x0a,   37 /* Public */,
      28,    2,  210,    2, 0x08,   38 /* Private */,
      31,    1,  215,    2, 0x08,   41 /* Private */,
      33,    0,  218,    2, 0x08,   43 /* Private */,
      34,    0,  219,    2, 0x08,   44 /* Private */,
      35,    0,  220,    2, 0x08,   45 /* Private */,
      36,    0,  221,    2, 0x08,   46 /* Private */,
      37,    0,  222,    2, 0x08,   47 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    3,    4,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    4,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    4,    8,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    4,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   14,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, 0x80000000 | 19,   17,   18,   20,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   17,   18,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QStringList,   17,   22,   23,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   17,   22,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 29,    8,   30,
    QMetaType::Void, 0x80000000 | 32,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject ROS2Executor::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ROS2Executor.offsetsAndSizes,
    qt_meta_data_ROS2Executor,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_ROS2Executor_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<ROS2Executor, std::true_type>,
        // method 'CommandStarted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'CommandOutput'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'CommandFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'CommandError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ProcessListChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'PackageListChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ROS2StatusChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'ExecuteCommand'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ExecuteLaunchFile'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QMap<QString,QString> &, std::false_type>,
        // method 'ExecuteLaunchFile'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ExecuteNode'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QStringList &, std::false_type>,
        // method 'ExecuteNode'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'KillProcess'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'KillAllProcesses'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'EmergencyStop'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'RefreshPackages'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnProcessFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<QProcess::ExitStatus, std::false_type>,
        // method 'OnProcessError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QProcess::ProcessError, std::false_type>,
        // method 'OnProcessStarted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnReadyReadStandardOutput'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnReadyReadStandardError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'UpdateProcessList'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'CheckROS2Status'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void ROS2Executor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROS2Executor *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->CommandStarted((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->CommandOutput((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 2: _t->CommandFinished((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 3: _t->CommandError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 4: _t->ProcessListChanged(); break;
        case 5: _t->PackageListChanged(); break;
        case 6: _t->ROS2StatusChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 7: _t->ExecuteCommand((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 8: _t->ExecuteLaunchFile((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QMap<QString,QString>>>(_a[3]))); break;
        case 9: _t->ExecuteLaunchFile((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 10: _t->ExecuteNode((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QStringList>>(_a[3]))); break;
        case 11: _t->ExecuteNode((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 12: _t->KillProcess((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 13: _t->KillAllProcesses(); break;
        case 14: _t->EmergencyStop(); break;
        case 15: _t->RefreshPackages(); break;
        case 16: _t->OnProcessFinished((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QProcess::ExitStatus>>(_a[2]))); break;
        case 17: _t->OnProcessError((*reinterpret_cast< std::add_pointer_t<QProcess::ProcessError>>(_a[1]))); break;
        case 18: _t->OnProcessStarted(); break;
        case 19: _t->OnReadyReadStandardOutput(); break;
        case 20: _t->OnReadyReadStandardError(); break;
        case 21: _t->UpdateProcessList(); break;
        case 22: _t->CheckROS2Status(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROS2Executor::*)(const QString & , const QString & );
            if (_t _q_method = &ROS2Executor::CommandStarted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROS2Executor::*)(const QString & , const QString & );
            if (_t _q_method = &ROS2Executor::CommandOutput; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ROS2Executor::*)(const QString & , int );
            if (_t _q_method = &ROS2Executor::CommandFinished; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ROS2Executor::*)(const QString & , const QString & );
            if (_t _q_method = &ROS2Executor::CommandError; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (ROS2Executor::*)();
            if (_t _q_method = &ROS2Executor::ProcessListChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (ROS2Executor::*)();
            if (_t _q_method = &ROS2Executor::PackageListChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (ROS2Executor::*)(bool );
            if (_t _q_method = &ROS2Executor::ROS2StatusChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 6;
                return;
            }
        }
    }
}

const QMetaObject *ROS2Executor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROS2Executor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ROS2Executor.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ROS2Executor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 23)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 23;
    }
    return _id;
}

// SIGNAL 0
void ROS2Executor::CommandStarted(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ROS2Executor::CommandOutput(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ROS2Executor::CommandFinished(const QString & _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ROS2Executor::CommandError(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void ROS2Executor::ProcessListChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void ROS2Executor::PackageListChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void ROS2Executor::ROS2StatusChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
