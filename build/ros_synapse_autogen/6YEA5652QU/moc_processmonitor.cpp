/****************************************************************************
** Meta object code from reading C++ file 'processmonitor.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/processmonitor.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'processmonitor.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_ProcessMonitor_t {
    uint offsetsAndSizes[60];
    char stringdata0[15];
    char stringdata1[15];
    char stringdata2[1];
    char stringdata3[4];
    char stringdata4[5];
    char stringdata5[16];
    char stringdata6[19];
    char stringdata7[12];
    char stringdata8[5];
    char stringdata9[19];
    char stringdata10[17];
    char stringdata11[10];
    char stringdata12[11];
    char stringdata13[13];
    char stringdata14[16];
    char stringdata15[15];
    char stringdata16[19];
    char stringdata17[12];
    char stringdata18[16];
    char stringdata19[19];
    char stringdata20[9];
    char stringdata21[18];
    char stringdata22[15];
    char stringdata23[13];
    char stringdata24[5];
    char stringdata25[13];
    char stringdata26[18];
    char stringdata27[22];
    char stringdata28[19];
    char stringdata29[23];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_ProcessMonitor_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_ProcessMonitor_t qt_meta_stringdata_ProcessMonitor = {
    {
        QT_MOC_LITERAL(0, 14),  // "ProcessMonitor"
        QT_MOC_LITERAL(15, 14),  // "ProcessStarted"
        QT_MOC_LITERAL(30, 0),  // ""
        QT_MOC_LITERAL(31, 3),  // "pid"
        QT_MOC_LITERAL(35, 4),  // "name"
        QT_MOC_LITERAL(40, 15),  // "ProcessFinished"
        QT_MOC_LITERAL(56, 18),  // "ProcessDataChanged"
        QT_MOC_LITERAL(75, 11),  // "ProcessData"
        QT_MOC_LITERAL(87, 4),  // "data"
        QT_MOC_LITERAL(92, 18),  // "ProcessListUpdated"
        QT_MOC_LITERAL(111, 16),  // "ROS2NodeDetected"
        QT_MOC_LITERAL(128, 9),  // "node_name"
        QT_MOC_LITERAL(138, 10),  // "namespace_"
        QT_MOC_LITERAL(149, 12),  // "ROS2NodeLost"
        QT_MOC_LITERAL(162, 15),  // "StartMonitoring"
        QT_MOC_LITERAL(178, 14),  // "StopMonitoring"
        QT_MOC_LITERAL(193, 18),  // "RefreshProcessList"
        QT_MOC_LITERAL(212, 11),  // "KillProcess"
        QT_MOC_LITERAL(224, 15),  // "KillProcessTree"
        QT_MOC_LITERAL(240, 18),  // "SetProcessPriority"
        QT_MOC_LITERAL(259, 8),  // "priority"
        QT_MOC_LITERAL(268, 17),  // "FilterByROS2Nodes"
        QT_MOC_LITERAL(286, 14),  // "show_only_ros2"
        QT_MOC_LITERAL(301, 12),  // "FilterByUser"
        QT_MOC_LITERAL(314, 4),  // "user"
        QT_MOC_LITERAL(319, 12),  // "FilterByName"
        QT_MOC_LITERAL(332, 17),  // "UpdateProcessList"
        QT_MOC_LITERAL(350, 21),  // "OnProcessScanFinished"
        QT_MOC_LITERAL(372, 18),  // "OnProcessScanError"
        QT_MOC_LITERAL(391, 22)   // "OnROS2NodeScanFinished"
    },
    "ProcessMonitor",
    "ProcessStarted",
    "",
    "pid",
    "name",
    "ProcessFinished",
    "ProcessDataChanged",
    "ProcessData",
    "data",
    "ProcessListUpdated",
    "ROS2NodeDetected",
    "node_name",
    "namespace_",
    "ROS2NodeLost",
    "StartMonitoring",
    "StopMonitoring",
    "RefreshProcessList",
    "KillProcess",
    "KillProcessTree",
    "SetProcessPriority",
    "priority",
    "FilterByROS2Nodes",
    "show_only_ros2",
    "FilterByUser",
    "user",
    "FilterByName",
    "UpdateProcessList",
    "OnProcessScanFinished",
    "OnProcessScanError",
    "OnROS2NodeScanFinished"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_ProcessMonitor[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,  128,    2, 0x06,    1 /* Public */,
       5,    2,  133,    2, 0x06,    4 /* Public */,
       6,    2,  138,    2, 0x06,    7 /* Public */,
       9,    0,  143,    2, 0x06,   10 /* Public */,
      10,    2,  144,    2, 0x06,   11 /* Public */,
      13,    2,  149,    2, 0x06,   14 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      14,    0,  154,    2, 0x0a,   17 /* Public */,
      15,    0,  155,    2, 0x0a,   18 /* Public */,
      16,    0,  156,    2, 0x0a,   19 /* Public */,
      17,    1,  157,    2, 0x0a,   20 /* Public */,
      18,    1,  160,    2, 0x0a,   22 /* Public */,
      19,    2,  163,    2, 0x0a,   24 /* Public */,
      21,    1,  168,    2, 0x0a,   27 /* Public */,
      23,    1,  171,    2, 0x0a,   29 /* Public */,
      25,    1,  174,    2, 0x0a,   31 /* Public */,
      26,    0,  177,    2, 0x08,   33 /* Private */,
      27,    0,  178,    2, 0x08,   34 /* Private */,
      28,    0,  179,    2, 0x08,   35 /* Private */,
      29,    0,  180,    2, 0x08,   36 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::QString,    3,    4,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,    3,    4,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 7,    3,    8,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   11,   12,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   11,   12,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,   20,
    QMetaType::Void, QMetaType::Bool,   22,
    QMetaType::Void, QMetaType::QString,   24,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject ProcessMonitor::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ProcessMonitor.offsetsAndSizes,
    qt_meta_data_ProcessMonitor,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_ProcessMonitor_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<ProcessMonitor, std::true_type>,
        // method 'ProcessStarted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ProcessFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ProcessDataChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<const ProcessData &, std::false_type>,
        // method 'ProcessListUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ROS2NodeDetected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ROS2NodeLost'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'StartMonitoring'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'StopMonitoring'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'RefreshProcessList'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'KillProcess'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'KillProcessTree'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'SetProcessPriority'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'FilterByROS2Nodes'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'FilterByUser'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'FilterByName'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'UpdateProcessList'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnProcessScanFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnProcessScanError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnROS2NodeScanFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void ProcessMonitor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ProcessMonitor *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->ProcessStarted((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->ProcessFinished((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 2: _t->ProcessDataChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<ProcessData>>(_a[2]))); break;
        case 3: _t->ProcessListUpdated(); break;
        case 4: _t->ROS2NodeDetected((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 5: _t->ROS2NodeLost((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 6: _t->StartMonitoring(); break;
        case 7: _t->StopMonitoring(); break;
        case 8: _t->RefreshProcessList(); break;
        case 9: _t->KillProcess((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 10: _t->KillProcessTree((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 11: _t->SetProcessPriority((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 12: _t->FilterByROS2Nodes((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 13: _t->FilterByUser((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 14: _t->FilterByName((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 15: _t->UpdateProcessList(); break;
        case 16: _t->OnProcessScanFinished(); break;
        case 17: _t->OnProcessScanError(); break;
        case 18: _t->OnROS2NodeScanFinished(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ProcessMonitor::*)(int , const QString & );
            if (_t _q_method = &ProcessMonitor::ProcessStarted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ProcessMonitor::*)(int , const QString & );
            if (_t _q_method = &ProcessMonitor::ProcessFinished; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ProcessMonitor::*)(int , const ProcessData & );
            if (_t _q_method = &ProcessMonitor::ProcessDataChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ProcessMonitor::*)();
            if (_t _q_method = &ProcessMonitor::ProcessListUpdated; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (ProcessMonitor::*)(const QString & , const QString & );
            if (_t _q_method = &ProcessMonitor::ROS2NodeDetected; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (ProcessMonitor::*)(const QString & , const QString & );
            if (_t _q_method = &ProcessMonitor::ROS2NodeLost; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject *ProcessMonitor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ProcessMonitor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ProcessMonitor.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ProcessMonitor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 19)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 19;
    }
    return _id;
}

// SIGNAL 0
void ProcessMonitor::ProcessStarted(int _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ProcessMonitor::ProcessFinished(int _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ProcessMonitor::ProcessDataChanged(int _t1, const ProcessData & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void ProcessMonitor::ProcessListUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void ProcessMonitor::ROS2NodeDetected(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void ProcessMonitor::ROS2NodeLost(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
