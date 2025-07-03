/****************************************************************************
** Meta object code from reading C++ file 'sshmanager.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/sshmanager.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sshmanager.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_SSHManager_t {
    uint offsetsAndSizes[64];
    char stringdata0[11];
    char stringdata1[10];
    char stringdata2[1];
    char stringdata3[13];
    char stringdata4[23];
    char stringdata5[16];
    char stringdata6[6];
    char stringdata7[14];
    char stringdata8[7];
    char stringdata9[16];
    char stringdata10[8];
    char stringdata11[10];
    char stringdata12[14];
    char stringdata13[6];
    char stringdata14[14];
    char stringdata15[19];
    char stringdata16[15];
    char stringdata17[20];
    char stringdata18[10];
    char stringdata19[6];
    char stringdata20[10];
    char stringdata21[18];
    char stringdata22[21];
    char stringdata23[12];
    char stringdata24[15];
    char stringdata25[23];
    char stringdata26[17];
    char stringdata27[26];
    char stringdata28[25];
    char stringdata29[20];
    char stringdata30[19];
    char stringdata31[16];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_SSHManager_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_SSHManager_t qt_meta_stringdata_SSHManager = {
    {
        QT_MOC_LITERAL(0, 10),  // "SSHManager"
        QT_MOC_LITERAL(11, 9),  // "Connected"
        QT_MOC_LITERAL(21, 0),  // ""
        QT_MOC_LITERAL(22, 12),  // "Disconnected"
        QT_MOC_LITERAL(35, 22),  // "ConnectionStateChanged"
        QT_MOC_LITERAL(58, 15),  // "ConnectionState"
        QT_MOC_LITERAL(74, 5),  // "state"
        QT_MOC_LITERAL(80, 13),  // "CommandOutput"
        QT_MOC_LITERAL(94, 6),  // "output"
        QT_MOC_LITERAL(101, 15),  // "CommandFinished"
        QT_MOC_LITERAL(117, 7),  // "command"
        QT_MOC_LITERAL(125, 9),  // "exit_code"
        QT_MOC_LITERAL(135, 13),  // "ErrorOccurred"
        QT_MOC_LITERAL(149, 5),  // "error"
        QT_MOC_LITERAL(155, 13),  // "ConnectToHost"
        QT_MOC_LITERAL(169, 18),  // "DisconnectFromHost"
        QT_MOC_LITERAL(188, 14),  // "ExecuteCommand"
        QT_MOC_LITERAL(203, 19),  // "ExecuteCommandAsync"
        QT_MOC_LITERAL(223, 9),  // "SendInput"
        QT_MOC_LITERAL(233, 5),  // "input"
        QT_MOC_LITERAL(239, 9),  // "Reconnect"
        QT_MOC_LITERAL(249, 17),  // "OnProcessFinished"
        QT_MOC_LITERAL(267, 20),  // "QProcess::ExitStatus"
        QT_MOC_LITERAL(288, 11),  // "exit_status"
        QT_MOC_LITERAL(300, 14),  // "OnProcessError"
        QT_MOC_LITERAL(315, 22),  // "QProcess::ProcessError"
        QT_MOC_LITERAL(338, 16),  // "OnProcessStarted"
        QT_MOC_LITERAL(355, 25),  // "OnReadyReadStandardOutput"
        QT_MOC_LITERAL(381, 24),  // "OnReadyReadStandardError"
        QT_MOC_LITERAL(406, 19),  // "OnConnectionTimeout"
        QT_MOC_LITERAL(426, 18),  // "OnHeartbeatTimeout"
        QT_MOC_LITERAL(445, 15)   // "CheckConnection"
    },
    "SSHManager",
    "Connected",
    "",
    "Disconnected",
    "ConnectionStateChanged",
    "ConnectionState",
    "state",
    "CommandOutput",
    "output",
    "CommandFinished",
    "command",
    "exit_code",
    "ErrorOccurred",
    "error",
    "ConnectToHost",
    "DisconnectFromHost",
    "ExecuteCommand",
    "ExecuteCommandAsync",
    "SendInput",
    "input",
    "Reconnect",
    "OnProcessFinished",
    "QProcess::ExitStatus",
    "exit_status",
    "OnProcessError",
    "QProcess::ProcessError",
    "OnProcessStarted",
    "OnReadyReadStandardOutput",
    "OnReadyReadStandardError",
    "OnConnectionTimeout",
    "OnHeartbeatTimeout",
    "CheckConnection"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_SSHManager[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  134,    2, 0x06,    1 /* Public */,
       3,    0,  135,    2, 0x06,    2 /* Public */,
       4,    1,  136,    2, 0x06,    3 /* Public */,
       7,    1,  139,    2, 0x06,    5 /* Public */,
       9,    2,  142,    2, 0x06,    7 /* Public */,
      12,    1,  147,    2, 0x06,   10 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      14,    0,  150,    2, 0x0a,   12 /* Public */,
      15,    0,  151,    2, 0x0a,   13 /* Public */,
      16,    1,  152,    2, 0x0a,   14 /* Public */,
      17,    1,  155,    2, 0x0a,   16 /* Public */,
      18,    1,  158,    2, 0x0a,   18 /* Public */,
      20,    0,  161,    2, 0x0a,   20 /* Public */,
      21,    2,  162,    2, 0x08,   21 /* Private */,
      24,    1,  167,    2, 0x08,   24 /* Private */,
      26,    0,  170,    2, 0x08,   26 /* Private */,
      27,    0,  171,    2, 0x08,   27 /* Private */,
      28,    0,  172,    2, 0x08,   28 /* Private */,
      29,    0,  173,    2, 0x08,   29 /* Private */,
      30,    0,  174,    2, 0x08,   30 /* Private */,
      31,    0,  175,    2, 0x08,   31 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,   10,   11,
    QMetaType::Void, QMetaType::QString,   13,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void, QMetaType::QString,   19,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 22,   11,   23,
    QMetaType::Void, 0x80000000 | 25,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject SSHManager::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_SSHManager.offsetsAndSizes,
    qt_meta_data_SSHManager,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_SSHManager_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<SSHManager, std::true_type>,
        // method 'Connected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'Disconnected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ConnectionStateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<ConnectionState, std::false_type>,
        // method 'CommandOutput'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'CommandFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'ErrorOccurred'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ConnectToHost'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'DisconnectFromHost'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'ExecuteCommand'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'ExecuteCommandAsync'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'SendInput'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'Reconnect'
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
        // method 'OnConnectionTimeout'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'OnHeartbeatTimeout'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'CheckConnection'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void SSHManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SSHManager *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->Connected(); break;
        case 1: _t->Disconnected(); break;
        case 2: _t->ConnectionStateChanged((*reinterpret_cast< std::add_pointer_t<ConnectionState>>(_a[1]))); break;
        case 3: _t->CommandOutput((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->CommandFinished((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 5: _t->ErrorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 6: _t->ConnectToHost(); break;
        case 7: _t->DisconnectFromHost(); break;
        case 8: _t->ExecuteCommand((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 9: _t->ExecuteCommandAsync((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 10: _t->SendInput((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 11: _t->Reconnect(); break;
        case 12: _t->OnProcessFinished((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QProcess::ExitStatus>>(_a[2]))); break;
        case 13: _t->OnProcessError((*reinterpret_cast< std::add_pointer_t<QProcess::ProcessError>>(_a[1]))); break;
        case 14: _t->OnProcessStarted(); break;
        case 15: _t->OnReadyReadStandardOutput(); break;
        case 16: _t->OnReadyReadStandardError(); break;
        case 17: _t->OnConnectionTimeout(); break;
        case 18: _t->OnHeartbeatTimeout(); break;
        case 19: _t->CheckConnection(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (SSHManager::*)();
            if (_t _q_method = &SSHManager::Connected; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (SSHManager::*)();
            if (_t _q_method = &SSHManager::Disconnected; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (SSHManager::*)(ConnectionState );
            if (_t _q_method = &SSHManager::ConnectionStateChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (SSHManager::*)(const QString & );
            if (_t _q_method = &SSHManager::CommandOutput; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (SSHManager::*)(const QString & , int );
            if (_t _q_method = &SSHManager::CommandFinished; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (SSHManager::*)(const QString & );
            if (_t _q_method = &SSHManager::ErrorOccurred; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject *SSHManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SSHManager::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SSHManager.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SSHManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 20)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 20;
    }
    return _id;
}

// SIGNAL 0
void SSHManager::Connected()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void SSHManager::Disconnected()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void SSHManager::ConnectionStateChanged(ConnectionState _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SSHManager::CommandOutput(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SSHManager::CommandFinished(const QString & _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void SSHManager::ErrorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
