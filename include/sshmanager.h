// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// SSH connection manager for secure remote access to TurtleBot3 robots
// with automatic reconnection, heartbeat monitoring, and command queuing.

#ifndef ROS_SYNAPSE_INCLUDE_SSHMANAGER_H_
#define ROS_SYNAPSE_INCLUDE_SSHMANAGER_H_

#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QTcpSocket>
#include <QHostAddress>
#include <QQueue>
#include <QMutex>

class SSHManager : public QObject {
  Q_OBJECT

 public:
  enum ConnectionState {
    kDisconnected,
    kConnecting,
    kConnected,
    kReconnecting,
    kError
  };

  explicit SSHManager(QObject* parent = nullptr);
  ~SSHManager();

  void SetConnectionParams(const QString& host, const QString& username, 
                           int port = 22, const QString& password = QString());
  void SetKeyFile(const QString& key_file);
  
  bool IsConnected() const;
  ConnectionState GetConnectionState() const;
  QString GetLastError() const;

 public slots:
  void ConnectToHost();
  void DisconnectFromHost();
  void ExecuteCommand(const QString& command);
  void ExecuteCommandAsync(const QString& command);
  void SendInput(const QString& input);
  void Reconnect();

 signals:
  void Connected();
  void Disconnected();
  void ConnectionStateChanged(ConnectionState state);
  void CommandOutput(const QString& output);
  void CommandFinished(const QString& command, int exit_code);
  void ErrorOccurred(const QString& error);

 private slots:
  void OnProcessFinished(int exit_code, QProcess::ExitStatus exit_status);
  void OnProcessError(QProcess::ProcessError error);
  void OnProcessStarted();
  void OnReadyReadStandardOutput();
  void OnReadyReadStandardError();
  void OnConnectionTimeout();
  void OnHeartbeatTimeout();
  void CheckConnection();

 private:
  void SetupProcess();
  void StartConnection();
  void HandleConnectionError(const QString& error);
  void StartHeartbeat();
  void StopHeartbeat();
  void ProcessCommandQueue();
  QString BuildSSHCommand(const QString& command = QString());
  
  QProcess* ssh_process_;
  QTimer* connection_timer_;
  QTimer* heartbeat_timer_;
  QTcpSocket* test_socket_;
  
  QString host_;
  QString username_;
  QString password_;
  QString key_file_;
  int port_;
  
  ConnectionState connection_state_;
  QString last_error_;
  
  QQueue<QString> command_queue_;
  QMutex command_mutex_;
  
  bool is_connected_;
  bool use_key_auth_;
  int connection_timeout_;
  int heartbeat_interval_;
  int max_reconnect_attempts_;
  int reconnect_attempts_;
  
  static const int kDefaultConnectionTimeout = 30000;
  static const int kDefaultHeartbeatInterval = 60000;
  static const int kMaxReconnectAttempts = 3;
};

#endif  // ROS_SYNAPSE_INCLUDE_SSHMANAGER_H_