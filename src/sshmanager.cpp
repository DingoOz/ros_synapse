// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "sshmanager.h"

SSHManager::SSHManager(QObject* parent)
    : QObject(parent),
      connection_state_(kDisconnected),
      is_connected_(false) {}

SSHManager::~SSHManager() {}

void SSHManager::SetConnectionParams(const QString& host, const QString& username,
                                     int port, const QString& password) {
  host_ = host;
  username_ = username;
  port_ = port;
  password_ = password;
}

void SSHManager::SetKeyFile(const QString& key_file) {
  key_file_ = key_file;
}

bool SSHManager::IsConnected() const {
  return is_connected_;
}

SSHManager::ConnectionState SSHManager::GetConnectionState() const {
  return connection_state_;
}

QString SSHManager::GetLastError() const {
  return last_error_;
}

void SSHManager::ConnectToHost() {}

void SSHManager::DisconnectFromHost() {}

void SSHManager::ExecuteCommand(const QString& command) {
  Q_UNUSED(command)
}

void SSHManager::ExecuteCommandAsync(const QString& command) {
  Q_UNUSED(command)
}

void SSHManager::SendInput(const QString& input) {
  Q_UNUSED(input)
}

void SSHManager::Reconnect() {}

void SSHManager::OnProcessFinished(int exit_code, QProcess::ExitStatus exit_status) {
  Q_UNUSED(exit_code)
  Q_UNUSED(exit_status)
}

void SSHManager::OnProcessError(QProcess::ProcessError error) {
  Q_UNUSED(error)
}

void SSHManager::OnProcessStarted() {}

void SSHManager::OnReadyReadStandardOutput() {}

void SSHManager::OnReadyReadStandardError() {}

void SSHManager::OnConnectionTimeout() {}

void SSHManager::OnHeartbeatTimeout() {}

void SSHManager::CheckConnection() {}

void SSHManager::SetupProcess() {}

void SSHManager::StartConnection() {}

void SSHManager::HandleConnectionError(const QString& error) {
  Q_UNUSED(error)
}

void SSHManager::StartHeartbeat() {}

void SSHManager::StopHeartbeat() {}

void SSHManager::ProcessCommandQueue() {}

QString SSHManager::BuildSSHCommand(const QString& command) {
  Q_UNUSED(command)
  return QString();
}