// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "sshmanager.h"
#include <QTimer>
#include <QDebug>

SSHManager::SSHManager(QObject* parent)
    : QObject(parent),
      ssh_process_(nullptr),
      connection_timer_(new QTimer(this)),
      heartbeat_timer_(new QTimer(this)),
      test_socket_(nullptr),
      port_(22),
      connection_state_(kDisconnected),
      is_connected_(false),
      use_key_auth_(false),
      connection_timeout_(kDefaultConnectionTimeout),
      heartbeat_interval_(kDefaultHeartbeatInterval),
      max_reconnect_attempts_(kMaxReconnectAttempts),
      reconnect_attempts_(0) {
  
  SetupProcess();
  
  // Setup connection timeout timer
  connection_timer_->setSingleShot(true);
  connect(connection_timer_, &QTimer::timeout, this, &SSHManager::OnConnectionTimeout);
  
  // Setup heartbeat timer
  connect(heartbeat_timer_, &QTimer::timeout, this, &SSHManager::OnHeartbeatTimeout);
}

SSHManager::~SSHManager() {
  if (ssh_process_) {
    ssh_process_->kill();
    ssh_process_->waitForFinished(3000);
  }
}

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

void SSHManager::ConnectToHost() {
  if (is_connected_) {
    return;
  }
  
  if (host_.isEmpty() || username_.isEmpty()) {
    HandleConnectionError("Host or username not set");
    return;
  }
  
  connection_state_ = kConnecting;
  emit ConnectionStateChanged(connection_state_);
  
  StartConnection();
}

void SSHManager::DisconnectFromHost() {
  if (!is_connected_) {
    return;
  }
  
  is_connected_ = false;
  connection_state_ = kDisconnected;
  
  if (ssh_process_) {
    ssh_process_->write("exit\n");
    ssh_process_->waitForFinished(3000);
    if (ssh_process_->state() != QProcess::NotRunning) {
      ssh_process_->kill();
    }
  }
  
  StopHeartbeat();
  emit ConnectionStateChanged(connection_state_);
  emit Disconnected();
}

void SSHManager::ExecuteCommand(const QString& command) {
  if (!is_connected_) {
    emit ErrorOccurred("Not connected to SSH server");
    return;
  }
  
  if (command.trimmed().isEmpty()) {
    return;
  }
  
  // Create a new SSH process for this command
  QProcess* cmd_process = new QProcess(this);
  
  // Build command
  QString program;
  QStringList arguments;
  
  if (!password_.isEmpty()) {
    program = "sshpass";
    arguments << "-p" << password_
             << "ssh"
             << "-o" << "StrictHostKeyChecking=no"
             << "-o" << "UserKnownHostsFile=/dev/null"
             << "-o" << "LogLevel=QUIET"
             << "-p" << QString::number(port_)
             << QString("%1@%2").arg(username_, host_)
             << command.trimmed();
  } else {
    program = "ssh";
    arguments << "-o" << "StrictHostKeyChecking=no"
             << "-o" << "UserKnownHostsFile=/dev/null"
             << "-o" << "LogLevel=QUIET"
             << "-p" << QString::number(port_)
             << QString("%1@%2").arg(username_, host_)
             << command.trimmed();
  }
  
  // Connect signals for this command
  connect(cmd_process, &QProcess::readyReadStandardOutput, [this, cmd_process]() {
    QByteArray data = cmd_process->readAllStandardOutput();
    QString output = QString::fromUtf8(data);
    if (!password_.isEmpty()) {
      output = output.replace(password_, "[PASSWORD HIDDEN]");
    }
    emit CommandOutput(output);
  });
  
  connect(cmd_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, cmd_process, command](int exit_code, QProcess::ExitStatus) {
    emit CommandFinished(command, exit_code);
    cmd_process->deleteLater();
  });
  
  cmd_process->start(program, arguments);
}

void SSHManager::ExecuteCommandAsync(const QString& command) {
  Q_UNUSED(command)
}

void SSHManager::SendInput(const QString& input) {
  Q_UNUSED(input)
}

void SSHManager::Reconnect() {}

void SSHManager::OnProcessFinished(int exit_code, QProcess::ExitStatus exit_status) {
  Q_UNUSED(exit_status)
  
  // If we had a successful connection (exit code 0) and we detected the connection, 
  // keep the "connected" status for the UI but mark process as finished
  if (exit_code == 0 && is_connected_) {
    // Successful command execution - keep connected status
    connection_timer_->stop();
  } else {
    // Connection failed or error occurred
    is_connected_ = false;
    connection_state_ = kDisconnected;
    connection_timer_->stop();
    StopHeartbeat();
    
    emit ConnectionStateChanged(connection_state_);
    emit Disconnected();
  }
  
  emit CommandFinished("SSH session", exit_code);
}

void SSHManager::OnProcessError(QProcess::ProcessError error) {
  QString error_message;
  switch (error) {
    case QProcess::FailedToStart:
      error_message = "SSH process failed to start - check if ssh is installed";
      break;
    case QProcess::Crashed:
      error_message = "SSH process crashed";
      break;
    case QProcess::Timedout:
      error_message = "SSH process timed out";
      break;
    case QProcess::WriteError:
      error_message = "SSH process write error";
      break;
    case QProcess::ReadError:
      error_message = "SSH process read error";
      break;
    default:
      error_message = "Unknown SSH process error";
      break;
  }
  
  HandleConnectionError(error_message);
}

void SSHManager::OnProcessStarted() {
  emit CommandOutput("SSH process started...");
  
  // Send password if provided
  if (!password_.isEmpty()) {
    // Wait a bit for password prompt
    QTimer::singleShot(1000, [this]() {
      if (ssh_process_ && ssh_process_->state() == QProcess::Running) {
        ssh_process_->write((password_ + "\n").toUtf8());
      }
    });
  }
  
  // Send a simple command to test the connection
  QTimer::singleShot(2000, [this]() {
    if (ssh_process_ && ssh_process_->state() == QProcess::Running) {
      ssh_process_->write("echo 'SSH_CONNECTION_READY'; exit 0\n");
    }
  });
}

void SSHManager::OnReadyReadStandardOutput() {
  if (!ssh_process_) {
    return;
  }
  
  QByteArray data = ssh_process_->readAllStandardOutput();
  QString output = QString::fromUtf8(data);
  
  if (!output.isEmpty()) {
    // Filter out password from output (in case it gets echoed)
    QString filtered_output = output;
    if (!password_.isEmpty()) {
      filtered_output = filtered_output.replace(password_, "[PASSWORD HIDDEN]");
    }
    
    // Check if this looks like a successful connection (MOTD, shell prompt, etc.)
    if (!is_connected_ && (output.contains("SSH_CONNECTION_READY") ||
                          output.contains("$") || output.contains("#") || 
                          output.contains("Welcome") || output.contains("Last login") ||
                          output.contains("Ubuntu") || output.contains("Linux"))) {
      is_connected_ = true;
      connection_state_ = kConnected;
      connection_timer_->stop();
      StartHeartbeat();
      emit ConnectionStateChanged(connection_state_);
      emit Connected();
    }
    
    emit CommandOutput(filtered_output);
  }
}

void SSHManager::OnReadyReadStandardError() {
  if (!ssh_process_) {
    return;
  }
  
  QByteArray data = ssh_process_->readAllStandardError();
  QString error = QString::fromUtf8(data);
  
  if (!error.isEmpty()) {
    // Filter out password from error output
    QString filtered_error = error;
    if (!password_.isEmpty()) {
      filtered_error = filtered_error.replace(password_, "[PASSWORD HIDDEN]");
    }
    
    // Check for password prompts
    if (error.contains("password:", Qt::CaseInsensitive) && !password_.isEmpty()) {
      ssh_process_->write((password_ + "\n").toUtf8());
      emit CommandOutput("[Password prompt detected - entering password]");
      return;
    }
    
    // Check for authentication failures
    if (error.contains("Permission denied") || error.contains("Authentication failed")) {
      HandleConnectionError("Authentication failed - check username/password");
      return;
    }
    
    // Check for connection failures
    if (error.contains("Connection refused") || error.contains("No route to host")) {
      HandleConnectionError("Connection failed - check host and port");
      return;
    }
    
    emit CommandOutput("SSH Error: " + filtered_error);
  }
}

void SSHManager::OnConnectionTimeout() {
  if (!is_connected_) {
    HandleConnectionError("Connection timeout - server did not respond");
  }
}

void SSHManager::OnHeartbeatTimeout() {}

void SSHManager::CheckConnection() {}

void SSHManager::SetupProcess() {
  if (ssh_process_) {
    ssh_process_->deleteLater();
  }
  
  ssh_process_ = new QProcess(this);
  
  connect(ssh_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this, &SSHManager::OnProcessFinished);
  connect(ssh_process_, &QProcess::errorOccurred,
          this, &SSHManager::OnProcessError);
  connect(ssh_process_, &QProcess::started,
          this, &SSHManager::OnProcessStarted);
  connect(ssh_process_, &QProcess::readyReadStandardOutput,
          this, &SSHManager::OnReadyReadStandardOutput);
  connect(ssh_process_, &QProcess::readyReadStandardError,
          this, &SSHManager::OnReadyReadStandardError);
}

void SSHManager::StartConnection() {
  if (!ssh_process_) {
    SetupProcess();
  }
  
  QString program;
  QStringList arguments;
  
  // Check if we can use sshpass for password authentication
  if (!password_.isEmpty()) {
    // Try to use sshpass if available
    program = "sshpass";
    arguments << "-p" << password_
             << "ssh"
             << "-o" << "StrictHostKeyChecking=no"
             << "-o" << "UserKnownHostsFile=/dev/null"
             << "-o" << "LogLevel=QUIET"
             << "-o" << "PubkeyAuthentication=no"  // Force password auth
             << "-o" << "ServerAliveInterval=30"   // Keep connection alive
             << "-o" << "ServerAliveCountMax=3"    // Max failed keepalives
             << "-p" << QString::number(port_)
             << QString("%1@%2").arg(username_, host_);
  } else {
    // Use regular ssh without password
    program = "ssh";
    arguments << "-o" << "StrictHostKeyChecking=no"
             << "-o" << "UserKnownHostsFile=/dev/null"
             << "-o" << "LogLevel=QUIET"
             << "-o" << "ServerAliveInterval=30"   // Keep connection alive
             << "-o" << "ServerAliveCountMax=3"    // Max failed keepalives
             << "-p" << QString::number(port_)
             << QString("%1@%2").arg(username_, host_);
  }
  
  // Start SSH process
  connection_timer_->start(connection_timeout_);
  ssh_process_->start(program, arguments);
  
  if (!ssh_process_->waitForStarted(5000)) {
    if (program == "sshpass") {
      emit CommandOutput("[sshpass not available - trying manual password entry]");
      // Fall back to regular SSH with manual password handling
      program = "ssh";
      arguments.clear();
      arguments << "-o" << "StrictHostKeyChecking=no"
               << "-o" << "UserKnownHostsFile=/dev/null"
               << "-o" << "LogLevel=QUIET"
               << "-o" << "PubkeyAuthentication=no"  // Force password auth
               << "-o" << "ServerAliveInterval=30"   // Keep connection alive
               << "-o" << "ServerAliveCountMax=3"    // Max failed keepalives
               << "-p" << QString::number(port_)
               << QString("%1@%2").arg(username_, host_);
      
      ssh_process_->start(program, arguments);
      if (!ssh_process_->waitForStarted(5000)) {
        HandleConnectionError("Failed to start SSH process");
        return;
      }
    } else {
      HandleConnectionError("Failed to start SSH process");
      return;
    }
  }
}

void SSHManager::HandleConnectionError(const QString& error) {
  last_error_ = error;
  connection_state_ = kError;
  is_connected_ = false;
  
  connection_timer_->stop();
  StopHeartbeat();
  
  emit ConnectionStateChanged(connection_state_);
  emit ErrorOccurred(error);
}

void SSHManager::StartHeartbeat() {
  heartbeat_timer_->start(heartbeat_interval_);
}

void SSHManager::StopHeartbeat() {
  heartbeat_timer_->stop();
}

void SSHManager::ProcessCommandQueue() {}

QString SSHManager::BuildSSHCommand(const QString& command) {
  Q_UNUSED(command)
  return QString();
}

void SSHManager::KillCommand(const QString& process_id) {
  if (!is_connected_ || !ssh_process_ || ssh_process_->state() != QProcess::Running) {
    qWarning() << "Cannot kill SSH command: not connected or SSH process not running";
    return;
  }
  
  // For SSH connections, we send Ctrl+C (ASCII 3) to interrupt the running command
  // This is the standard way to interrupt a running process via SSH
  QByteArray interrupt_signal;
  interrupt_signal.append(char(3)); // Ctrl+C character
  
  qDebug() << "Sending Ctrl+C signal to SSH process for process ID:" << process_id;
  ssh_process_->write(interrupt_signal);
  
  // Alternatively, we could send a kill command if we know the PID
  // ExecuteCommand(QString("kill -INT %1").arg(process_id));
}