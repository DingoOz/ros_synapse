// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "ros2executor.h"

ROS2Executor::ROS2Executor(QObject* parent)
    : QObject(parent),
      execution_mode_(kLocal),
      domain_id_(0),
      ros2_available_(false) {}

ROS2Executor::~ROS2Executor() {}

void ROS2Executor::SetExecutionMode(ExecutionMode mode) {
  execution_mode_ = mode;
}

void ROS2Executor::SetROS2Environment(const QString& workspace, int domain_id) {
  workspace_ = workspace;
  domain_id_ = domain_id;
}

void ROS2Executor::SetTurtleBot3Model(const QString& model) {
  turtlebot3_model_ = model;
}

void ROS2Executor::SetRemoteConnection(const QString& host, const QString& username) {
  remote_host_ = host;
  remote_username_ = username;
}

bool ROS2Executor::IsROS2Available() const {
  return ros2_available_;
}

QStringList ROS2Executor::GetAvailablePackages() const {
  return available_packages_;
}

QStringList ROS2Executor::GetPackageExecutables(const QString& package) const {
  return package_executables_.value(package);
}

QStringList ROS2Executor::GetPackageLaunchFiles(const QString& package) const {
  return package_launch_files_.value(package);
}

QMap<QString, ROS2Executor::ProcessInfo> ROS2Executor::GetRunningProcesses() const {
  return running_processes_;
}

void ROS2Executor::ExecuteCommand(const QString& command) {
  if (command.trimmed().isEmpty()) {
    return;
  }
  
  // Create new process for this command
  QProcess* process = CreateProcess();
  if (!process) {
    emit CommandError("Failed to create process");
    return;
  }
  
  QString process_id = GenerateProcessId();
  
  // Store process info
  ProcessInfo info;
  info.process = process;
  info.command = command;
  info.start_time = QDateTime::currentDateTime();
  info.is_running = true;
  
  running_processes_[process_id] = info;
  
  // Set up process environment
  QStringList env = BuildEnvironment();
  if (!env.isEmpty()) {
    process->setEnvironment(env);
  }
  
  // Parse command - handle GUI applications that need to be detached
  QStringList args = command.split(' ', Qt::SkipEmptyParts);
  if (args.isEmpty()) {
    delete process;
    running_processes_.remove(process_id);
    emit CommandError("Empty command");
    return;
  }
  
  QString program = args.takeFirst();
  
  // For GUI applications like rviz2, use detached process
  if (program == "rviz2" || program.contains("rviz") || 
      command.contains("gazebo") || command.contains("rqt")) {
    
    emit CommandOutput(QString("Starting GUI application: %1").arg(command));
    emit CommandStarted(command, process_id);
    
    // Start detached process for GUI applications
    bool started = QProcess::startDetached(program, args);
    if (started) {
      emit CommandOutput(QString("GUI application started successfully: %1").arg(program));
    } else {
      emit CommandError(QString("Failed to start GUI application: %1").arg(program));
    }
    
    // Clean up the process since we're using detached
    delete process;
    running_processes_.remove(process_id);
    return;
  }
  
  // For regular commands, use managed process
  emit CommandStarted(command, process_id);
  emit CommandOutput(QString("Starting: %1").arg(command));
  
  // Connect process signals
  connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, process_id](int exit_code, QProcess::ExitStatus) {
    OnProcessFinished(exit_code, QProcess::NormalExit);
    CleanupProcess(process_id);
  });
  
  connect(process, &QProcess::errorOccurred, [this, process_id](QProcess::ProcessError error) {
    Q_UNUSED(error)
    OnProcessError(error);
    CleanupProcess(process_id);
  });
  
  connect(process, &QProcess::readyReadStandardOutput, [this, process]() {
    QByteArray data = process->readAllStandardOutput();
    QString output = QString::fromUtf8(data);
    if (!output.isEmpty()) {
      emit CommandOutput(output);
    }
  });
  
  connect(process, &QProcess::readyReadStandardError, [this, process]() {
    QByteArray data = process->readAllStandardError();
    QString error = QString::fromUtf8(data);
    if (!error.isEmpty()) {
      emit CommandError(error);
    }
  });
  
  // Start the process
  process->start(program, args);
  
  if (!process->waitForStarted(5000)) {
    emit CommandError(QString("Failed to start command: %1").arg(command));
    CleanupProcess(process_id);
  }
}

void ROS2Executor::ExecuteLaunchFile(const QString& package, const QString& launch_file,
                                     const QMap<QString, QString>& parameters) {
  Q_UNUSED(package)
  Q_UNUSED(launch_file)
  Q_UNUSED(parameters)
}

void ROS2Executor::ExecuteNode(const QString& package, const QString& executable,
                               const QStringList& arguments) {
  Q_UNUSED(package)
  Q_UNUSED(executable)
  Q_UNUSED(arguments)
}

void ROS2Executor::KillProcess(const QString& process_id) {
  Q_UNUSED(process_id)
}

void ROS2Executor::KillAllProcesses() {}

void ROS2Executor::EmergencyStop() {}

void ROS2Executor::RefreshPackages() {}

void ROS2Executor::OnProcessFinished(int exit_code, QProcess::ExitStatus exit_status) {
  Q_UNUSED(exit_code)
  Q_UNUSED(exit_status)
}

void ROS2Executor::OnProcessError(QProcess::ProcessError error) {
  Q_UNUSED(error)
}

void ROS2Executor::OnProcessStarted() {}

void ROS2Executor::OnReadyReadStandardOutput() {}

void ROS2Executor::OnReadyReadStandardError() {}

void ROS2Executor::UpdateProcessList() {}

void ROS2Executor::CheckROS2Status() {}

void ROS2Executor::SetupEnvironment() {}

void ROS2Executor::SetupProcessMonitoring() {}

QString ROS2Executor::GenerateProcessId() {
  static int counter = 0;
  return QString("proc_%1_%2").arg(QDateTime::currentMSecsSinceEpoch()).arg(++counter);
}

QProcess* ROS2Executor::CreateProcess() {
  QProcess* process = new QProcess(this);
  ConfigureProcess(process);
  return process;
}

void ROS2Executor::ConfigureProcess(QProcess* process) {
  if (!process) return;
  
  // Set working directory to workspace if available
  if (!workspace_.isEmpty()) {
    process->setWorkingDirectory(workspace_);
  }
  
  // Set process environment
  QStringList env = BuildEnvironment();
  if (!env.isEmpty()) {
    process->setEnvironment(env);
  }
}

QString ROS2Executor::BuildCommand(const QString& base_command) {
  Q_UNUSED(base_command)
  return QString();
}

QStringList ROS2Executor::BuildEnvironment() {
  QStringList env = QProcess::systemEnvironment();
  
  // Add ROS2 domain ID
  env << QString("ROS_DOMAIN_ID=%1").arg(domain_id_);
  
  // Add TurtleBot3 model if set
  if (!turtlebot3_model_.isEmpty()) {
    env << QString("TURTLEBOT3_MODEL=%1").arg(turtlebot3_model_);
  }
  
  // Source ROS2 setup if workspace is set
  if (!workspace_.isEmpty()) {
    env << QString("ROS_WORKSPACE=%1").arg(workspace_);
  }
  
  return env;
}

void ROS2Executor::CleanupProcess(const QString& process_id) {
  if (!running_processes_.contains(process_id)) {
    return;
  }
  
  ProcessInfo info = running_processes_[process_id];
  if (info.process) {
    info.process->deleteLater();
  }
  
  running_processes_.remove(process_id);
  emit ProcessListChanged();
}

void ROS2Executor::ScanForPackages() {}

void ROS2Executor::ScanPackageContent(const QString& package) {
  Q_UNUSED(package)
}