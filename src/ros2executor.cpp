// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "ros2executor.h"
#include <QDir>
#include <QFile>

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

void ROS2Executor::SetWorkingDirectory(const QString& directory) {
  working_directory_ = directory;
  qDebug() << "ROS2Executor working directory set to:" << working_directory_;
}

void ROS2Executor::SetSetupBashFile(const QString& setup_file) {
  setup_bash_file_ = setup_file;
  qDebug() << "ROS2Executor setup.bash file set to:" << setup_bash_file_;
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
  
  // Build the final command with setup.bash sourcing if available
  QString final_command = command;
  
  // If setup.bash file is set, wrap command to source it first
  if (!setup_bash_file_.isEmpty()) {
    QString expanded_setup = setup_bash_file_;
    // Expand ~ to home directory
    if (expanded_setup.startsWith("~/")) {
      expanded_setup = QDir::homePath() + expanded_setup.mid(1);
    }
    
    // Check if setup.bash file exists
    if (QFile::exists(expanded_setup)) {
      final_command = QString("bash -c \"source %1 && %2\"")
                      .arg(expanded_setup)
                      .arg(command);
    } else {
      emit CommandOutput(QString("Warning: Setup.bash file not found: %1").arg(expanded_setup));
    }
  }
  
  // Parse final command - handle GUI applications that need to be detached
  QStringList args = final_command.split(' ', Qt::SkipEmptyParts);
  if (args.isEmpty()) {
    delete process;
    running_processes_.remove(process_id);
    emit CommandError("Empty command");
    return;
  }
  
  QString program = args.takeFirst();
  
  // For GUI applications like rviz2, use detached process
  if (command.contains("rviz2") || command.contains("rviz") || 
      command.contains("gazebo") || command.contains("rqt")) {
    
    emit CommandOutput(QString("Starting GUI application: %1").arg(command));
    emit CommandStarted(command, process_id);
    
    // For GUI applications, use the final_command with setup.bash if available
    bool started;
    if (final_command != command) {
      // Using setup.bash wrapper - need to execute through shell
      started = QProcess::startDetached("bash", QStringList() << "-c" << final_command);
    } else {
      // Direct execution
      started = QProcess::startDetached(program, args);
    }
    
    if (started) {
      emit CommandOutput(QString("GUI application started successfully: %1").arg(command));
    } else {
      emit CommandError(QString("Failed to start GUI application: %1").arg(command));
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
  
  // Start the process - use shell execution if setup.bash is being used
  if (final_command != command) {
    // Using setup.bash wrapper - execute through shell
    process->start("bash", QStringList() << "-c" << final_command);
  } else {
    // Direct execution
    process->start(program, args);
  }
  
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
  if (!running_processes_.contains(process_id)) {
    qWarning() << "Process not found:" << process_id;
    return;
  }
  
  ProcessInfo info = running_processes_[process_id];
  if (info.process && info.process->state() == QProcess::Running) {
    qDebug() << "Terminating process:" << process_id;
    
    // Send SIGTERM first (equivalent to Ctrl+C)
    info.process->terminate();
    
    // Give it 3 seconds to terminate gracefully
    if (!info.process->waitForFinished(3000)) {
      qDebug() << "Process didn't terminate gracefully, killing forcefully";
      info.process->kill();
      info.process->waitForFinished(1000);
    }
    
    emit CommandFinished(process_id, info.process->exitCode());
  } else {
    qDebug() << "Process" << process_id << "is not running";
  }
  
  CleanupProcess(process_id);
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
  
  // Set working directory - prefer working_directory_ over workspace_
  QString work_dir;
  if (!working_directory_.isEmpty()) {
    work_dir = working_directory_;
    // Expand ~ to home directory
    if (work_dir.startsWith("~/")) {
      work_dir = QDir::homePath() + work_dir.mid(1);
    }
  } else if (!workspace_.isEmpty()) {
    work_dir = workspace_;
  }
  
  if (!work_dir.isEmpty()) {
    process->setWorkingDirectory(work_dir);
    qDebug() << "Process working directory set to:" << work_dir;
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