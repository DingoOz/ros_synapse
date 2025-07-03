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
  Q_UNUSED(command)
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
  return QString();
}

QProcess* ROS2Executor::CreateProcess() {
  return nullptr;
}

void ROS2Executor::ConfigureProcess(QProcess* process) {
  Q_UNUSED(process)
}

QString ROS2Executor::BuildCommand(const QString& base_command) {
  Q_UNUSED(base_command)
  return QString();
}

QStringList ROS2Executor::BuildEnvironment() {
  return QStringList();
}

void ROS2Executor::CleanupProcess(const QString& process_id) {
  Q_UNUSED(process_id)
}

void ROS2Executor::ScanForPackages() {}

void ROS2Executor::ScanPackageContent(const QString& package) {
  Q_UNUSED(package)
}