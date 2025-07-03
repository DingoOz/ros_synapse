// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "processmonitor.h"

const QStringList ProcessMonitor::kDefaultTreeHeaders = {
  "Name", "PID", "Status", "CPU %", "Memory", "Start Time", "Command"
};

ProcessMonitor::ProcessMonitor(QObject* parent)
    : QObject(parent),
      tree_widget_(nullptr),
      is_monitoring_(false),
      remote_monitoring_(false),
      update_interval_(kDefaultUpdateInterval),
      show_only_ros2_(false) {}

ProcessMonitor::~ProcessMonitor() {}

void ProcessMonitor::SetTreeWidget(QTreeWidget* tree_widget) {
  tree_widget_ = tree_widget;
}

void ProcessMonitor::SetUpdateInterval(int milliseconds) {
  update_interval_ = milliseconds;
}

void ProcessMonitor::SetRemoteMonitoring(bool enabled, const QString& host,
                                         const QString& username) {
  remote_monitoring_ = enabled;
  remote_host_ = host;
  remote_username_ = username;
}

QMap<int, ProcessMonitor::ProcessData> ProcessMonitor::GetProcessList() const {
  return process_map_;
}

QStringList ProcessMonitor::GetROS2Nodes() const {
  return ros2_nodes_;
}

bool ProcessMonitor::IsProcessRunning(int pid) const {
  return process_map_.contains(pid);
}

ProcessMonitor::ProcessData ProcessMonitor::GetProcessData(int pid) const {
  return process_map_.value(pid);
}

void ProcessMonitor::StartMonitoring() {
  is_monitoring_ = true;
}

void ProcessMonitor::StopMonitoring() {
  is_monitoring_ = false;
}

void ProcessMonitor::RefreshProcessList() {}

void ProcessMonitor::KillProcess(int pid) {
  Q_UNUSED(pid)
}

void ProcessMonitor::KillProcessTree(int pid) {
  Q_UNUSED(pid)
}

void ProcessMonitor::SetProcessPriority(int pid, int priority) {
  Q_UNUSED(pid)
  Q_UNUSED(priority)
}

void ProcessMonitor::FilterByROS2Nodes(bool show_only_ros2) {
  show_only_ros2_ = show_only_ros2;
}

void ProcessMonitor::FilterByUser(const QString& user) {
  user_filter_ = user;
}

void ProcessMonitor::FilterByName(const QString& name) {
  name_filter_ = name;
}

void ProcessMonitor::UpdateProcessList() {}

void ProcessMonitor::OnProcessScanFinished() {}

void ProcessMonitor::OnProcessScanError() {}

void ProcessMonitor::OnROS2NodeScanFinished() {}

void ProcessMonitor::SetupProcessScanning() {}

void ProcessMonitor::ScanProcesses() {}

void ProcessMonitor::ScanROS2Nodes() {}

void ProcessMonitor::ParseProcessOutput(const QString& output) {
  Q_UNUSED(output)
}

void ProcessMonitor::ParseROS2NodeOutput(const QString& output) {
  Q_UNUSED(output)
}

void ProcessMonitor::UpdateTreeWidget() {}

void ProcessMonitor::AddProcessToTree(const ProcessData& process_data) {
  Q_UNUSED(process_data)
}

void ProcessMonitor::RemoveProcessFromTree(int pid) {
  Q_UNUSED(pid)
}

void ProcessMonitor::UpdateProcessInTree(int pid, const ProcessData& process_data) {
  Q_UNUSED(pid)
  Q_UNUSED(process_data)
}

QTreeWidgetItem* ProcessMonitor::FindTreeItem(int pid) {
  Q_UNUSED(pid)
  return nullptr;
}

QString ProcessMonitor::FormatMemoryUsage(double memory_kb) {
  Q_UNUSED(memory_kb)
  return QString();
}

QString ProcessMonitor::FormatCPUUsage(double cpu_percent) {
  Q_UNUSED(cpu_percent)
  return QString();
}

QString ProcessMonitor::FormatUptime(const QDateTime& start_time) {
  Q_UNUSED(start_time)
  return QString();
}

bool ProcessMonitor::MatchesFilters(const ProcessData& process_data) {
  Q_UNUSED(process_data)
  return true;
}