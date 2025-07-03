// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// ROS2 command executor for local and remote execution of ROS2 nodes,
// launch files, and process management with environment configuration.

#ifndef ROS_SYNAPSE_INCLUDE_ROS2EXECUTOR_H_
#define ROS_SYNAPSE_INCLUDE_ROS2EXECUTOR_H_

#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QMap>
#include <QStringList>
#include <QMutex>
#include <QQueue>
#include <QDateTime>

class ROS2Executor : public QObject {
  Q_OBJECT

 public:
  enum ExecutionMode {
    kLocal,
    kRemote
  };

  struct ProcessInfo {
    QProcess* process;
    QString command;
    QString package;
    QString executable;
    QDateTime start_time;
    int pid;
    bool is_running;
  };

  explicit ROS2Executor(QObject* parent = nullptr);
  ~ROS2Executor();

  void SetExecutionMode(ExecutionMode mode);
  void SetROS2Environment(const QString& workspace, int domain_id = 0);
  void SetTurtleBot3Model(const QString& model);
  void SetRemoteConnection(const QString& host, const QString& username);
  
  bool IsROS2Available() const;
  QStringList GetAvailablePackages() const;
  QStringList GetPackageExecutables(const QString& package) const;
  QStringList GetPackageLaunchFiles(const QString& package) const;
  QMap<QString, ProcessInfo> GetRunningProcesses() const;

 public slots:
  void ExecuteCommand(const QString& command);
  void ExecuteLaunchFile(const QString& package, const QString& launch_file, 
                         const QMap<QString, QString>& parameters = 
                             QMap<QString, QString>());
  void ExecuteNode(const QString& package, const QString& executable,
                   const QStringList& arguments = QStringList());
  void KillProcess(const QString& process_id);
  void KillAllProcesses();
  void EmergencyStop();
  void RefreshPackages();

 signals:
  void CommandStarted(const QString& command, const QString& process_id);
  void CommandOutput(const QString& process_id, const QString& output);
  void CommandFinished(const QString& process_id, int exit_code);
  void CommandError(const QString& process_id, const QString& error);
  void ProcessListChanged();
  void PackageListChanged();
  void ROS2StatusChanged(bool available);

 private slots:
  void OnProcessFinished(int exit_code, QProcess::ExitStatus exit_status);
  void OnProcessError(QProcess::ProcessError error);
  void OnProcessStarted();
  void OnReadyReadStandardOutput();
  void OnReadyReadStandardError();
  void UpdateProcessList();
  void CheckROS2Status();

 private:
  void SetupEnvironment();
  void SetupProcessMonitoring();
  QString GenerateProcessId();
  QProcess* CreateProcess();
  void ConfigureProcess(QProcess* process);
  QString BuildCommand(const QString& base_command);
  QStringList BuildEnvironment();
  void CleanupProcess(const QString& process_id);
  void ScanForPackages();
  void ScanPackageContent(const QString& package);
  
  ExecutionMode execution_mode_;
  QString workspace_;
  int domain_id_;
  QString turtlebot3_model_;
  QString remote_host_;
  QString remote_username_;
  
  QMap<QString, ProcessInfo> running_processes_;
  QMap<QString, QStringList> package_executables_;
  QMap<QString, QStringList> package_launch_files_;
  QStringList available_packages_;
  
  QTimer* process_monitor_timer_;
  QTimer* ros2_status_timer_;
  QMutex process_mutex_;
  
  bool ros2_available_;
  QString ros2_path_;
  QStringList environment_;
  
  static const int kProcessMonitorInterval = 5000;
  static const int kROS2StatusCheckInterval = 30000;
};

#endif  // ROS_SYNAPSE_INCLUDE_ROS2EXECUTOR_H_