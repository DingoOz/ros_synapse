// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// Process monitoring system for tracking ROS2 nodes and system processes
// with real-time resource usage monitoring and tree widget display.

#ifndef ROS_SYNAPSE_INCLUDE_PROCESSMONITOR_H_
#define ROS_SYNAPSE_INCLUDE_PROCESSMONITOR_H_

#include <QObject>
#include <QTimer>
#include <QProcess>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QMap>
#include <QStringList>
#include <QDateTime>

class ProcessMonitor : public QObject {
  Q_OBJECT

 public:
  struct ProcessData {
    QString name;
    int pid;
    QString status;
    double cpu_usage;
    double memory_usage;
    QDateTime start_time;
    QString command;
    QString user;
    bool is_ros2_node;
    QString node_name;
    QString namespace_;
  };

  explicit ProcessMonitor(QObject* parent = nullptr);
  ~ProcessMonitor();

  void SetTreeWidget(QTreeWidget* tree_widget);
  void SetUpdateInterval(int milliseconds);
  void SetRemoteMonitoring(bool enabled, const QString& host = QString(), 
                           const QString& username = QString());
  
  QMap<int, ProcessData> GetProcessList() const;
  QStringList GetROS2Nodes() const;
  bool IsProcessRunning(int pid) const;
  ProcessData GetProcessData(int pid) const;

 public slots:
  void StartMonitoring();
  void StopMonitoring();
  void RefreshProcessList();
  void KillProcess(int pid);
  void KillProcessTree(int pid);
  void SetProcessPriority(int pid, int priority);
  void FilterByROS2Nodes(bool show_only_ros2);
  void FilterByUser(const QString& user);
  void FilterByName(const QString& name);

 signals:
  void ProcessStarted(int pid, const QString& name);
  void ProcessFinished(int pid, const QString& name);
  void ProcessDataChanged(int pid, const ProcessData& data);
  void ProcessListUpdated();
  void ROS2NodeDetected(const QString& node_name, const QString& namespace_);
  void ROS2NodeLost(const QString& node_name, const QString& namespace_);

 private slots:
  void UpdateProcessList();
  void OnProcessScanFinished();
  void OnProcessScanError();
  void OnROS2NodeScanFinished();

 private:
  void SetupProcessScanning();
  void ScanProcesses();
  void ScanROS2Nodes();
  void ParseProcessOutput(const QString& output);
  void ParseROS2NodeOutput(const QString& output);
  void UpdateTreeWidget();
  void AddProcessToTree(const ProcessData& process_data);
  void RemoveProcessFromTree(int pid);
  void UpdateProcessInTree(int pid, const ProcessData& process_data);
  QTreeWidgetItem* FindTreeItem(int pid);
  QString FormatMemoryUsage(double memory_kb);
  QString FormatCPUUsage(double cpu_percent);
  QString FormatUptime(const QDateTime& start_time);
  bool MatchesFilters(const ProcessData& process_data);
  
  QTimer* update_timer_;
  QProcess* process_scanner_;
  QProcess* ros2_node_scanner_;
  QTreeWidget* tree_widget_;
  
  QMap<int, ProcessData> process_map_;
  QMap<int, ProcessData> previous_process_map_;
  QStringList ros2_nodes_;
  QStringList previous_ros2_nodes_;
  
  bool is_monitoring_;
  bool remote_monitoring_;
  QString remote_host_;
  QString remote_username_;
  
  int update_interval_;
  bool show_only_ros2_;
  QString user_filter_;
  QString name_filter_;
  
  QStringList tree_headers_;
  
  static const int kDefaultUpdateInterval = 2000;
  static const QStringList kDefaultTreeHeaders;
};

#endif  // ROS_SYNAPSE_INCLUDE_PROCESSMONITOR_H_