// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// Main application window providing tabbed interface for TurtleBot3 control
// with SSH management, ROS2 execution, and process monitoring capabilities.

#ifndef ROS_SYNAPSE_INCLUDE_MAINWINDOW_H_
#define ROS_SYNAPSE_INCLUDE_MAINWINDOW_H_

#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QTextEdit>
#include <QStatusBar>
#include <QMenuBar>
#include <QAction>
#include <QTimer>
#include <QLabel>
#include <QFrame>
#include <QNetworkInterface>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

class CommandWidget;
class SSHTabWidget;
class SSHManager;
class ROS2Executor;
class ProcessMonitor;
class ConfigManager;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private slots:
  void OnConnectionStatusChanged(bool connected);
  void OnCommandExecuted(const QString& command, const QString& output);
  void OnProcessStatusChanged(const QString& process, const QString& status);
  void UpdateStatusBar();
  void ShowAbout();
  void ShowSettings();
  void OnLogMessageReceived(const rcl_interfaces::msg::Log::SharedPtr msg);
  void OnSSHConnectionChanged(bool connected);

 private:
  void SetupUI();
  void SetupStatusRow();
  void SetupMenuBar();
  void SetupStatusBar();
  void SetupConnections();
  void SetupTheme();
  void SetupROS2();
  void SetupSSHStatusTracking();
  void SetupCommandExecution();
  void OnCommandReady(const QString& command);
  void AppendLogMessage(const QString& message, const QString& level);
  void UpdateStatusInfo();
  void UpdateSSHConnectionStatus();
  QString GetLocalIPv4Address();
  QString GetRosDomainId();

  QTabWidget* tab_widget_;
  QWidget* quick_launch_tab_;
  CommandWidget* command_widget_;
  SSHTabWidget* ssh_tab_widget_;
  QWidget* terminal_tab_;
  QWidget* monitor_tab_;
  QWidget* config_tab_;
  
  QTextEdit* output_display_;
  QTextEdit* log_display_;
  QSplitter* main_splitter_;
  QWidget* central_widget_;
  
  // Status row widgets
  QFrame* status_frame_;
  QLabel* domain_id_label_;
  QLabel* ipv4_label_;
  QLabel* ssh_status_label_;
  QTimer* status_update_timer_;
  
  // SSH connection tracking
  bool ssh_connected_;
  QString ssh_remote_address_;
  
  SSHManager* ssh_manager_;
  ROS2Executor* ros2_executor_;
  ProcessMonitor* process_monitor_;
  ConfigManager* config_manager_;
  
  QTimer* status_timer_;
  QStatusBar* status_bar_;
  
  QAction* connect_action_;
  QAction* disconnect_action_;
  QAction* settings_action_;
  QAction* about_action_;
  QAction* quit_action_;
  
  // ROS2 components
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  QTimer* ros_spin_timer_;
};

#endif  // ROS_SYNAPSE_INCLUDE_MAINWINDOW_H_