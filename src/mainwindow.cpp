// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "mainwindow.h"
#include "commandwidget.h"
#include "sshmanager.h"
#include "ros2executor.h"
#include "processmonitor.h"
#include "configmanager.h"
#include <QApplication>
#include <QVBoxLayout>
#include <QSplitter>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QScrollBar>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      command_widget_(new CommandWidget(this)),
      ssh_manager_(new SSHManager(this)),
      ros2_executor_(new ROS2Executor(this)),
      process_monitor_(new ProcessMonitor(this)),
      config_manager_(new ConfigManager(this)),
      ros_spin_timer_(new QTimer(this)) {
  
  SetupUI();
  SetupROS2();
  setWindowTitle("ROS Synapse");
  resize(1200, 800);
}

MainWindow::~MainWindow() {
  if (executor_) {
    executor_->cancel();
  }
}

void MainWindow::OnConnectionStatusChanged(bool connected) {
  Q_UNUSED(connected)
}

void MainWindow::OnCommandExecuted(const QString& command, const QString& output) {
  Q_UNUSED(command)
  Q_UNUSED(output)
}

void MainWindow::OnProcessStatusChanged(const QString& process, const QString& status) {
  Q_UNUSED(process)
  Q_UNUSED(status)
}

void MainWindow::UpdateStatusBar() {}

void MainWindow::ShowAbout() {}

void MainWindow::ShowSettings() {}

void MainWindow::SetupMenuBar() {}

void MainWindow::SetupStatusBar() {}

void MainWindow::SetupConnections() {}

void MainWindow::SetupTheme() {}

void MainWindow::SetupUI() {
  // Create central widget and main layout
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);
  
  QVBoxLayout* main_layout = new QVBoxLayout(central_widget_);
  
  // Create tab widget for main content
  tab_widget_ = new QTabWidget(this);
  tab_widget_->addTab(command_widget_, "Command Builder");
  
  // Create log display
  log_display_ = new QTextEdit(this);
  log_display_->setMaximumHeight(200);
  log_display_->setReadOnly(true);
  log_display_->setPlaceholderText("ROS2 log messages will appear here...");
  log_display_->setStyleSheet("QTextEdit { background-color: #2b2b2b; color: #ffffff; }");
  
  // Add widgets to main layout
  main_layout->addWidget(tab_widget_, 1); // Tab widget takes most space
  main_layout->addWidget(log_display_);   // Log display at bottom
}

void MainWindow::SetupROS2() {
  try {
    // Initialize ROS2 if not already initialized
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
    
    // Create ROS2 node
    ros_node_ = rclcpp::Node::make_shared("ros_synapse_gui");
    
    // Create executor
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);
    
    // Create log subscription
    log_subscription_ = ros_node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 10, 
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        OnLogMessageReceived(msg);
      });
    
    // Setup timer to spin ROS2 executor
    connect(ros_spin_timer_, &QTimer::timeout, [this]() {
      if (executor_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });
    ros_spin_timer_->start(50); // Spin every 50ms
    
  } catch (const std::exception& e) {
    qWarning() << "Failed to initialize ROS2:" << e.what();
  }
}

void MainWindow::OnLogMessageReceived(const rcl_interfaces::msg::Log::SharedPtr msg) {
  QString level;
  switch (msg->level) {
    case rcl_interfaces::msg::Log::DEBUG:
      level = "DEBUG";
      break;
    case rcl_interfaces::msg::Log::INFO:
      level = "INFO";
      break;
    case rcl_interfaces::msg::Log::WARN:
      level = "WARN";
      break;
    case rcl_interfaces::msg::Log::ERROR:
      level = "ERROR";
      break;
    case rcl_interfaces::msg::Log::FATAL:
      level = "FATAL";
      break;
    default:
      level = "UNKNOWN";
      break;
  }
  
  QString timestamp = QString::number(msg->stamp.sec) + "." + QString::number(msg->stamp.nanosec);
  QString message = QString("[%1] [%2] %3: %4")
                   .arg(timestamp)
                   .arg(QString::fromStdString(msg->name))
                   .arg(level)
                   .arg(QString::fromStdString(msg->msg));
  
  AppendLogMessage(message, level);
}

void MainWindow::AppendLogMessage(const QString& message, const QString& level) {
  QTextCursor cursor = log_display_->textCursor();
  cursor.movePosition(QTextCursor::End);
  
  // Set text format based on log level
  QTextCharFormat format;
  if (level == "ERROR" || level == "FATAL") {
    format.setForeground(QColor(255, 100, 100)); // Red for errors
  } else if (level == "WARN") {
    format.setForeground(QColor(255, 255, 100)); // Yellow for warnings
  } else {
    format.setForeground(QColor(255, 255, 255)); // White for others
  }
  
  cursor.setCharFormat(format);
  cursor.insertText(message + "\n");
  
  // Auto-scroll to bottom
  QScrollBar* scrollbar = log_display_->verticalScrollBar();
  scrollbar->setValue(scrollbar->maximum());
  
  // Limit log buffer size (keep last 1000 lines)
  if (log_display_->document()->lineCount() > 1000) {
    cursor.movePosition(QTextCursor::Start);
    cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, 100);
    cursor.removeSelectedText();
  }
}