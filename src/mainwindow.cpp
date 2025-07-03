// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "mainwindow.h"
#include "commandwidget.h"
#include "sshtabwidget.h"
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
#include <QNetworkInterface>
#include <QHostAddress>
#include <cstdlib>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      command_widget_(new CommandWidget(this)),
      ssh_tab_widget_(new SSHTabWidget(this)),
      ssh_manager_(new SSHManager(this)),
      ros2_executor_(new ROS2Executor(this)),
      process_monitor_(new ProcessMonitor(this)),
      config_manager_(new ConfigManager(this)),
      status_update_timer_(new QTimer(this)),
      ros_spin_timer_(new QTimer(this)),
      ssh_connected_(false),
      ssh_remote_address_("") {
  
  SetupUI();
  SetupROS2();
  SetupSSHStatusTracking();
  
  // Setup status update timer
  connect(status_update_timer_, &QTimer::timeout, this, &MainWindow::UpdateStatusInfo);
  status_update_timer_->start(5000); // Update every 5 seconds
  UpdateStatusInfo(); // Initial update
  
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
  
  // Setup status row at the top
  SetupStatusRow();
  main_layout->addWidget(status_frame_);
  
  // Create tab widget for main content
  tab_widget_ = new QTabWidget(this);
  tab_widget_->addTab(command_widget_, "Command Builder");
  tab_widget_->addTab(ssh_tab_widget_, "SSH Commands");
  
  // Create log display
  log_display_ = new QTextEdit(this);
  log_display_->setMaximumHeight(200);
  log_display_->setReadOnly(true);
  log_display_->setPlaceholderText("ROS2 log messages will appear here...");
  log_display_->setStyleSheet(
    "QTextEdit { background-color: #2b2b2b; color: #ffffff; }"
    "QToolTip { color: #ffffff; background-color: #333333; border: 1px solid #666666; padding: 4px; border-radius: 3px; }"
  );
  log_display_->setToolTip("/rosout");
  
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

void MainWindow::SetupStatusRow() {
  // Create status frame
  status_frame_ = new QFrame(this);
  status_frame_->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
  status_frame_->setFixedHeight(40);
  status_frame_->setStyleSheet(
    "QFrame { "
    "  background-color: #3b3b3b; "
    "  border: 1px solid #555555; "
    "  border-radius: 4px; "
    "}"
  );
  
  // Create horizontal layout for status frame
  QHBoxLayout* status_layout = new QHBoxLayout(status_frame_);
  status_layout->setContentsMargins(10, 5, 10, 5);
  
  // Create domain ID label
  domain_id_label_ = new QLabel("ROS_DOMAIN_ID: Loading...", this);
  domain_id_label_->setStyleSheet(
    "QLabel { "
    "  color: #ffffff; "
    "  font-weight: bold; "
    "  padding: 2px 8px; "
    "  background-color: #4a4a4a; "
    "  border-radius: 3px; "
    "}"
  );
  
  // Create IPv4 label  
  ipv4_label_ = new QLabel("IPv4: Loading...", this);
  ipv4_label_->setStyleSheet(
    "QLabel { "
    "  color: #ffffff; "
    "  font-weight: bold; "
    "  padding: 2px 8px; "
    "  background-color: #4a4a4a; "
    "  border-radius: 3px; "
    "}"
  );
  
  // Create SSH status label
  ssh_status_label_ = new QLabel("SSH: N/C", this);
  ssh_status_label_->setStyleSheet(
    "QLabel { "
    "  color: #ff6666; "
    "  font-weight: bold; "
    "  padding: 2px 8px; "
    "  background-color: #4a4a4a; "
    "  border-radius: 3px; "
    "}"
  );
  
  // Add labels to layout
  status_layout->addWidget(domain_id_label_);
  status_layout->addWidget(ipv4_label_);
  status_layout->addWidget(ssh_status_label_);
  status_layout->addStretch(); // Push labels to the left
}

void MainWindow::UpdateStatusInfo() {
  // Update ROS_DOMAIN_ID
  QString domain_id = GetRosDomainId();
  domain_id_label_->setText(QString("ROS_DOMAIN_ID: %1").arg(domain_id));
  
  // Update local IPv4 address
  QString ipv4 = GetLocalIPv4Address();
  ipv4_label_->setText(QString("IPv4: %1").arg(ipv4));
  
  // Update SSH connection status
  UpdateSSHConnectionStatus();
}

QString MainWindow::GetRosDomainId() {
  // Try to get ROS_DOMAIN_ID from environment variable
  const char* domain_id_env = std::getenv("ROS_DOMAIN_ID");
  if (domain_id_env && strlen(domain_id_env) > 0) {
    return QString(domain_id_env);
  }
  
  // Default to 0 if not set
  return "0 (default)";
}

QString MainWindow::GetLocalIPv4Address() {
  // Get all network interfaces
  QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
  
  for (const QNetworkInterface& interface : interfaces) {
    // Skip loopback and inactive interfaces
    if (interface.flags() & QNetworkInterface::IsLoopBack ||
        !(interface.flags() & QNetworkInterface::IsUp) ||
        !(interface.flags() & QNetworkInterface::IsRunning)) {
      continue;
    }
    
    // Get address entries for this interface
    QList<QNetworkAddressEntry> entries = interface.addressEntries();
    for (const QNetworkAddressEntry& entry : entries) {
      QHostAddress addr = entry.ip();
      
      // Look for IPv4 addresses that are not loopback
      if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
          !addr.isLoopback() &&
          !addr.isNull()) {
        return addr.toString();
      }
    }
  }
  
  return "Not Available";
}

void MainWindow::SetupSSHStatusTracking() {
  // Connect to SSH tab's connection status signals
  connect(ssh_tab_widget_, &SSHTabWidget::ConnectionStatusChanged,
          this, &MainWindow::OnSSHConnectionChanged);
}

void MainWindow::OnSSHConnectionChanged(bool connected) {
  qDebug() << "SSH connection changed:" << connected;
  ssh_connected_ = connected;
  
  if (connected) {
    // Extract IP address from SSH tab widget
    ssh_remote_address_ = ssh_tab_widget_->GetCurrentHost();
    qDebug() << "SSH connected to:" << ssh_remote_address_;
  } else {
    qDebug() << "SSH disconnected";
    ssh_remote_address_ = "";
  }
  
  UpdateSSHConnectionStatus();
}

void MainWindow::UpdateSSHConnectionStatus() {
  if (ssh_connected_ && !ssh_remote_address_.isEmpty()) {
    ssh_status_label_->setText(QString("SSH: %1").arg(ssh_remote_address_));
    ssh_status_label_->setStyleSheet(
      "QLabel { "
      "  color: #66ff66; "
      "  font-weight: bold; "
      "  padding: 2px 8px; "
      "  background-color: #4a4a4a; "
      "  border-radius: 3px; "
      "}"
    );
  } else {
    ssh_status_label_->setText("SSH: N/C");
    ssh_status_label_->setStyleSheet(
      "QLabel { "
      "  color: #ff6666; "
      "  font-weight: bold; "
      "  padding: 2px 8px; "
      "  background-color: #4a4a4a; "
      "  border-radius: 3px; "
      "}"
    );
  }
}