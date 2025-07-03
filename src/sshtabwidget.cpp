// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "sshtabwidget.h"
#include "configmanager.h"
#include "sshmanager.h"
#include <QRegularExpression>
#include <QDebug>
#include <QScrollBar>

SSHTabWidget::SSHTabWidget(QWidget* parent)
    : QWidget(parent),
      config_manager_(new ConfigManager(this)),
      ssh_manager_(new SSHManager(this)),
      status_update_timer_(new QTimer(this)),
      is_connected_(false),
      current_port_(22) {
  
  SetupUI();
  
  // Connect SSH manager signals
  connect(ssh_manager_, &SSHManager::Connected, 
          this, [this]() { OnSSHConnectionStatusChanged(true); });
  connect(ssh_manager_, &SSHManager::Disconnected, 
          this, [this]() { OnSSHConnectionStatusChanged(false); });
  connect(ssh_manager_, &SSHManager::CommandOutput,
          this, &SSHTabWidget::OnSSHCommandOutput);
  connect(ssh_manager_, &SSHManager::CommandFinished,
          this, &SSHTabWidget::OnSSHCommandFinished);
  connect(ssh_manager_, &SSHManager::ErrorOccurred,
          this, &SSHTabWidget::OnSSHErrorOccurred);
  
  // Setup status update timer
  connect(status_update_timer_, &QTimer::timeout, 
          this, &SSHTabWidget::UpdateConnectionStatus);
  status_update_timer_->start(2000); // Update every 2 seconds
  
  UpdateStatusLabel();
}

SSHTabWidget::~SSHTabWidget() {}

void SSHTabWidget::SetupUI() {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  
  // Setup connection area
  SetupConnectionArea();
  main_layout->addWidget(connection_group_);
  
  // Setup command rows
  SetupCommandRows();
  main_layout->addWidget(command_rows_group_);
  
  // Setup output display
  output_group_ = new QGroupBox("Command Output", this);
  QVBoxLayout* output_layout = new QVBoxLayout(output_group_);
  
  output_display_ = new QTextEdit(this);
  output_display_->setMaximumHeight(200);
  output_display_->setReadOnly(true);
  output_display_->setPlaceholderText("SSH command output will appear here...");
  output_display_->setStyleSheet(
    "QTextEdit { background-color: #2b2b2b; color: #ffffff; }"
  );
  
  output_layout->addWidget(output_display_);
  main_layout->addWidget(output_group_);
  
  main_layout->addStretch();
}

void SSHTabWidget::SetupConnectionArea() {
  connection_group_ = new QGroupBox("SSH Connection", this);
  QHBoxLayout* connection_layout = new QHBoxLayout(connection_group_);
  
  // SSH address label and input
  address_label_ = new QLabel("SSH Address:", this);
  ssh_address_edit_ = new QLineEdit(this);
  ssh_address_edit_->setPlaceholderText("user@host or user@host -p port");
  ssh_address_edit_->setText("dingo@192.168.1.235");  // Default example
  
  // Connect and disconnect buttons
  connect_button_ = new QPushButton("Connect", this);
  disconnect_button_ = new QPushButton("Disconnect", this);
  disconnect_button_->setEnabled(false);
  
  // Status label
  status_label_ = new QLabel("Disconnected", this);
  status_label_->setStyleSheet(
    "QLabel { "
    "  color: #ff6666; "
    "  font-weight: bold; "
    "  padding: 2px 8px; "
    "  background-color: #4a4a4a; "
    "  border-radius: 3px; "
    "}"
  );
  
  // Add widgets to layout
  connection_layout->addWidget(address_label_);
  connection_layout->addWidget(ssh_address_edit_, 1);
  connection_layout->addWidget(connect_button_);
  connection_layout->addWidget(disconnect_button_);
  connection_layout->addWidget(status_label_);
  
  // Connect signals
  connect(connect_button_, &QPushButton::clicked,
          this, &SSHTabWidget::OnConnectButtonClicked);
  connect(disconnect_button_, &QPushButton::clicked,
          this, &SSHTabWidget::OnDisconnectButtonClicked);
  connect(ssh_address_edit_, &QLineEdit::textChanged,
          this, &SSHTabWidget::OnSSHAddressChanged);
}

void SSHTabWidget::SetupCommandRows() {
  command_rows_group_ = new QGroupBox("SSH Commands", this);
  QGridLayout* grid_layout = new QGridLayout(command_rows_group_);
  
  // Create 2 rows of controls as requested
  for (int row = 0; row < 2; ++row) {
    CommandRow command_row;
    
    // Create button
    command_row.button = new QPushButton(QString("Execute Row %1").arg(row + 1));
    command_row.button->setEnabled(false); // Disabled until connected
    
    // Create first dropdown
    command_row.dropdown1 = new QComboBox();
    command_row.dropdown1->setEditable(true);
    command_row.dropdown1->addItems({"ls", "cd", "pwd", "ps", "top", "df"});
    command_row.dropdown1->setCurrentText(row == 0 ? "ls" : "ps");
    
    // Create second dropdown  
    command_row.dropdown2 = new QComboBox();
    command_row.dropdown2->setEditable(true);
    command_row.dropdown2->addItems({"-la", "-aux", "-h", "--help", ""});
    command_row.dropdown2->setCurrentText(row == 0 ? "-la" : "-aux");
    
    // Add to grid layout
    grid_layout->addWidget(command_row.button, row, 0);
    grid_layout->addWidget(command_row.dropdown1, row, 1);
    grid_layout->addWidget(command_row.dropdown2, row, 2);
    
    // Connect button signal
    connect(command_row.button, &QPushButton::clicked, [this, row]() {
      OnCommandRowButtonClicked(row);
    });
    
    command_rows_.append(command_row);
  }
}

void SSHTabWidget::OnConnectButtonClicked() {
  QString address = ssh_address_edit_->text().trimmed();
  if (address.isEmpty()) {
    output_display_->append("Error: Please enter an SSH address");
    return;
  }
  
  if (!ValidateSSHAddress(address)) {
    output_display_->append("Error: Invalid SSH address format");
    return;
  }
  
  QString user, host;
  int port;
  ParseSSHAddress(address, user, host, port);
  
  current_user_ = user;
  current_host_ = host;
  current_port_ = port;
  
  output_display_->append(QString("Connecting to %1@%2:%3...").arg(user, host).arg(port));
  
  // Configure SSH manager
  ssh_manager_->SetConnectionParams(host, user, port);
  ssh_manager_->ConnectToHost();
  
  connect_button_->setEnabled(false);
  status_label_->setText("Connecting...");
  status_label_->setStyleSheet(
    "QLabel { "
    "  color: #ffaa00; "
    "  font-weight: bold; "
    "  padding: 2px 8px; "
    "  background-color: #4a4a4a; "
    "  border-radius: 3px; "
    "}"
  );
}

void SSHTabWidget::OnDisconnectButtonClicked() {
  ssh_manager_->DisconnectFromHost();
  output_display_->append("Disconnecting...");
}

void SSHTabWidget::OnSSHAddressChanged() {
  // Enable/disable connect button based on address validity
  bool valid = ValidateSSHAddress(ssh_address_edit_->text().trimmed());
  connect_button_->setEnabled(valid && !is_connected_);
}

void SSHTabWidget::OnCommandRowButtonClicked(int row) {
  if (!is_connected_ || row >= command_rows_.size()) {
    return;
  }
  
  const CommandRow& command_row = command_rows_[row];
  QString command1 = command_row.dropdown1->currentText().trimmed();
  QString command2 = command_row.dropdown2->currentText().trimmed();
  
  QString full_command = command1;
  if (!command2.isEmpty()) {
    full_command += " " + command2;
  }
  
  output_display_->append(QString("$ %1").arg(full_command));
  ssh_manager_->ExecuteCommand(full_command);
}

void SSHTabWidget::OnSSHConnectionStatusChanged(bool connected) {
  is_connected_ = connected;
  
  connect_button_->setEnabled(!connected && ValidateSSHAddress(ssh_address_edit_->text()));
  disconnect_button_->setEnabled(connected);
  
  // Enable/disable command buttons
  for (CommandRow& row : command_rows_) {
    row.button->setEnabled(connected);
  }
  
  UpdateStatusLabel();
  emit ConnectionStatusChanged(connected);
  
  if (connected) {
    output_display_->append(QString("Connected to %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
  } else {
    output_display_->append("Disconnected from SSH");
  }
}

void SSHTabWidget::OnSSHCommandOutput(const QString& output) {
  output_display_->append(output);
  
  // Auto-scroll to bottom
  QScrollBar* scrollbar = output_display_->verticalScrollBar();
  scrollbar->setValue(scrollbar->maximum());
}

void SSHTabWidget::OnSSHCommandFinished(const QString& command, int exit_code) {
  output_display_->append(QString("Command '%1' finished with exit code: %2").arg(command).arg(exit_code));
}

void SSHTabWidget::OnSSHErrorOccurred(const QString& error) {
  output_display_->append(QString("SSH Error: %1").arg(error));
}

void SSHTabWidget::UpdateConnectionStatus() {
  UpdateStatusLabel();
}

void SSHTabWidget::UpdateStatusLabel() {
  if (is_connected_) {
    status_label_->setText("Connected");
    status_label_->setStyleSheet(
      "QLabel { "
      "  color: #66ff66; "
      "  font-weight: bold; "
      "  padding: 2px 8px; "
      "  background-color: #4a4a4a; "
      "  border-radius: 3px; "
      "}"
    );
  } else {
    status_label_->setText("Disconnected");
    status_label_->setStyleSheet(
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

void SSHTabWidget::ParseSSHAddress(const QString& address, QString& user, 
                                   QString& host, int& port) {
  port = 22; // Default SSH port
  
  // Handle formats: user@host, user@host -p port
  QRegularExpression regex(R"(^([^@]+)@([^\s]+)(?:\s+-p\s+(\d+))?$)");
  QRegularExpressionMatch match = regex.match(address);
  
  if (match.hasMatch()) {
    user = match.captured(1);
    host = match.captured(2);
    if (!match.captured(3).isEmpty()) {
      port = match.captured(3).toInt();
    }
  }
}

bool SSHTabWidget::ValidateSSHAddress(const QString& address) {
  if (address.isEmpty()) {
    return false;
  }
  
  // Validate format: user@host or user@host -p port
  QRegularExpression regex(R"(^[^@\s]+@[^@\s]+(?:\s+-p\s+\d+)?$)");
  return regex.match(address).hasMatch();
}