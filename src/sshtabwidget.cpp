// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "sshtabwidget.h"
#include "configmanager.h"
#include "sshmanager.h"
#include "sshpassworddialog.h"
#include <QRegularExpression>
#include <QDebug>
#include <QScrollBar>
#include <toml++/toml.hpp>

SSHTabWidget::SSHTabWidget(QWidget* parent)
    : QWidget(parent),
      config_manager_(new ConfigManager(this)),
      ssh_manager_(new SSHManager(this)),
      status_update_timer_(new QTimer(this)),
      is_connected_(false),
      current_port_(22) {
  
  SetupUI();
  LoadDefaultsFromConfig();
  
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

QString SSHTabWidget::GetCurrentHost() const {
  return current_host_;
}

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
  // Default will be set from config file
  
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
  
  // Allow Enter key to trigger connect when address field has focus
  connect(ssh_address_edit_, &QLineEdit::returnPressed, [this]() {
    if (connect_button_->isEnabled() && !ssh_address_edit_->text().trimmed().isEmpty()) {
      OnConnectButtonClicked();
    }
  });
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
    // Options will be loaded from config file
    
    // Create second dropdown  
    command_row.dropdown2 = new QComboBox();
    command_row.dropdown2->setEditable(true);
    // Options will be loaded from config file
    
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
  
  // Show password dialog
  SSHPasswordDialog* password_dialog = new SSHPasswordDialog(user, host, port, this);
  
  if (password_dialog->exec() == QDialog::Accepted) {
    QString password = password_dialog->GetPassword();
    
    // Output raw SSH connection information
    output_display_->append("=== SSH Connection Details ===");
    output_display_->append(QString("SSH Address: %1").arg(address));
    output_display_->append(QString("Parsed - User: %1, Host: %2, Port: %3").arg(user, host).arg(port));
    output_display_->append(QString("Password: %1").arg(password.isEmpty() ? "[Empty]" : QString("[%1 characters]").arg(password.length())));
    output_display_->append(QString("SSH Command: ssh %1@%2 -p %3").arg(user, host).arg(port));
    output_display_->append("=============================");
    output_display_->append(QString("Connecting to %1@%2:%3...").arg(user, host).arg(port));
    
    // Configure SSH manager with password
    ssh_manager_->SetConnectionParams(host, user, port, password);
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
  } else {
    output_display_->append("Connection cancelled by user");
  }
  
  password_dialog->deleteLater();
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
  
  output_display_->append(QString("=== Executing SSH Command ==="));
  output_display_->append(QString("Command: %1").arg(full_command));
  output_display_->append(QString("Row: %1").arg(row + 1));
  output_display_->append(QString("SSH Target: %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
  output_display_->append("=============================");
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
    output_display_->append("=== SSH Connection Established ===");
    output_display_->append(QString("Successfully connected to %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
    output_display_->append(QString("SSH session active - ready to execute commands"));
    output_display_->append("================================");
  } else {
    output_display_->append("=== SSH Connection Closed ===");
    output_display_->append("Disconnected from SSH server");
    output_display_->append("Command execution disabled");
    output_display_->append("=============================");
  }
}

void SSHTabWidget::OnSSHCommandOutput(const QString& output) {
  output_display_->append(output);
  
  // Auto-scroll to bottom
  QScrollBar* scrollbar = output_display_->verticalScrollBar();
  scrollbar->setValue(scrollbar->maximum());
}

void SSHTabWidget::OnSSHCommandFinished(const QString& command, int exit_code) {
  output_display_->append("=== Command Execution Complete ===");
  output_display_->append(QString("Command: %1").arg(command));
  output_display_->append(QString("Exit Code: %1").arg(exit_code));
  output_display_->append(QString("Status: %1").arg(exit_code == 0 ? "SUCCESS" : "FAILED"));
  output_display_->append("================================");
}

void SSHTabWidget::OnSSHErrorOccurred(const QString& error) {
  output_display_->append("=== SSH ERROR OCCURRED ===");
  output_display_->append(QString("Error: %1").arg(error));
  output_display_->append(QString("Target: %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
  output_display_->append("=========================");
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

void SSHTabWidget::LoadDefaultsFromConfig() {
  try {
    // Try to load config from file
    toml::table config;
    try {
      config = toml::parse_file("../settings.toml");
    } catch (const toml::parse_error& err) {
      try {
        config = toml::parse_file("settings.toml");
      } catch (const toml::parse_error& err2) {
        qWarning() << "Failed to load settings.toml:" << err2.what();
        return;
      }
    }
    
    // Load SSH address default
    if (auto ssh_section = config["ssh"].as_table()) {
      if (auto default_address = ssh_section->get("default_address")->as_string()) {
        QString address = QString::fromStdString(default_address->get()).remove('"');
        ssh_address_edit_->setText(address);
      }
      
      // Load dropdown options
      QStringList dropdown1_options;
      QStringList dropdown2_options;
      
      if (auto dropdown1_array = ssh_section->get("dropdown1_options")->as_array()) {
        for (auto&& elem : *dropdown1_array) {
          if (auto str = elem.as_string()) {
            dropdown1_options << QString::fromStdString(str->get()).remove('"');
          }
        }
      }
      
      if (auto dropdown2_array = ssh_section->get("dropdown2_options")->as_array()) {
        for (auto&& elem : *dropdown2_array) {
          if (auto str = elem.as_string()) {
            dropdown2_options << QString::fromStdString(str->get()).remove('"');
          }
        }
      }
      
      // Set fallback options if not found in config
      if (dropdown1_options.isEmpty()) {
        dropdown1_options = {"ls", "cd", "pwd", "ps", "top", "df"};
      }
      if (dropdown2_options.isEmpty()) {
        dropdown2_options = {"-la", "-aux", "-h", "--help", ""};
      }
      
      // Populate dropdown options for all command rows
      for (CommandRow& row : command_rows_) {
        row.dropdown1->addItems(dropdown1_options);
        row.dropdown2->addItems(dropdown2_options);
      }
      
      // Load command defaults (after options are populated)
      if (auto default_commands = ssh_section->get("default_commands")->as_array()) {
        for (size_t i = 0; i < default_commands->size() && i < command_rows_.size(); ++i) {
          if (auto cmd_array = default_commands->get(i)->as_array()) {
            if (cmd_array->size() >= 1) {
              if (auto cmd1 = cmd_array->get(0)->as_string()) {
                QString command1 = QString::fromStdString(cmd1->get()).remove('"');
                command_rows_[i].dropdown1->setCurrentText(command1);
              }
            }
            if (cmd_array->size() >= 2) {
              if (auto cmd2 = cmd_array->get(1)->as_string()) {
                QString command2 = QString::fromStdString(cmd2->get()).remove('"');
                command_rows_[i].dropdown2->setCurrentText(command2);
              }
            }
          }
        }
      }
    }
    
  } catch (const std::exception& e) {
    qWarning() << "Error loading SSH defaults from config:" << e.what();
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