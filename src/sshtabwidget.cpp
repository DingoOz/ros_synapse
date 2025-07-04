// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "sshtabwidget.h"
#include "configmanager.h"
#include "sshmanager.h"
#include "sshpassworddialog.h"
#include <QRegularExpression>
#include <QDebug>
#include <QScrollBar>
#include <QDateTime>
#include <toml++/toml.hpp>

SSHTabWidget::SSHTabWidget(QWidget* parent)
    : QWidget(parent),
      config_manager_(new ConfigManager(this)),
      ssh_manager_(new SSHManager(this)),
      status_update_timer_(new QTimer(this)),
      connection_poll_timer_(new QTimer(this)),
      is_connected_(false),
      current_port_(22),
      poll_enabled_(false),
      poll_interval_seconds_(5),
      poll_timeout_seconds_(3),
      poll_failure_count_(0),
      max_poll_failures_(3) {
  
  SetupUI();
  LoadDefaultsFromConfig();
  LoadPollingConfig();
  
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
  
  // Setup connection polling timer
  connect(connection_poll_timer_, &QTimer::timeout,
          this, &SSHTabWidget::PollSSHConnection);
  
  UpdateStatusLabel();
}

SSHTabWidget::~SSHTabWidget() {}

QString SSHTabWidget::GetCurrentHost() const {
  return current_host_;
}

QString SSHTabWidget::GetSetupBashFile() const {
  if (!setup_bash_edit_) {
    return QString();
  }
  return setup_bash_edit_->text();
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
  connection_group_ = new QGroupBox("SSH Configuration", this);
  QVBoxLayout* main_connection_layout = new QVBoxLayout(connection_group_);
  
  // First row: SSH address and controls
  QHBoxLayout* address_layout = new QHBoxLayout();
  
  // SSH address label and input
  address_label_ = new QLabel("SSH Address:", this);
  ssh_address_edit_ = new QLineEdit(this);
  ssh_address_edit_->setPlaceholderText("user@host or user@host -p port");
  // Default will be set from config file
  
  // Connect and disconnect buttons
  connect_button_ = new QPushButton("Connect", this);
  disconnect_button_ = new QPushButton("Disconnect", this);
  disconnect_button_->setEnabled(false);
  
  // Terminal button
  terminal_button_ = new QPushButton("Open Terminal", this);
  terminal_button_->setEnabled(false); // Disabled until connected
  
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
  
  // Add widgets to address layout
  address_layout->addWidget(address_label_);
  address_layout->addWidget(ssh_address_edit_, 1);
  address_layout->addWidget(connect_button_);
  address_layout->addWidget(disconnect_button_);
  address_layout->addWidget(terminal_button_);
  address_layout->addWidget(status_label_);
  
  // Second row: Setup.bash configuration
  QHBoxLayout* setup_layout = new QHBoxLayout();
  
  QLabel* setup_label = new QLabel("Setup.bash:", this);
  setup_bash_edit_ = new QLineEdit(this);
  setup_bash_edit_->setPlaceholderText("Enter remote setup.bash file path...");
  setup_bash_edit_->setMinimumWidth(400);
  
  setup_bash_button_ = new QPushButton("Browse", this);
  setup_bash_button_->setMinimumWidth(100);
  setup_bash_button_->setEnabled(false); // Disabled until connected
  
  setup_layout->addWidget(setup_label);
  setup_layout->addWidget(setup_bash_edit_, 1);
  setup_layout->addWidget(setup_bash_button_);
  
  // Add both layouts to main layout
  main_connection_layout->addLayout(address_layout);
  main_connection_layout->addLayout(setup_layout);
  
  // Load setup.bash default value
  LoadSetupBashFromConfig();
  
  // Connect signals
  connect(connect_button_, &QPushButton::clicked,
          this, &SSHTabWidget::OnConnectButtonClicked);
  connect(disconnect_button_, &QPushButton::clicked,
          this, &SSHTabWidget::OnDisconnectButtonClicked);
  connect(terminal_button_, &QPushButton::clicked,
          this, &SSHTabWidget::OnTerminalButtonClicked);
  connect(ssh_address_edit_, &QLineEdit::textChanged,
          this, &SSHTabWidget::OnSSHAddressChanged);
  connect(setup_bash_button_, &QPushButton::clicked,
          this, &SSHTabWidget::OnSetupBashButtonClicked);
  
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
    
    // Create stop button
    command_row.stop_button = new QPushButton("Ctrl+C");
    command_row.stop_button->setMaximumWidth(80);
    command_row.stop_button->setEnabled(false); // Disabled until process starts
    command_row.stop_button->setStyleSheet(
      "QPushButton {"
      "  background-color: #d63031;"
      "  color: white;"
      "  border: none;"
      "  padding: 4px 8px;"
      "  border-radius: 3px;"
      "  font-weight: bold;"
      "}"
      "QPushButton:hover {"
      "  background-color: #e17055;"
      "}"
      "QPushButton:disabled {"
      "  background-color: #666666;"
      "  color: #999999;"
      "}"
    );
    
    // Initialize process ID as empty
    command_row.process_id = "";
    
    // Add to grid layout
    grid_layout->addWidget(command_row.button, row, 0);
    grid_layout->addWidget(command_row.dropdown1, row, 1);
    grid_layout->addWidget(command_row.dropdown2, row, 2);
    grid_layout->addWidget(command_row.stop_button, row, 3);
    
    // Connect button signals
    connect(command_row.button, &QPushButton::clicked, [this, row]() {
      OnCommandRowButtonClicked(row);
    });
    
    connect(command_row.stop_button, &QPushButton::clicked, [this, row]() {
      OnStopButtonClicked(row);
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
    
    // Store connection details including password for terminal spawning
    current_password_ = password;
    
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
  
  // Check if setup.bash file is configured and wrap command if needed
  QString setup_bash_file = setup_bash_edit_->text().trimmed();
  QString final_command = full_command;
  
  if (!setup_bash_file.isEmpty()) {
    // Wrap command to source setup.bash first
    final_command = QString("bash -c \"source %1 && %2\"")
                    .arg(setup_bash_file)
                    .arg(full_command);
    
    output_display_->append(QString("=== Executing SSH Command with Setup.bash ==="));
    output_display_->append(QString("Setup.bash: %1").arg(setup_bash_file));
    output_display_->append(QString("Original Command: %1").arg(full_command));
    output_display_->append(QString("Final Command: %1").arg(final_command));
  } else {
    output_display_->append(QString("=== Executing SSH Command ==="));
    output_display_->append(QString("Command: %1").arg(full_command));
  }
  
  output_display_->append(QString("Row: %1").arg(row + 1));
  output_display_->append(QString("SSH Target: %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
  output_display_->append("=============================");
  output_display_->append(QString("$ %1").arg(final_command));
  
  // Generate a process ID for tracking this command
  QString process_id = QString("ssh_cmd_%1_%2").arg(QDateTime::currentMSecsSinceEpoch()).arg(row + 1);
  SetRowProcessId(row, process_id);
  
  ssh_manager_->ExecuteCommand(final_command);
}

void SSHTabWidget::OnSSHConnectionStatusChanged(bool connected) {
  is_connected_ = connected;
  
  // Connect button should be enabled when disconnected (regardless of polling)
  connect_button_->setEnabled(!connected);
  disconnect_button_->setEnabled(connected);
  
  // Enable/disable command buttons and clear process IDs when disconnecting
  for (int i = 0; i < command_rows_.size(); ++i) {
    CommandRow& row = command_rows_[i];
    row.button->setEnabled(connected);
    
    // When disconnecting, clear all process IDs and disable stop buttons
    if (!connected) {
      row.process_id = "";
      row.stop_button->setEnabled(false);
    }
  }
  
  // Enable/disable terminal button
  terminal_button_->setEnabled(connected);
  
  // Enable/disable setup.bash button
  setup_bash_button_->setEnabled(connected);
  
  UpdateStatusLabel();
  emit ConnectionStatusChanged(connected);
  
  if (connected) {
    output_display_->append("=== SSH Connection Established ===");
    output_display_->append(QString("Successfully connected to %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
    output_display_->append(QString("SSH session active - ready to execute commands"));
    output_display_->append("================================");
    
    // Start connection polling when connected
    StartConnectionPolling();
  } else {
    output_display_->append("=== SSH Connection Closed ===");
    output_display_->append("Disconnected from SSH server");
    output_display_->append("Command execution disabled");
    output_display_->append("=============================");
    
    // Stop connection polling when disconnected
    StopConnectionPolling();
    
    // Clear stored password for security
    current_password_.clear();
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
  
  // Clear all process IDs and disable stop buttons when a command finishes
  // Note: Since SSH commands run sequentially, we clear all when any finishes
  for (int i = 0; i < command_rows_.size(); ++i) {
    ClearRowProcessId(i);
  }
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

void SSHTabWidget::OnTerminalButtonClicked() {
  if (!is_connected_) {
    output_display_->append("Error: Not connected to SSH server");
    return;
  }
  
  output_display_->append("=== Opening SSH Terminal ===");
  output_display_->append(QString("Target: %1@%2:%3").arg(current_user_, current_host_).arg(current_port_));
  
  SpawnSSHTerminal();
}

QString SSHTabWidget::FindAvailableTerminal() {
  // List of terminal applications to try, in order of preference
  QStringList terminals = {
    "gnome-terminal",
    "konsole", 
    "xfce4-terminal",
    "mate-terminal",
    "lxterminal",
    "rxvt-unicode",
    "urxvt",
    "rxvt",
    "xterm"
  };
  
  for (const QString& terminal : terminals) {
    QProcess process;
    process.start("which", QStringList() << terminal);
    process.waitForFinished(1000);
    
    if (process.exitCode() == 0) {
      return terminal;
    }
  }
  
  return QString(); // No terminal found
}

void SSHTabWidget::SpawnSSHTerminal() {
  QString terminal = FindAvailableTerminal();
  
  if (terminal.isEmpty()) {
    output_display_->append("Error: No compatible terminal application found");
    output_display_->append("Supported terminals: gnome-terminal, konsole, xfce4-terminal, mate-terminal, lxterminal, xterm");
    return;
  }
  
  // Build SSH command
  QString ssh_command;
  if (!current_password_.isEmpty()) {
    ssh_command = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p %2 %3@%4")
                  .arg(current_password_, QString::number(current_port_), current_user_, current_host_);
  } else {
    ssh_command = QString("ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p %1 %2@%3")
                  .arg(QString::number(current_port_), current_user_, current_host_);
  }
  
  // Build terminal command based on terminal type
  QStringList terminal_args;
  
  if (terminal == "gnome-terminal") {
    terminal_args << "--" << "bash" << "-c" << ssh_command;
  } else if (terminal == "konsole") {
    terminal_args << "-e" << "bash" << "-c" << ssh_command;
  } else if (terminal == "xfce4-terminal") {
    terminal_args << "-e" << "bash" << "-c" << ssh_command;
  } else if (terminal == "mate-terminal") {
    terminal_args << "-e" << "bash" << "-c" << ssh_command;
  } else if (terminal == "lxterminal") {
    terminal_args << "-e" << "bash" << "-c" << ssh_command;
  } else {
    // For xterm and other basic terminals
    terminal_args << "-e" << "bash" << "-c" << ssh_command;
  }
  
  // Start the terminal
  QProcess* terminal_process = new QProcess(this);
  terminal_process->start(terminal, terminal_args);
  
  if (terminal_process->waitForStarted(2000)) {
    output_display_->append(QString("SSH terminal opened using: %1").arg(terminal));
    output_display_->append("=============================");
  } else {
    output_display_->append(QString("Failed to start terminal: %1").arg(terminal));
    output_display_->append("=============================");
    terminal_process->deleteLater();
  }
}

void SSHTabWidget::LoadPollingConfig() {
  try {
    // Try to load config from file
    toml::table config;
    try {
      config = toml::parse_file("../settings.toml");
    } catch (const toml::parse_error& err) {
      try {
        config = toml::parse_file("settings.toml");
      } catch (const toml::parse_error& err2) {
        qWarning() << "Failed to load settings.toml for polling config:" << err2.what();
        return;
      }
    }
    
    if (auto ssh_section = config["ssh"].as_table()) {
      // Load polling configuration
      if (auto poll_enabled = ssh_section->get("poll_connection")->as_boolean()) {
        poll_enabled_ = poll_enabled->get();
      }
      if (auto poll_interval = ssh_section->get("poll_interval")->as_integer()) {
        poll_interval_seconds_ = static_cast<int>(poll_interval->get());
      }
      if (auto poll_timeout = ssh_section->get("poll_timeout")->as_integer()) {
        poll_timeout_seconds_ = static_cast<int>(poll_timeout->get());
      }
      if (auto max_failures = ssh_section->get("max_poll_failures")->as_integer()) {
        max_poll_failures_ = static_cast<int>(max_failures->get());
      }
      
      qDebug() << "SSH polling config loaded - enabled:" << poll_enabled_ 
               << "interval:" << poll_interval_seconds_ << "s"
               << "timeout:" << poll_timeout_seconds_ << "s"
               << "max_failures:" << max_poll_failures_;
      
      if (!poll_enabled_) {
        qDebug() << "SSH polling is disabled (recommended for command-based SSH connections)";
      }
    }
  } catch (const std::exception& e) {
    qWarning() << "Error loading SSH polling config:" << e.what();
    // Use default values if config fails to load
    poll_enabled_ = false;  // Disabled by default - doesn't work well with command-based SSH
    poll_interval_seconds_ = 5;
    poll_timeout_seconds_ = 3;
    max_poll_failures_ = 3;
  }
}

void SSHTabWidget::StartConnectionPolling() {
  if (poll_enabled_ && !connection_poll_timer_->isActive()) {
    poll_failure_count_ = 0; // Reset failure count when starting polling
    connection_poll_timer_->start(poll_interval_seconds_ * 1000); // Convert to milliseconds
    qDebug() << "Started SSH connection polling with" << poll_interval_seconds_ << "second intervals";
  }
}

void SSHTabWidget::StopConnectionPolling() {
  if (connection_poll_timer_->isActive()) {
    connection_poll_timer_->stop();
    qDebug() << "Stopped SSH connection polling";
  }
}

void SSHTabWidget::PollSSHConnection() {
  if (!is_connected_ || current_host_.isEmpty() || current_user_.isEmpty()) {
    qDebug() << "Skipping SSH poll - not connected or missing connection info";
    return;
  }
  
  // Skip polling if we've reached max failures to avoid spam
  if (poll_failure_count_ >= max_poll_failures_) {
    qDebug() << "Skipping SSH poll - too many failures";
    return;
  }
  
  qDebug() << "Polling SSH connection to" << current_user_ << "@" << current_host_ << ":" << current_port_;
  
  // Create a lightweight SSH test to check connection status
  QProcess* poll_process = new QProcess(this);
  
  // Build polling command - simple echo test with timeout
  QString program;
  QStringList arguments;
  
  if (!current_password_.isEmpty()) {
    program = "sshpass";
    arguments << "-p" << current_password_
             << "ssh"
             << "-o" << "StrictHostKeyChecking=no"
             << "-o" << "UserKnownHostsFile=/dev/null"
             << "-o" << "LogLevel=QUIET"
             << "-o" << "BatchMode=yes"
             << "-o" << QString("ConnectTimeout=%1").arg(poll_timeout_seconds_)
             << "-p" << QString::number(current_port_)
             << QString("%1@%2").arg(current_user_, current_host_)
             << "echo SSH_POLL_OK";
  } else {
    program = "ssh";
    arguments << "-o" << "StrictHostKeyChecking=no"
             << "-o" << "UserKnownHostsFile=/dev/null"
             << "-o" << "LogLevel=QUIET"
             << "-o" << "BatchMode=yes"
             << "-o" << QString("ConnectTimeout=%1").arg(poll_timeout_seconds_)
             << "-p" << QString::number(current_port_)
             << QString("%1@%2").arg(current_user_, current_host_)
             << "echo SSH_POLL_OK";
  }
  
  // Set process timeout and connect completion handler
  QTimer::singleShot(poll_timeout_seconds_ * 1000 + 1000, poll_process, [poll_process]() {
    if (poll_process->state() == QProcess::Running) {
      poll_process->kill();
    }
  });
  
  connect(poll_process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          [this, poll_process](int exit_code, QProcess::ExitStatus) {
    bool connection_ok = (exit_code == 0);
    
    if (connection_ok && is_connected_) {
      // Connection successful - reset failure count
      poll_failure_count_ = 0;
    } else if (!connection_ok && is_connected_) {
      // Connection failed - increment failure count
      poll_failure_count_++;
      qDebug() << "SSH poll failed (" << poll_failure_count_ << "/" << max_poll_failures_ << ")";
      
      // Only disconnect after multiple consecutive failures
      if (poll_failure_count_ >= max_poll_failures_) {
        qDebug() << "SSH polling detected persistent connection loss after" << max_poll_failures_ << "failures";
        OnSSHConnectionStatusChanged(false);
        poll_failure_count_ = 0; // Reset for next connection
      }
    } else if (connection_ok && !is_connected_) {
      // Connection restored - update status (this case is less likely)
      qDebug() << "SSH polling detected connection restoration";
      OnSSHConnectionStatusChanged(true);
      poll_failure_count_ = 0;
    }
    
    poll_process->deleteLater();
  });
  
  qDebug() << "Executing SSH poll command:" << program << arguments.join(" ");
  poll_process->start(program, arguments);
}

void SSHTabWidget::LoadSetupBashFromConfig() {
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
    
    // Load SSH setup.bash default
    if (auto ssh_section = config["ssh"].as_table()) {
      if (auto setup_bash = ssh_section->get("setup_bash_file")->as_string()) {
        QString setup_file = QString::fromStdString(setup_bash->get()).remove('"');
        setup_bash_edit_->setText(setup_file);
      }
    }
  } catch (const std::exception& e) {
    qWarning() << "Error loading SSH setup.bash from config:" << e.what();
  }
}

void SSHTabWidget::OnSetupBashButtonClicked() {
  // For SSH, this would typically be a remote file browser
  // For now, we'll allow manual editing since browsing remote files is complex
  // In a future version, this could open a remote file browser when connected
  
  if (!is_connected_) {
    // If not connected, show a message that this is for remote files
    output_display_->append("Note: Setup.bash path should be a path on the remote server (e.g., ~/ros2_ws/install/setup.bash)");
    return;
  }
  
  // TODO: Implement remote file browsing when connected
  output_display_->append("Remote file browsing not yet implemented. Please enter the path manually.");
}

void SSHTabWidget::SetRowProcessId(int row, const QString& process_id) {
  if (row >= 0 && row < command_rows_.size()) {
    command_rows_[row].process_id = process_id;
    command_rows_[row].stop_button->setEnabled(!process_id.isEmpty());
    qDebug() << "SSH Row" << (row + 1) << "process ID set to:" << process_id;
  }
}

void SSHTabWidget::ClearRowProcessId(int row) {
  if (row >= 0 && row < command_rows_.size()) {
    command_rows_[row].process_id = "";
    command_rows_[row].stop_button->setEnabled(false);
    qDebug() << "SSH Row" << (row + 1) << "process ID cleared";
  }
}

void SSHTabWidget::OnStopButtonClicked(int row) {
  if (row >= 0 && row < command_rows_.size()) {
    const CommandRow& command_row = command_rows_[row];
    
    if (!command_row.process_id.isEmpty()) {
      qDebug() << "Stopping SSH process for Row" << (row + 1) << "- Process ID:" << command_row.process_id;
      
      // For SSH commands, we need to send a kill signal to the remote process
      // This is typically done by sending Ctrl+C (SIGINT) through the SSH connection
      if (ssh_manager_) {
        output_display_->append(QString("=== Terminating SSH Command ==="));
        output_display_->append(QString("Sending Ctrl+C to process: %1").arg(command_row.process_id));
        output_display_->append(QString("Row: %1").arg(row + 1));
        output_display_->append("==============================");
        
        // Send Ctrl+C signal to the SSH process
        ssh_manager_->KillCommand(command_row.process_id);
        
        // Clear the process ID and disable the stop button
        command_rows_[row].process_id = "";
        command_rows_[row].stop_button->setEnabled(false);
      }
    } else {
      qDebug() << "No running SSH process found for Row" << (row + 1);
      output_display_->append(QString("No running process found for Row %1").arg(row + 1));
    }
  }
}