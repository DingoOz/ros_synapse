// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// SSH tab widget for remote command execution with connection management
// and 2-row command interface similar to Command Builder.

#ifndef ROS_SYNAPSE_INCLUDE_SSHTABWIDGET_H_
#define ROS_SYNAPSE_INCLUDE_SSHTABWIDGET_H_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QTextEdit>
#include <QTimer>

class ConfigManager;
class SSHManager;

class SSHTabWidget : public QWidget {
  Q_OBJECT

 public:
  explicit SSHTabWidget(QWidget* parent = nullptr);
  ~SSHTabWidget();
  
  QString GetCurrentHost() const;
  QString GetSetupBashFile() const;
  void SetRowProcessId(int row, const QString& process_id);
  void ClearRowProcessId(int row);

 signals:
  void CommandExecuted(const QString& command, const QString& output);
  void ConnectionStatusChanged(bool connected);

 private slots:
  void OnConnectButtonClicked();
  void OnDisconnectButtonClicked();
  void OnTerminalButtonClicked();
  void OnSSHAddressChanged();
  void OnSetupBashButtonClicked();
  void OnCommandRowButtonClicked(int row);
  void OnStopButtonClicked(int row);
  void OnSSHConnectionStatusChanged(bool connected);
  void OnSSHCommandOutput(const QString& output);
  void OnSSHCommandFinished(const QString& command, int exit_code);
  void OnSSHErrorOccurred(const QString& error);
  void UpdateConnectionStatus();

 private:
  void SetupUI();
  void SetupConnectionArea();
  void SetupCommandRows();
  void LoadDefaultsFromConfig();
  void LoadSetupBashFromConfig();
  void UpdateStatusLabel();
  void ParseSSHAddress(const QString& address, QString& user, QString& host, 
                       int& port);
  bool ValidateSSHAddress(const QString& address);
  QString FindAvailableTerminal();
  void SpawnSSHTerminal();
  void LoadPollingConfig();
  void StartConnectionPolling();
  void StopConnectionPolling();
  void PollSSHConnection();

  // Connection area widgets
  QGroupBox* connection_group_;
  QLabel* address_label_;
  QLineEdit* ssh_address_edit_;
  QPushButton* connect_button_;
  QPushButton* disconnect_button_;
  QPushButton* terminal_button_;
  QLabel* status_label_;
  
  // Setup bash controls
  QLineEdit* setup_bash_edit_;
  QPushButton* setup_bash_button_;

  // Command rows widgets
  QGroupBox* command_rows_group_;
  struct CommandRow {
    QPushButton* button;
    QComboBox* dropdown1;
    QComboBox* dropdown2;
    QPushButton* stop_button;
    QString process_id;  // Track the process ID for this row
  };
  QList<CommandRow> command_rows_;

  // Output display
  QGroupBox* output_group_;
  QTextEdit* output_display_;

  // Backend components
  ConfigManager* config_manager_;
  SSHManager* ssh_manager_;
  QTimer* status_update_timer_;
  QTimer* connection_poll_timer_;

  // Connection state
  bool is_connected_;
  QString current_host_;
  QString current_user_;
  QString current_password_;
  int current_port_;
  
  // Polling configuration
  bool poll_enabled_;
  int poll_interval_seconds_;
  int poll_timeout_seconds_;
  int poll_failure_count_;
  int max_poll_failures_;
};

#endif  // ROS_SYNAPSE_INCLUDE_SSHTABWIDGET_H_