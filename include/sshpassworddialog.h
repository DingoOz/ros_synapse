// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// SSH password dialog for secure credential input during SSH connection
// establishment with masked password field and connection validation.

#ifndef ROS_SYNAPSE_INCLUDE_SSHPASSWORDDIALOG_H_
#define ROS_SYNAPSE_INCLUDE_SSHPASSWORDDIALOG_H_

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFormLayout>

class SSHPasswordDialog : public QDialog {
  Q_OBJECT

 public:
  explicit SSHPasswordDialog(const QString& username, const QString& host, 
                           int port, QWidget* parent = nullptr);
  ~SSHPasswordDialog();

  QString GetPassword() const;
  void SetPassword(const QString& password);

 private slots:
  void OnPasswordChanged();
  void OnConnectClicked();
  void OnCancelClicked();

 private:
  void SetupUI();
  void SetupConnections();
  
  QVBoxLayout* main_layout_;
  QFormLayout* form_layout_;
  QHBoxLayout* button_layout_;
  
  QLabel* info_label_;
  QLabel* username_label_;
  QLabel* host_label_;
  QLabel* port_label_;
  QLabel* password_label_;
  
  QLineEdit* password_edit_;
  QPushButton* connect_button_;
  QPushButton* cancel_button_;
  QDialogButtonBox* button_box_;
  
  QString username_;
  QString host_;
  int port_;
};

#endif  // ROS_SYNAPSE_INCLUDE_SSHPASSWORDDIALOG_H_