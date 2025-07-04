// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "sshpassworddialog.h"
#include <QApplication>
#include <QKeyEvent>

SSHPasswordDialog::SSHPasswordDialog(const QString& username, const QString& host, 
                                   int port, QWidget* parent)
    : QDialog(parent),
      username_(username),
      host_(host),
      port_(port) {
  
  setWindowTitle("SSH Password Required");
  setModal(true);
  setFixedSize(500, 250);
  
  // Set window flags for better dark theme integration
  setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
  
  SetupUI();
  SetupConnections();
  
  // Focus on password field
  password_edit_->setFocus();
}

SSHPasswordDialog::~SSHPasswordDialog() {}

QString SSHPasswordDialog::GetPassword() const {
  return password_edit_->text();
}

void SSHPasswordDialog::SetPassword(const QString& password) {
  password_edit_->setText(password);
}

void SSHPasswordDialog::SetupUI() {
  main_layout_ = new QVBoxLayout(this);
  
  // Set dialog background to match dark theme
  setStyleSheet("QDialog { background-color: #2b2b2b; color: #ffffff; }");
  
  // Info label
  info_label_ = new QLabel("Enter password for SSH connection:", this);
  info_label_->setWordWrap(true);
  info_label_->setStyleSheet("font-weight: bold; color: #ffffff; margin-bottom: 10px; background-color: transparent;");
  main_layout_->addWidget(info_label_);
  
  // Connection details form
  form_layout_ = new QFormLayout();
  form_layout_->setSpacing(12);
  
  // Style for form labels - dark theme
  QString label_style = "QLabel { color: #ffffff; font-weight: bold; background-color: transparent; }";
  
  username_label_ = new QLabel(username_, this);
  username_label_->setStyleSheet("font-family: monospace; background-color: #4a4a4a; color: #ffffff; padding: 4px 8px; border-radius: 3px;");
  QLabel* username_field_label = new QLabel("Username:", this);
  username_field_label->setStyleSheet(label_style);
  form_layout_->addRow(username_field_label, username_label_);
  
  host_label_ = new QLabel(host_, this);
  host_label_->setStyleSheet("font-family: monospace; background-color: #4a4a4a; color: #ffffff; padding: 4px 8px; border-radius: 3px;");
  QLabel* host_field_label = new QLabel("Host:", this);
  host_field_label->setStyleSheet(label_style);
  form_layout_->addRow(host_field_label, host_label_);
  
  port_label_ = new QLabel(QString::number(port_), this);
  port_label_->setStyleSheet("font-family: monospace; background-color: #4a4a4a; color: #ffffff; padding: 4px 8px; border-radius: 3px;");
  QLabel* port_field_label = new QLabel("Port:", this);
  port_field_label->setStyleSheet(label_style);
  form_layout_->addRow(port_field_label, port_label_);
  
  // Password input
  password_edit_ = new QLineEdit(this);
  password_edit_->setEchoMode(QLineEdit::Password);
  password_edit_->setPlaceholderText("Enter password...");
  password_edit_->setMinimumHeight(35);
  password_edit_->setStyleSheet(
    "QLineEdit {"
    "  padding: 10px;"
    "  border: 2px solid #555555;"
    "  border-radius: 4px;"
    "  font-size: 14px;"
    "  background-color: #3b3b3b;"
    "  color: #ffffff;"
    "}"
    "QLineEdit:focus {"
    "  border-color: #0078d4;"
    "}"
    "QLineEdit::placeholder {"
    "  color: #aaaaaa;"
    "}"
  );
  QLabel* password_field_label = new QLabel("Password:", this);
  password_field_label->setStyleSheet(label_style);
  form_layout_->addRow(password_field_label, password_edit_);
  
  main_layout_->addLayout(form_layout_);
  
  // Buttons
  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  button_box_->setStyleSheet("QDialogButtonBox { background-color: transparent; }");
  connect_button_ = button_box_->button(QDialogButtonBox::Ok);
  cancel_button_ = button_box_->button(QDialogButtonBox::Cancel);
  
  connect_button_->setText("Connect");
  connect_button_->setEnabled(false);
  connect_button_->setStyleSheet(
    "QPushButton {"
    "  background-color: #0078d4;"
    "  color: #ffffff;"
    "  border: none;"
    "  padding: 8px 16px;"
    "  border-radius: 4px;"
    "  font-weight: bold;"
    "}"
    "QPushButton:hover {"
    "  background-color: #106ebe;"
    "}"
    "QPushButton:disabled {"
    "  background-color: #555555;"
    "  color: #888888;"
    "}"
  );
  
  cancel_button_->setText("Cancel");
  cancel_button_->setStyleSheet(
    "QPushButton {"
    "  background-color: #4a4a4a;"
    "  color: #ffffff;"
    "  border: 1px solid #666666;"
    "  padding: 8px 16px;"
    "  border-radius: 4px;"
    "}"
    "QPushButton:hover {"
    "  background-color: #5a5a5a;"
    "}"
  );
  
  main_layout_->addWidget(button_box_);
  
  main_layout_->addStretch();
}

void SSHPasswordDialog::SetupConnections() {
  connect(password_edit_, &QLineEdit::textChanged, 
          this, &SSHPasswordDialog::OnPasswordChanged);
  connect(connect_button_, &QPushButton::clicked, 
          this, &SSHPasswordDialog::OnConnectClicked);
  connect(cancel_button_, &QPushButton::clicked, 
          this, &SSHPasswordDialog::OnCancelClicked);
  
  // Allow Enter key to connect
  connect(password_edit_, &QLineEdit::returnPressed, [this]() {
    if (connect_button_->isEnabled()) {
      OnConnectClicked();
    }
  });
}

void SSHPasswordDialog::OnPasswordChanged() {
  bool has_password = !password_edit_->text().isEmpty();
  connect_button_->setEnabled(has_password);
}

void SSHPasswordDialog::OnConnectClicked() {
  accept();
}

void SSHPasswordDialog::OnCancelClicked() {
  reject();
}