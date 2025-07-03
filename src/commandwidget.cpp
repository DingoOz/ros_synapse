// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "commandwidget.h"
#include "configmanager.h"
#include <QGridLayout>
#include <QSignalMapper>

CommandWidget::CommandWidget(QWidget* parent) 
    : QWidget(parent),
      config_manager_(new ConfigManager(this)) {
  setMinimumSize(800, 600);
  SetupUI();
}

void CommandWidget::OnPackageSelected(const QString& package) {
  Q_UNUSED(package)
}

void CommandWidget::OnLaunchFileSelected(const QString& launch_file) {
  Q_UNUSED(launch_file)
}

void CommandWidget::OnExecutableSelected(const QString& executable) {
  Q_UNUSED(executable)
}

void CommandWidget::OnParameterChanged() {}

void CommandWidget::OnBuildCommand() {}

void CommandWidget::OnExecuteCommand() {}

void CommandWidget::OnSaveTemplate() {}

void CommandWidget::OnLoadTemplate() {}

void CommandWidget::OnClearParameters() {}

void CommandWidget::SetupUI() {
  main_layout_ = new QVBoxLayout(this);
  
  // Setup control rows group
  control_rows_group_ = new QGroupBox("Command Controls", this);
  main_layout_->addWidget(control_rows_group_);
  
  SetupControlRows();
  
  // Add stretch to push everything to the top
  main_layout_->addStretch();
}

void CommandWidget::SetupQuickLaunchButtons() {}

void CommandWidget::SetupCommandBuilder() {}

void CommandWidget::SetupParameterForm() {}

void CommandWidget::SetupControlButtons() {}

void CommandWidget::UpdateParameterForm() {}

void CommandWidget::BuildCommand() {}

void CommandWidget::ExecuteQuickCommand(const QString& command) {
  Q_UNUSED(command)
}

void CommandWidget::LoadPackageInfo(const QString& package) {
  Q_UNUSED(package)
}

void CommandWidget::LoadCommandTemplates() {}

void CommandWidget::SaveCommandTemplate(const QString& name, 
                                         const QJsonObject& template_data) {
  Q_UNUSED(name)
  Q_UNUSED(template_data)
}

QJsonObject CommandWidget::GetParameterValues() {
  return QJsonObject();
}

void CommandWidget::SetParameterValues(const QJsonObject& parameters) {
  Q_UNUSED(parameters)
}

void CommandWidget::SetupControlRows() {
  QGridLayout* grid_layout = new QGridLayout(control_rows_group_);
  
  // Create 3 rows of controls
  for (int row = 0; row < 3; ++row) {
    ControlRow control_row;
    
    // Create button
    control_row.button = new QPushButton(QString("Execute Row %1").arg(row + 1));
    
    // Create first dropdown
    control_row.dropdown1 = new QComboBox();
    control_row.dropdown1->setEditable(true);
    
    // Create second dropdown  
    control_row.dropdown2 = new QComboBox();
    control_row.dropdown2->setEditable(true);
    
    // Load options from TOML
    QString dropdown1_key = QString("row%1_dropdown1_options").arg(row + 1);
    QString dropdown1_default_key = QString("row%1_dropdown1_default").arg(row + 1);
    QString dropdown2_key = QString("row%1_dropdown2_options").arg(row + 1);
    QString dropdown2_default_key = QString("row%1_dropdown2_default").arg(row + 1);
    
    QStringList dropdown1_options = config_manager_->GetControlDropdownOptions(dropdown1_key);
    QString dropdown1_default = config_manager_->GetControlDefaultValue(dropdown1_default_key);
    QStringList dropdown2_options = config_manager_->GetControlDropdownOptions(dropdown2_key);
    QString dropdown2_default = config_manager_->GetControlDefaultValue(dropdown2_default_key);
    
    // Populate dropdowns
    control_row.dropdown1->addItems(dropdown1_options);
    control_row.dropdown1->setCurrentText(dropdown1_default);
    
    control_row.dropdown2->addItems(dropdown2_options);
    control_row.dropdown2->setCurrentText(dropdown2_default);
    
    // Add to grid layout
    grid_layout->addWidget(control_row.button, row, 0);
    grid_layout->addWidget(control_row.dropdown1, row, 1);
    grid_layout->addWidget(control_row.dropdown2, row, 2);
    
    // Connect button signal
    connect(control_row.button, &QPushButton::clicked, [this, row]() {
      OnControlRowButtonClicked(row);
    });
    
    control_rows_.append(control_row);
  }
}

void CommandWidget::OnControlRowButtonClicked(int row) {
  if (row >= 0 && row < control_rows_.size()) {
    const ControlRow& control_row = control_rows_[row];
    QString dropdown1_value = control_row.dropdown1->currentText();
    QString dropdown2_value = control_row.dropdown2->currentText();
    
    QString command = QString("%1 %2").arg(dropdown1_value, dropdown2_value);
    
    // Emit signal or execute command
    emit CommandReady(command);
    qDebug() << "Row" << (row + 1) << "executed:" << command;
  }
}