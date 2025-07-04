// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "commandwidget.h"
#include "configmanager.h"
#include <QGridLayout>
#include <QSignalMapper>
#include <QFileDialog>
#include <QDir>

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
  
  // Setup working directory controls
  SetupWorkingDirectory();
  main_layout_->addWidget(working_directory_group_);
  
  // Setup control rows group
  control_rows_group_ = new QGroupBox("Command Controls", this);
  main_layout_->addWidget(control_rows_group_);
  
  SetupControlRows();
  
  // Add stretch to push everything to the top
  main_layout_->addStretch();
}

void CommandWidget::SetupWorkingDirectory() {
  // Create working directory group
  working_directory_group_ = new QGroupBox("Working Directory", this);
  QHBoxLayout* wd_layout = new QHBoxLayout(working_directory_group_);
  
  // Create text box for working directory
  working_directory_edit_ = new QLineEdit(this);
  working_directory_edit_->setPlaceholderText("Enter working directory path...");
  working_directory_edit_->setMinimumWidth(400);
  
  // Load default value from config
  LoadWorkingDirectoryFromConfig();
  
  // Create browse button
  working_directory_button_ = new QPushButton("Browse", this);
  working_directory_button_->setMinimumWidth(100);
  
  // Add widgets to layout
  wd_layout->addWidget(new QLabel("Path:", this));
  wd_layout->addWidget(working_directory_edit_, 1); // Give text box more space
  wd_layout->addWidget(working_directory_button_);
  
  // Connect button signal
  connect(working_directory_button_, &QPushButton::clicked,
          this, &CommandWidget::OnWorkingDirectoryButtonClicked);
  
  // Connect text change signal
  connect(working_directory_edit_, &QLineEdit::textChanged,
          this, &CommandWidget::WorkingDirectoryChanged);
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
  
  // Create 4 rows of controls
  for (int row = 0; row < 4; ++row) {
    ControlRow control_row;
    
    // Create button
    control_row.button = new QPushButton(QString("Execute Row %1").arg(row + 1));
    
    // Create first dropdown
    control_row.dropdown1 = new QComboBox();
    control_row.dropdown1->setEditable(true);
    
    // Create second dropdown  
    control_row.dropdown2 = new QComboBox();
    control_row.dropdown2->setEditable(true);
    
    // Create stop button
    control_row.stop_button = new QPushButton("Ctrl+C");
    control_row.stop_button->setMaximumWidth(80);
    control_row.stop_button->setEnabled(false); // Disabled until process starts
    control_row.stop_button->setStyleSheet(
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
      "  background-color: #636e72;"
      "  color: #b2bec3;"
      "}"
    );
    
    // Initialize process tracking
    control_row.process_id = "";
    
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
    grid_layout->addWidget(control_row.stop_button, row, 3);
    
    // Connect button signals
    connect(control_row.button, &QPushButton::clicked, [this, row]() {
      OnControlRowButtonClicked(row);
    });
    
    connect(control_row.stop_button, &QPushButton::clicked, [this, row]() {
      OnStopButtonClicked(row);
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
    
    // Emit signal with row number
    emit CommandReady(command, row);
    qDebug() << "Row" << (row + 1) << "executed:" << command;
  }
}

void CommandWidget::OnWorkingDirectoryButtonClicked() {
  QString current_dir = working_directory_edit_->text();
  if (current_dir.isEmpty()) {
    current_dir = QDir::homePath();
  }
  
  QString selected_dir = QFileDialog::getExistingDirectory(
    this,
    "Select Working Directory",
    current_dir,
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
  );
  
  if (!selected_dir.isEmpty()) {
    working_directory_edit_->setText(selected_dir);
    emit WorkingDirectoryChanged(selected_dir);
  }
}

QString CommandWidget::GetWorkingDirectory() const {
  if (!working_directory_edit_) {
    return QString();
  }
  return working_directory_edit_->text();
}

void CommandWidget::LoadWorkingDirectoryFromConfig() {
  if (!config_manager_) {
    return;
  }
  
  // Try to load from config, fall back to default
  QString working_dir = config_manager_->GetROS2WorkingDirectory();
  if (working_dir.isEmpty()) {
    working_dir = "~/Programming/tb3_autonomy";
  }
  
  working_directory_edit_->setText(working_dir);
}

void CommandWidget::OnStopButtonClicked(int row) {
  if (row >= 0 && row < control_rows_.size()) {
    const ControlRow& control_row = control_rows_[row];
    
    if (!control_row.process_id.isEmpty()) {
      qDebug() << "Stopping process for Row" << (row + 1) << "- Process ID:" << control_row.process_id;
      emit StopProcessRequested(control_row.process_id);
      
      // Disable the stop button and clear process ID
      control_row.stop_button->setEnabled(false);
      control_rows_[row].process_id = "";
    } else {
      qDebug() << "No running process found for Row" << (row + 1);
    }
  }
}

void CommandWidget::SetRowProcessId(int row, const QString& process_id) {
  if (row >= 0 && row < control_rows_.size()) {
    control_rows_[row].process_id = process_id;
    control_rows_[row].stop_button->setEnabled(!process_id.isEmpty());
    qDebug() << "Row" << (row + 1) << "process ID set to:" << process_id;
  }
}

void CommandWidget::ClearRowProcessId(int row) {
  if (row >= 0 && row < control_rows_.size()) {
    control_rows_[row].process_id = "";
    control_rows_[row].stop_button->setEnabled(false);
    qDebug() << "Row" << (row + 1) << "process ID cleared";
  }
}