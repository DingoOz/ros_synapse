// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// Command builder widget for dynamic ROS2 command generation with parameter
// forms, template management, and quick launch functionality.

#ifndef ROS_SYNAPSE_INCLUDE_COMMANDWIDGET_H_
#define ROS_SYNAPSE_INCLUDE_COMMANDWIDGET_H_

#include <QWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

class ConfigManager;

class CommandWidget : public QWidget {
  Q_OBJECT

 public:
  explicit CommandWidget(QWidget* parent = nullptr);
  
  QString GetWorkingDirectory() const;
  void SetRowProcessId(int row, const QString& process_id);
  void ClearRowProcessId(int row);

 signals:
  void CommandReady(const QString& command, int row);
  void CommandExecuted(const QString& command, const QString& output);
  void WorkingDirectoryChanged(const QString& directory);
  void StopProcessRequested(const QString& process_id);

 private slots:
  void OnPackageSelected(const QString& package);
  void OnLaunchFileSelected(const QString& launch_file);
  void OnExecutableSelected(const QString& executable);
  void OnParameterChanged();
  void OnBuildCommand();
  void OnExecuteCommand();
  void OnSaveTemplate();
  void OnLoadTemplate();
  void OnClearParameters();
  void OnWorkingDirectoryButtonClicked();
  void OnStopButtonClicked(int row);

 private:
  void SetupUI();
  void SetupWorkingDirectory();
  void SetupQuickLaunchButtons();
  void SetupCommandBuilder();
  void SetupParameterForm();
  void SetupControlButtons();
  void SetupControlRows();
  void UpdateParameterForm();
  void BuildCommand();
  void ExecuteQuickCommand(const QString& command);
  void LoadPackageInfo(const QString& package);
  void LoadCommandTemplates();
  void SaveCommandTemplate(const QString& name, 
                           const QJsonObject& template_data);
  QJsonObject GetParameterValues();
  void SetParameterValues(const QJsonObject& parameters);
  void OnControlRowButtonClicked(int row);
  void LoadWorkingDirectoryFromConfig();

  QVBoxLayout* main_layout_;
  QGroupBox* quick_launch_group_;
  QGroupBox* command_builder_group_;
  QGroupBox* parameter_group_;
  QGroupBox* control_rows_group_;
  
  QComboBox* package_combo_;
  QComboBox* launch_file_combo_;
  QComboBox* executable_combo_;
  QComboBox* template_combo_;
  
  QScrollArea* parameter_scroll_area_;
  QWidget* parameter_widget_;
  QFormLayout* parameter_layout_;
  
  QTextEdit* command_preview_;
  QTextEdit* output_display_;
  
  QPushButton* build_button_;
  QPushButton* execute_button_;
  QPushButton* save_template_button_;
  QPushButton* load_template_button_;
  QPushButton* clear_button_;
  
  // Working directory controls
  QGroupBox* working_directory_group_;
  QLineEdit* working_directory_edit_;
  QPushButton* working_directory_button_;
  
  QList<QPushButton*> quick_launch_buttons_;
  QList<QWidget*> parameter_widgets_;
  
  // Control rows widgets
  struct ControlRow {
    QPushButton* button;
    QComboBox* dropdown1;
    QComboBox* dropdown2;
    QPushButton* stop_button;
    QString process_id;  // Track the process ID for this row
  };
  QList<ControlRow> control_rows_;
  
  QString current_command_;
  QJsonObject package_info_;
  QJsonObject command_templates_;
  
  ConfigManager* config_manager_;
};

#endif  // ROS_SYNAPSE_INCLUDE_COMMANDWIDGET_H_