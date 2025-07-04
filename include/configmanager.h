// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// Configuration manager for persistent settings, connection profiles,
// ROS2 environment, and UI preferences with JSON import/export support.

#ifndef ROS_SYNAPSE_INCLUDE_CONFIGMANAGER_H_
#define ROS_SYNAPSE_INCLUDE_CONFIGMANAGER_H_

#include <QObject>
#include <QSettings>
#include <QJsonObject>
#include <QJsonDocument>
#include <QStringList>
#include <QMap>
#include <toml++/toml.h>

class ConfigManager : public QObject {
  Q_OBJECT

 public:
  struct ConnectionProfile {
    QString name;
    QString host;
    QString username;
    QString password;
    QString key_file;
    int port;
    bool use_key_auth;
    QString description;
  };

  struct ROS2Config {
    QString workspace;
    int domain_id;
    QString turtlebot3_model;
    QString distribution_path;
    QStringList additional_paths;
    QMap<QString, QString> environment_variables;
  };

  struct UIConfig {
    QString theme;
    bool dark_mode;
    int font_size;
    QString font_family;
    bool show_status_bar;
    bool show_tool_bar;
    QMap<QString, QVariant> window_geometry;
    QMap<QString, QVariant> splitter_states;
  };

  explicit ConfigManager(QObject* parent = nullptr);
  ~ConfigManager();

  void LoadSettings();
  void SaveSettings();
  void ResetToDefaults();
  
  QStringList GetProfileNames() const;
  ConnectionProfile GetConnectionProfile(const QString& name) const;
  void SaveConnectionProfile(const QString& name, 
                             const ConnectionProfile& profile);
  void DeleteConnectionProfile(const QString& name);
  QString GetDefaultProfile() const;
  void SetDefaultProfile(const QString& name);
  
  ROS2Config GetROS2Config() const;
  void SetROS2Config(const ROS2Config& config);
  
  UIConfig GetUIConfig() const;
  void SetUIConfig(const UIConfig& config);
  
  QJsonObject GetCommandTemplates() const;
  void SetCommandTemplates(const QJsonObject& templates);
  void AddCommandTemplate(const QString& name, 
                          const QJsonObject& template_data);
  void RemoveCommandTemplate(const QString& name);
  
  QString GetConfigFilePath() const;
  bool ExportConfig(const QString& file_path) const;
  bool ImportConfig(const QString& file_path);
  
  // TOML settings methods
  QStringList GetControlDropdownOptions(const QString& control_name) const;
  QString GetControlDefaultValue(const QString& control_name) const;
  bool GetDarkModeEnabled() const;
  QString GetROS2WorkingDirectory() const;
  QString GetSetupBashFile() const;
  QString GetSSHSetupBashFile() const;

 signals:
  void ConfigChanged();
  void ProfileAdded(const QString& name);
  void ProfileRemoved(const QString& name);
  void ProfileChanged(const QString& name);

 private slots:
  void OnSettingsChanged();

 private:
  void InitializeDefaults();
  void LoadConnectionProfiles();
  void SaveConnectionProfiles();
  void LoadROS2Config();
  void SaveROS2Config();
  void LoadUIConfig();
  void SaveUIConfig();
  void LoadCommandTemplates();
  void SaveCommandTemplates();
  void ValidateConfig();
  QJsonObject ProfileToJson(const ConnectionProfile& profile) const;
  ConnectionProfile ProfileFromJson(const QJsonObject& json) const;
  QJsonObject ROS2ConfigToJson(const ROS2Config& config) const;
  ROS2Config ROS2ConfigFromJson(const QJsonObject& json) const;
  QJsonObject UIConfigToJson(const UIConfig& config) const;
  UIConfig UIConfigFromJson(const QJsonObject& json) const;
  
  QSettings* settings_;
  QMap<QString, ConnectionProfile> connection_profiles_;
  ROS2Config ros2_config_;
  UIConfig ui_config_;
  QJsonObject command_templates_;
  
  QString default_profile_;
  QString config_file_path_;
  
  // TOML configuration
  mutable toml::table toml_config_;
  
  static const QString kSettingsGroupProfiles;
  static const QString kSettingsGroupROS2;
  static const QString kSettingsGroupUI;
  static const QString kSettingsGroupTemplates;
  static const QString kSettingsKeyDefaultProfile;
  
  static const ROS2Config kDefaultROS2Config;
  static const UIConfig kDefaultUIConfig;
};

#endif  // ROS_SYNAPSE_INCLUDE_CONFIGMANAGER_H_