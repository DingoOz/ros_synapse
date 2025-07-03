// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "configmanager.h"
#include <QDir>
#include <QDebug>
#include <QFile>

const QString ConfigManager::kSettingsGroupProfiles = "Profiles";
const QString ConfigManager::kSettingsGroupROS2 = "ROS2";
const QString ConfigManager::kSettingsGroupUI = "UI";
const QString ConfigManager::kSettingsGroupTemplates = "Templates";
const QString ConfigManager::kSettingsKeyDefaultProfile = "DefaultProfile";

ConfigManager::ConfigManager(QObject* parent)
    : QObject(parent),
      settings_(new QSettings(this)) {
  // Load TOML configuration
  QString toml_path = "../settings.toml"; // Look in parent directory from build
  if (!QFile::exists(toml_path)) {
    toml_path = "settings.toml"; // Fallback to current directory
  }
  
  try {
    toml_config_ = toml::parse_file(toml_path.toStdString());
    qDebug() << "Successfully loaded settings.toml from" << toml_path;
  } catch (const toml::parse_error& err) {
    qWarning() << "Failed to parse settings.toml:" << err.what();
    qDebug() << "Working directory:" << QDir::currentPath();
    qDebug() << "Tried path:" << toml_path;
  }
}

ConfigManager::~ConfigManager() {}

void ConfigManager::LoadSettings() {}

void ConfigManager::SaveSettings() {}

void ConfigManager::ResetToDefaults() {}

QStringList ConfigManager::GetProfileNames() const {
  return connection_profiles_.keys();
}

ConfigManager::ConnectionProfile ConfigManager::GetConnectionProfile(const QString& name) const {
  return connection_profiles_.value(name);
}

void ConfigManager::SaveConnectionProfile(const QString& name,
                                          const ConnectionProfile& profile) {
  connection_profiles_[name] = profile;
}

void ConfigManager::DeleteConnectionProfile(const QString& name) {
  connection_profiles_.remove(name);
}

QString ConfigManager::GetDefaultProfile() const {
  return default_profile_;
}

void ConfigManager::SetDefaultProfile(const QString& name) {
  default_profile_ = name;
}

ConfigManager::ROS2Config ConfigManager::GetROS2Config() const {
  return ros2_config_;
}

void ConfigManager::SetROS2Config(const ROS2Config& config) {
  ros2_config_ = config;
}

ConfigManager::UIConfig ConfigManager::GetUIConfig() const {
  return ui_config_;
}

void ConfigManager::SetUIConfig(const UIConfig& config) {
  ui_config_ = config;
}

QJsonObject ConfigManager::GetCommandTemplates() const {
  return command_templates_;
}

void ConfigManager::SetCommandTemplates(const QJsonObject& templates) {
  command_templates_ = templates;
}

void ConfigManager::AddCommandTemplate(const QString& name,
                                       const QJsonObject& template_data) {
  command_templates_[name] = template_data;
}

void ConfigManager::RemoveCommandTemplate(const QString& name) {
  command_templates_.remove(name);
}

QString ConfigManager::GetConfigFilePath() const {
  return config_file_path_;
}

bool ConfigManager::ExportConfig(const QString& file_path) const {
  Q_UNUSED(file_path)
  return false;
}

bool ConfigManager::ImportConfig(const QString& file_path) {
  Q_UNUSED(file_path)
  return false;
}

void ConfigManager::OnSettingsChanged() {}

void ConfigManager::InitializeDefaults() {}

void ConfigManager::LoadConnectionProfiles() {}

void ConfigManager::SaveConnectionProfiles() {}

void ConfigManager::LoadROS2Config() {}

void ConfigManager::SaveROS2Config() {}

void ConfigManager::LoadUIConfig() {}

void ConfigManager::SaveUIConfig() {}

void ConfigManager::LoadCommandTemplates() {}

void ConfigManager::SaveCommandTemplates() {}

void ConfigManager::ValidateConfig() {}

QJsonObject ConfigManager::ProfileToJson(const ConnectionProfile& profile) const {
  Q_UNUSED(profile)
  return QJsonObject();
}

ConfigManager::ConnectionProfile ConfigManager::ProfileFromJson(const QJsonObject& json) const {
  Q_UNUSED(json)
  return ConnectionProfile();
}

QJsonObject ConfigManager::ROS2ConfigToJson(const ROS2Config& config) const {
  Q_UNUSED(config)
  return QJsonObject();
}

ConfigManager::ROS2Config ConfigManager::ROS2ConfigFromJson(const QJsonObject& json) const {
  Q_UNUSED(json)
  return ROS2Config();
}

QJsonObject ConfigManager::UIConfigToJson(const UIConfig& config) const {
  Q_UNUSED(config)
  return QJsonObject();
}

ConfigManager::UIConfig ConfigManager::UIConfigFromJson(const QJsonObject& json) const {
  Q_UNUSED(json)
  return UIConfig();
}

QStringList ConfigManager::GetControlDropdownOptions(const QString& control_name) const {
  QStringList options;
  try {
    if (auto controls = toml_config_["controls"]) {
      if (auto option_array = controls[control_name.toStdString()]) {
        if (option_array.is_array()) {
          for (const auto& item : *option_array.as_array()) {
            if (item.is_string()) {
              options << QString::fromStdString(item.as_string()->get());
            }
          }
        }
      }
    }
  } catch (const std::exception& e) {
    qWarning() << "Error reading control options:" << e.what();
  }
  return options;
}

QString ConfigManager::GetControlDefaultValue(const QString& control_name) const {
  qDebug() << "Looking for default value for:" << control_name;
  try {
    if (auto controls = toml_config_["controls"]) {
      qDebug() << "Found controls section";
      if (auto default_val = controls[control_name.toStdString()]) {
        qDebug() << "Found key:" << control_name;
        if (default_val.is_string()) {
          QString result = QString::fromStdString(default_val.as_string()->get());
          qDebug() << "Default value:" << result;
          return result;
        } else {
          qDebug() << "Value is not a string";
        }
      } else {
        qDebug() << "Key not found:" << control_name;
      }
    } else {
      qDebug() << "Controls section not found";
    }
  } catch (const std::exception& e) {
    qWarning() << "Error reading control default:" << e.what();
  }
  return QString();
}

bool ConfigManager::GetDarkModeEnabled() const {
  try {
    if (auto ui = toml_config_["ui"]) {
      if (auto dark_mode = ui["dark_mode"]) {
        if (dark_mode.is_boolean()) {
          return dark_mode.as_boolean()->get();
        }
      }
    }
  } catch (const std::exception& e) {
    qWarning() << "Error reading dark mode setting:" << e.what();
  }
  return true; // Default to dark mode
}