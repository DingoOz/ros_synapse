// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0
//
// Main application entry point for TurtleBot3 Controller with dark theme
// initialization and application configuration.

#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>
#include <QDebug>
#include "mainwindow.h"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  
  app.setApplicationName("ROS Synapse");
  app.setApplicationVersion("1.0.0");
  app.setOrganizationName("ROS Synapse");
  app.setOrganizationDomain("ros.synapse");
  
  QString config_dir = QStandardPaths::writableLocation(
      QStandardPaths::AppConfigLocation);
  if (!QDir(config_dir).exists()) {
    QDir().mkpath(config_dir);
  }
  
  app.setStyle(QStyleFactory::create("Fusion"));
  
  QPalette dark_palette;
  dark_palette.setColor(QPalette::Window, QColor(53, 53, 53));
  dark_palette.setColor(QPalette::WindowText, Qt::white);
  dark_palette.setColor(QPalette::Base, QColor(25, 25, 25));
  dark_palette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
  dark_palette.setColor(QPalette::ToolTipBase, Qt::white);
  dark_palette.setColor(QPalette::ToolTipText, Qt::white);
  dark_palette.setColor(QPalette::Text, Qt::white);
  dark_palette.setColor(QPalette::Button, QColor(53, 53, 53));
  dark_palette.setColor(QPalette::ButtonText, Qt::white);
  dark_palette.setColor(QPalette::BrightText, Qt::red);
  dark_palette.setColor(QPalette::Link, QColor(42, 130, 218));
  dark_palette.setColor(QPalette::Highlight, QColor(42, 130, 218));
  dark_palette.setColor(QPalette::HighlightedText, Qt::black);
  app.setPalette(dark_palette);
  
  MainWindow window;
  window.show();
  
  return app.exec();
}