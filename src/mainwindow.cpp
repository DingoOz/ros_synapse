// Copyright 2024 TurtleBot3 Controller Project
// Licensed under the Apache License, Version 2.0

#include "mainwindow.h"
#include "commandwidget.h"
#include "sshmanager.h"
#include "ros2executor.h"
#include "processmonitor.h"
#include "configmanager.h"
#include <QApplication>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      tab_widget_(new QTabWidget(this)),
      command_widget_(new CommandWidget(this)),
      ssh_manager_(new SSHManager(this)),
      ros2_executor_(new ROS2Executor(this)),
      process_monitor_(new ProcessMonitor(this)),
      config_manager_(new ConfigManager(this)) {
  setCentralWidget(tab_widget_);
  setWindowTitle("ROS Synapse");
  resize(1200, 800);
  
  tab_widget_->addTab(command_widget_, "Command Builder");
}

MainWindow::~MainWindow() {}

void MainWindow::OnConnectionStatusChanged(bool connected) {
  Q_UNUSED(connected)
}

void MainWindow::OnCommandExecuted(const QString& command, const QString& output) {
  Q_UNUSED(command)
  Q_UNUSED(output)
}

void MainWindow::OnProcessStatusChanged(const QString& process, const QString& status) {
  Q_UNUSED(process)
  Q_UNUSED(status)
}

void MainWindow::UpdateStatusBar() {}

void MainWindow::ShowAbout() {}

void MainWindow::ShowSettings() {}

void MainWindow::SetupUI() {}

void MainWindow::SetupMenuBar() {}

void MainWindow::SetupStatusBar() {}

void MainWindow::SetupConnections() {}

void MainWindow::SetupTheme() {}