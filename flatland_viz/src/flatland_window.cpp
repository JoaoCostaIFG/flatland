#include "flatland_viz/flatland_window.h"

#include <exception>
#include <fstream>
#include <memory>
#include <string>

#include <OgreRenderWindow.h>
#include <OgreMaterialManager.h>

#include <QApplication>  // NOLINT cpplint cannot handle include order here
#include <QCloseEvent>  // NOLINT cpplint cannot handle include order here
#include <QDesktopServices>  // NOLINT cpplint cannot handle include order here
#include <QDir>  // NOLINT cpplint cannot handle include order here
#include <QFile>  // NOLINT cpplint cannot handle include order here
#include <QFileDialog>  // NOLINT cpplint cannot handle include order here
#include <QHBoxLayout>  // NOLINT cpplint cannot handle include order here
#include <QMenu>  // NOLINT cpplint cannot handle include order here
#include <QMenuBar>  // NOLINT cpplint cannot handle include order here
#include <QMessageBox>  // NOLINT cpplint cannot handle include order here
#include <QShortcut>  // NOLINT cpplint cannot handle include order here
#include <QSplashScreen>  // NOLINT cpplint cannot handle include order here
#include <QStatusBar>  // NOLINT cpplint cannot handle include order here
#include <QTimer>  // NOLINT cpplint cannot handle include order here
#include <QToolBar>  // NOLINT cpplint cannot handle include order here
#include <QToolButton>  // NOLINT cpplint cannot handle include order here

#include <rviz_common/load_resource.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/yaml_config_writer.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#define CONFIG_EXTENSION "flatland"
#define CONFIG_EXTENSION_WILDCARD "*." CONFIG_EXTENSION
#define RECENT_CONFIG_COUNT 10

using namespace rviz_common;

FlatlandWindow::FlatlandWindow(
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent)
    : QMainWindow(parent),
      app_(nullptr),
      render_panel_(nullptr),
      file_menu_(nullptr),
      recent_configs_menu_(nullptr),
      toolbar_(nullptr),
      manager_(nullptr),
      splash_(nullptr),
      toolbar_actions_(nullptr),
      add_tool_action_(nullptr),
      remove_tool_menu_(nullptr),
      initialized_(false),
      loading_(false),
      post_load_timer_(new QTimer(this)),
      frame_count_(0),
      toolbar_visible_(true),
      rviz_ros_node_(rviz_ros_node)
{
  setObjectName("FlatlandWindow");

  post_load_timer_->setSingleShot(true);
  connect(post_load_timer_, SIGNAL(timeout()), this, SLOT(markLoadingDone()));

  package_path_ = ament_index_cpp::get_package_share_directory("rviz_common");
  QDir splash_path(QString::fromStdString(package_path_) + "/images/splash.png");
  splash_path_ = splash_path.absolutePath();

  status_label_ = new QLabel("");
  statusBar()->addPermanentWidget(status_label_, 1);
  connect(this, SIGNAL(statusUpdate(const QString&)), status_label_, SLOT(setText(const QString&)));

  fps_label_ = new QLabel("");
  fps_label_->setMinimumWidth(40);
  fps_label_->setAlignment(Qt::AlignRight);
  statusBar()->addPermanentWidget(fps_label_, 0);
  original_status_bar_ = statusBar();

  setWindowTitle("Flatland Viz");
}

FlatlandWindow::~FlatlandWindow()
{
  delete manager_;
  delete render_panel_;
}

rviz_rendering::RenderWindow * FlatlandWindow::getRenderWindow()
{
  return render_panel_->getRenderWindow();
}

void FlatlandWindow::setApp(QApplication * app)
{
  app_ = app;
}

void FlatlandWindow::setStatus(const QString & message)
{
  Q_EMIT statusUpdate(message);
}

void FlatlandWindow::updateFps()
{
  frame_count_++;
  auto wall_diff = std::chrono::steady_clock::now() - last_fps_calc_time_;

  if (wall_diff > std::chrono::seconds(1)) {
    using std::chrono::duration;
    float fps = frame_count_ / std::chrono::duration_cast<duration<double>>(wall_diff).count();
    frame_count_ = 0;
    last_fps_calc_time_ = std::chrono::steady_clock::now();
    if (original_status_bar_ == statusBar()) {
      fps_label_->setText(QString::number(static_cast<int>(fps)) + QString(" fps"));
    }
  }
}

void FlatlandWindow::closeEvent(QCloseEvent * event)
{
  if (prepareToExit()) {
    event->accept();
  } else {
    event->ignore();
  }
}

void FlatlandWindow::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  setStatus("");
}

void FlatlandWindow::setSplashPath(const QString & splash_path)
{
  splash_path_ = splash_path;
}

void FlatlandWindow::initialize(
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    const QString & display_config_file)
{
  initConfigs();

  loadPersistentSettings();

  QDir app_icon_path(QString::fromStdString(package_path_) + "/icons/package.png");
  QIcon app_icon(app_icon_path.absolutePath());
  app_->setWindowIcon(app_icon);

  if (splash_path_ != "") {
    QPixmap splash_image(splash_path_);
    splash_ = new QSplashScreen(splash_image);
    splash_->show();
    connect(this, SIGNAL(statusUpdate(const QString&)), splash_, SLOT(showMessage(const QString&)));
  }
  Q_EMIT statusUpdate("Initializing");

  // Periodically process events for the splash screen.
  // See: http://doc.qt.io/qt-5/qsplashscreen.html#details
  if (app_) {app_->processEvents();}

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  QWidget * central_widget = new QWidget(this);
  QHBoxLayout * central_layout = new QHBoxLayout;
  central_layout->setSpacing(0);
  central_layout->setMargin(0);

  render_panel_ = new RenderPanel(central_widget);

  central_layout->addWidget(render_panel_, 1);

  central_widget->setLayout(central_layout);

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  initMenus();

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  initToolbars();

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  setCentralWidget(central_widget);

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  render_panel_->getRenderWindow()->initialize();

  auto clock = rviz_ros_node.lock()->get_raw_node()->get_clock();
  manager_ = new VisualizationManager(render_panel_, rviz_ros_node, this, clock);

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  render_panel_->initialize(manager_);

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  ToolManager * tool_man = manager_->getToolManager();

  connect(manager_, SIGNAL(configChanged()), this, SLOT(setDisplayConfigModified()));
  connect(tool_man, SIGNAL(toolAdded(Tool*)), this, SLOT(addTool(Tool*)));
  connect(tool_man, SIGNAL(toolRemoved(Tool*)), this, SLOT(removeTool(Tool*)));
  connect(tool_man, SIGNAL(toolRefreshed(Tool*)), this, SLOT(refreshTool(Tool*)));
  connect(tool_man, SIGNAL(toolChanged(Tool*)), this, SLOT(indicateToolIsCurrent(Tool*)));

  manager_->initialize();

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  if (display_config_file != "") {
    loadDisplayConfig(display_config_file);
  } else {
    loadDisplayConfig(QString::fromStdString(default_display_config_file_));
  }

  // Periodically process events for the splash screen.
  if (app_) {app_->processEvents();}

  delete splash_;
  splash_ = nullptr;

  initialized_ = true;
  Q_EMIT statusUpdate("Flatland is ready.");

  connect(manager_, SIGNAL(preUpdate()), this, SLOT(updateFps()));
  connect(
      manager_, SIGNAL(statusUpdate(const QString&)), this,
      SIGNAL(statusUpdate(const QString&)));
}

VisualizationManager *
FlatlandWindow::getManager()
{
  return manager_;
}

void FlatlandWindow::initConfigs()
{
  home_dir_ = QDir::toNativeSeparators(QDir::homePath()).toStdString();

  config_dir_ = "";
  if (home_dir_ != "") {
    config_dir_ += home_dir_ + "/";
  }
  config_dir_ += ".rviz2";
  persistent_settings_file_ = config_dir_ + "/persistent_settings";
  default_display_config_file_ = config_dir_ + "/default." CONFIG_EXTENSION;

  QFile config_dir_as_file(QString::fromStdString(config_dir_));
  QDir config_dir_as_dir(QString::fromStdString(config_dir_));
  if (config_dir_as_file.exists() && !config_dir_as_dir.exists()) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
        "Moving file [" << config_dir_.c_str() << "] out of the way to recreate it as a directory.");
    std::string backup_file = config_dir_ + ".bak";

    if (!config_dir_as_file.rename(QString::fromStdString(backup_file))) {
      RVIZ_COMMON_LOG_ERROR("Failed to rename config directory while backing up.");
    }
  }

  QDir config_dir_as_qdir;
  if (!config_dir_as_qdir.mkpath(QString::fromStdString(config_dir_))) {
    RVIZ_COMMON_LOG_ERROR_STREAM("failed to make config dir: " << config_dir_);
  }
}

void FlatlandWindow::loadPersistentSettings()
{
  YamlConfigReader reader;
  Config config;
  reader.readFile(config, QString::fromStdString(persistent_settings_file_));
  if (!reader.error()) {
    QString last_config_dir;
    if (config.mapGetString("Last Config Dir", &last_config_dir))
    {
      last_config_dir_ = last_config_dir.toStdString();
    }

    Config recent_configs_list = config.mapGetChild("Recent Configs");
    recent_configs_.clear();
    int num_recent = recent_configs_list.listLength();
    for (int i = 0; i < num_recent; i++) {
      recent_configs_.push_back(
          recent_configs_list.listChildAt(
              i).getValue().toString().toStdString());
    }
  } else {
    RVIZ_COMMON_LOG_ERROR(qPrintable(reader.errorMessage()));
  }
}

void FlatlandWindow::savePersistentSettings()
{
  Config config;
  config.mapSetValue("Last Config Dir", QString::fromStdString(last_config_dir_));
  Config recent_configs_list = config.mapMakeChild("Recent Configs");
  for (D_string::iterator it = recent_configs_.begin(); it != recent_configs_.end(); ++it) {
    recent_configs_list.listAppendNew().setValue(QString::fromStdString(*it));
  }

  YamlConfigWriter writer;
  writer.writeFile(config, QString::fromStdString(persistent_settings_file_));

  if (writer.error()) {
    RVIZ_COMMON_LOG_ERROR(qPrintable(writer.errorMessage()));
  }
}

void FlatlandWindow::initMenus()
{
  file_menu_ = menuBar()->addMenu("&File");

  QAction * file_menu_open_action = file_menu_->addAction(
      "&Open Config", this, SLOT(
      onOpen()), QKeySequence("Ctrl+O"));
  this->addAction(file_menu_open_action);
  QAction * file_menu_save_action = file_menu_->addAction(
      "&Save Config", this, SLOT(
      onSave()), QKeySequence("Ctrl+S"));
  this->addAction(file_menu_save_action);
  QAction * file_menu_save_as_action =
      file_menu_->addAction(
          "Save Config &As", this, SLOT(onSaveAs()),
          QKeySequence("Ctrl+Shift+S"));
  this->addAction(file_menu_save_as_action);

  recent_configs_menu_ = file_menu_->addMenu("&Recent Configs");
  file_menu_->addSeparator();

  QAction * file_menu_quit_action = file_menu_->addAction(
      "&Quit", this, SLOT(
      close()), QKeySequence("Ctrl+Q"));
  this->addAction(file_menu_quit_action);

  view_menu_ = menuBar()->addMenu("&Panels");
  QAction * fullscreen_action = view_menu_->addAction(
      "&Fullscreen", this, SLOT(
      setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  // Also add to window, or the shortcut doest work when the menu is hidden
  this->addAction(fullscreen_action);
  connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action, SLOT(setChecked(bool)));
  new QShortcut(Qt::Key_Escape, this, SLOT(exitFullScreen()));
  view_menu_->addSeparator();

  QMenu * help_menu = menuBar()->addMenu("&Help");
  help_menu->addAction("Open Flatland wiki in browser", this, SLOT(onHelpWiki()));
  help_menu->addSeparator();
  help_menu->addAction("&About", this, SLOT(onHelpAbout()));
}

void FlatlandWindow::initToolbars()
{
  QFont font;
  font.setPointSize(font.pointSizeF() * 0.9);

  // make toolbar with plugin tools

  toolbar_ = addToolBar("Tools");
  toolbar_->setFont(font);
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setObjectName("Tools");
  toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toolbar_actions_ = new QActionGroup(this);
  connect(
      toolbar_actions_, SIGNAL(triggered(QAction*)), this,
      SLOT(onToolbarActionTriggered(QAction*)));
  view_menu_->addAction(toolbar_->toggleViewAction());

  add_tool_action_ = new QAction("", toolbar_actions_);
  add_tool_action_->setToolTip("Add a new tool");
  add_tool_action_->setIcon(loadPixmap("package://rviz_common/icons/plus.png"));
  toolbar_->addAction(add_tool_action_);
  connect(add_tool_action_, SIGNAL(triggered()), this, SLOT(openNewToolDialog()));

  remove_tool_menu_ = new QMenu();
  QToolButton * remove_tool_button = new QToolButton();
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip("Remove a tool from the toolbar");
  remove_tool_button->setIcon(loadPixmap("package://rviz_common/icons/minus.png"));
  toolbar_->addWidget(remove_tool_button);
  connect(
      remove_tool_menu_, SIGNAL(triggered(QAction*)), this, SLOT(
      onToolbarRemoveTool(QAction*)));
}

void FlatlandWindow::hideDockImpl(Qt::DockWidgetArea area, bool hide)
{
  QList<PanelDockWidget *> dock_widgets = findChildren<PanelDockWidget *>();

  for (QList<PanelDockWidget *>::iterator it = dock_widgets.begin(); it != dock_widgets.end();
       it++)
  {
    Qt::DockWidgetArea curr_area = dockWidgetArea(*it);
    if (area == curr_area) {
      (*it)->setCollapsed(hide);
    }
    // allow/disallow docking to this area for all widgets
    if (hide) {
      (*it)->setAllowedAreas((*it)->allowedAreas() & ~area);
    } else {
      (*it)->setAllowedAreas((*it)->allowedAreas() | area);
    }
  }
}

void FlatlandWindow::openNewToolDialog()
{
  RCLCPP_ERROR(rclcpp::get_logger("flatland_viz"), "openNewToolDialog called");

  QString class_id;
  QStringList empty;
  ToolManager * tool_man = manager_->getToolManager();

  /*
  NewObjectDialog * dialog = new NewObjectDialog(
      tool_man->getFactory(),
      "Tool",
      empty,
      tool_man->getToolClasses(),
      &class_id);
  manager_->stopUpdate();
  if (dialog->exec() == QDialog::Accepted) {
    tool_man->addTool(class_id);
  }
  manager_->startUpdate();
   */
  activateWindow();  // Force keyboard focus back on main window.
}

void FlatlandWindow::updateRecentConfigMenu()
{
  recent_configs_menu_->clear();

  D_string::iterator it = recent_configs_.begin();
  D_string::iterator end = recent_configs_.end();
  for (; it != end; ++it) {
    if (*it != "") {
      std::string display_name = *it;
      if (display_name == default_display_config_file_) {
        display_name += " (default)";
      }
      if (display_name.find(home_dir_) == 0) {
        display_name = (
            QDir::homePath() + "/" +
                QString::fromStdString(display_name.substr(home_dir_.size()))
        ).toStdString();
      }
      QString qdisplay_name = QString::fromStdString(display_name);
      QAction * action = new QAction(qdisplay_name, this);
      action->setData(QString::fromStdString(*it));
      connect(action, SIGNAL(triggered()), this, SLOT(onRecentConfigSelected()));
      recent_configs_menu_->addAction(action);
    }
  }
}

void FlatlandWindow::markRecentConfig(const std::string & path)
{
  D_string::iterator it = std::find(recent_configs_.begin(), recent_configs_.end(), path);
  if (it != recent_configs_.end()) {
    recent_configs_.erase(it);
  }

  recent_configs_.push_front(path);

  if (recent_configs_.size() > RECENT_CONFIG_COUNT) {
    recent_configs_.pop_back();
  }

  updateRecentConfigMenu();
}

void FlatlandWindow::loadDisplayConfig(const QString & qpath)
{
  std::string path = qpath.toStdString();
  QFileInfo path_info(qpath);
  std::string actual_load_path = path;
  if (!path_info.exists() || path_info.isDir()) {
    actual_load_path = package_path_ + "/default.rviz";
    if (!QFile(QString::fromStdString(actual_load_path)).exists()) {
      RVIZ_COMMON_LOG_ERROR_STREAM(
          "Default display config '" <<
                                     actual_load_path.c_str() << "' not found.  Flatland will be very empty at first.");
      return;
    }
  }

  // Check if we have unsaved changes to the current config the same
  // as we do during exit, with the same option to cancel.
  if (!prepareToExit()) {
    return;
  }

  setWindowModified(false);
  loading_ = true;

  YamlConfigReader reader;
  Config config;
  reader.readFile(config, QString::fromStdString(actual_load_path));
  if (!reader.error()) {
    try {
      load(config);
    } catch (const std::exception & e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Could not load display config: " << e.what());
    }
  }

  markRecentConfig(path);

  setDisplayConfigFile(path);

  last_config_dir_ = path_info.absolutePath().toStdString();

  post_load_timer_->start(1000);
}

void FlatlandWindow::markLoadingDone()
{
  loading_ = false;
}

void FlatlandWindow::setDisplayConfigModified()
{
  if (!loading_) {
    if (!isWindowModified()) {
      setWindowModified(true);
    }
  }
}

void FlatlandWindow::setDisplayConfigFile(const std::string & path)
{
  display_config_file_ = path;

  std::string title;
  if (path == default_display_config_file_) {
    title = "Flatland Viz[*]";
  } else {
    title = QDir::toNativeSeparators(QString::fromStdString(path)).toStdString() + "[*] - Flatland Viz";
  }
  setWindowTitle(QString::fromStdString(title));
}

bool FlatlandWindow::saveDisplayConfig(const QString & path)
{
  Config config;
  save(config);

  YamlConfigWriter writer;
  writer.writeFile(config, path);

  if (writer.error()) {
    RVIZ_COMMON_LOG_ERROR(qPrintable(writer.errorMessage()));
    error_message_ = writer.errorMessage();
    return false;
  } else {
    setWindowModified(false);
    error_message_ = "";
    return true;
  }
}

QString FlatlandWindow::getErrorMessage() const
{
  return error_message_;
}

void FlatlandWindow::save(Config config)
{
  manager_->save(config.mapMakeChild("Visualization Manager"));
  saveWindowGeometry(config.mapMakeChild("Window Geometry"));
}

void FlatlandWindow::load(const Config & config)
{
  manager_->load(config.mapGetChild("Visualization Manager"));
  loadWindowGeometry(config.mapGetChild("Window Geometry"));
}

void FlatlandWindow::loadWindowGeometry(const Config & config)
{
  int x, y;
  if (config.mapGetInt("X", &x) &&
      config.mapGetInt("Y", &y))
  {
    move(x, y);
  }

  int width, height;
  if (config.mapGetInt("Width", &width) &&
      config.mapGetInt("Height", &height))
  {
    resize(width, height);
  }

  QString main_window_config;
  if (config.mapGetString("QMainWindow State", &main_window_config)) {
    restoreState(QByteArray::fromHex(qPrintable(main_window_config)));
  }

  // load panel dock widget states (collapsed or not)
  QList<PanelDockWidget *> dock_widgets = findChildren<PanelDockWidget *>();

  for (QList<PanelDockWidget *>::iterator it = dock_widgets.begin(); it != dock_widgets.end();
       it++)
  {
    Config itConfig = config.mapGetChild((*it)->windowTitle());

    if (itConfig.isValid()) {
      (*it)->load(itConfig);
    }
  }
}

void FlatlandWindow::saveWindowGeometry(Config config)
{
  config.mapSetValue("X", x());
  config.mapSetValue("Y", y());
  config.mapSetValue("Width", width());
  config.mapSetValue("Height", height());

  QByteArray window_state = saveState().toHex();
  config.mapSetValue("QMainWindow State", window_state.constData());

  // save panel dock widget states (collapsed or not)
  QList<PanelDockWidget *> dock_widgets = findChildren<PanelDockWidget *>();

  for (QList<PanelDockWidget *>::iterator it = dock_widgets.begin(); it != dock_widgets.end();
       it++)
  {
    (*it)->save(config.mapMakeChild((*it)->windowTitle()));
  }
}

bool FlatlandWindow::prepareToExit()
{
  if (!initialized_) {
    return true;
  }

  savePersistentSettings();

  if (isWindowModified()) {
    QMessageBox box(this);
    box.setText("There are unsaved changes.");
    box.setInformativeText(QString::fromStdString("Save changes to " + display_config_file_ + "?"));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);
    int result = box.exec();
    switch (result) {
      case QMessageBox::Save:
        if (saveDisplayConfig(QString::fromStdString(display_config_file_))) {
          return true;
        } else {
          QMessageBox box(this);
          box.setWindowTitle("Failed to save.");
          box.setText(getErrorMessage());
          box.setInformativeText(
              QString::fromStdString(
                  "Save copy of " + display_config_file_ + " to another file?"));
          box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
          box.setDefaultButton(QMessageBox::Save);
          int result = box.exec();
          switch (result) {
            case QMessageBox::Save:
              onSaveAs();
              return true;
            case QMessageBox::Discard:
              return true;
            default:
              return false;
          }
        }
      case QMessageBox::Discard:
        return true;
      default:
        return false;
    }
  } else {
    return true;
  }
}

void FlatlandWindow::onOpen()
{
  QString filename = QFileDialog::getOpenFileName(
      this, "Choose a file to open",
      QString::fromStdString(last_config_dir_),
      "Flatland config files (" CONFIG_EXTENSION_WILDCARD ")");

  if (!filename.isEmpty()) {
    if (!QFile(filename).exists()) {
      QString message = filename + " does not exist!";
      QMessageBox::critical(this, "Config file does not exist", message);
      return;
    }

    loadDisplayConfig(filename);
  }
}

void FlatlandWindow::onSave()
{
  if (!initialized_) {
    return;
  }

  savePersistentSettings();

  if (!saveDisplayConfig(QString::fromStdString(display_config_file_))) {
    QMessageBox box(this);
    box.setWindowTitle("Failed to save.");
    box.setText(getErrorMessage());
    box.setInformativeText(
        QString::fromStdString(
            "Save copy of " + display_config_file_ + " to another file?"));
    box.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Save);
    if (box.exec() == QMessageBox::Save) {
      onSaveAs();
    }
  }
}

void FlatlandWindow::onSaveAs()
{
  QString q_filename = QFileDialog::getSaveFileName(
      this, "Choose a file to save to",
      QString::fromStdString(last_config_dir_),
      "Flatland config files (" CONFIG_EXTENSION_WILDCARD ")");

  if (!q_filename.isEmpty()) {
    if (!q_filename.endsWith("." CONFIG_EXTENSION)) {
      q_filename += "." CONFIG_EXTENSION;
    }

    if (!saveDisplayConfig(q_filename)) {
      QMessageBox::critical(this, "Failed to save.", getErrorMessage());
    }

    std::string filename = q_filename.toStdString();
    markRecentConfig(filename);
    last_config_dir_ = QDir(q_filename).dirName().toStdString();
    setDisplayConfigFile(filename);
  }
}

void FlatlandWindow::onRecentConfigSelected()
{
  QAction * action = dynamic_cast<QAction *>(sender());
  if (action) {
    QString path = action->data().toString();
    if (path.size() != 0) {
      if (!QFile(path).exists()) {
        QString message = path + " does not exist!";
        QMessageBox::critical(this, "Config file does not exist", message);
        return;
      }

      loadDisplayConfig(path);
    }
  }
}

void FlatlandWindow::addTool(Tool * tool)
{
  QAction * action = new QAction(tool->getName(), toolbar_actions_);
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
  action->setCheckable(true);
  toolbar_->insertAction(add_tool_action_, action);
  action_to_tool_map_[action] = tool;
  tool_to_action_map_[tool] = action;

  remove_tool_menu_->addAction(tool->getName());
}

void FlatlandWindow::onToolbarActionTriggered(QAction * action)
{
  RCLCPP_ERROR(rclcpp::get_logger("flatland_viz"), "onToolbarActionTriggered called");

  Tool* current_tool = manager_->getToolManager()->getCurrentTool();
  Tool* tool = action_to_tool_map_[action];

  if (!tool) return;
  manager_->getToolManager()->setCurrentTool(tool);

  // If the simulation pause/resume tool was clicked, automatically and
  // immediately switch back to the previously active tool
  if (tool->getClassId().toStdString() == "flatland_viz/PauseSim") {
    manager_->getToolManager()->setCurrentTool(current_tool);
    tool = current_tool;
    indicateToolIsCurrent(tool);
  }

  // Show or hide interactive markers depending on whether interact mode is
  // active
  viz_->enableInteractiveMarkers(tool->getClassId().toStdString() == "rviz/Interact");
}

void FlatlandWindow::onToolbarRemoveTool(QAction * remove_tool_menu_action)
{
  RCLCPP_ERROR(rclcpp::get_logger("flatland_viz"), "onToolbarRemoveTool called");
  QString name = remove_tool_menu_action->text();

  for (int i = 0; i < manager_->getToolManager()->numTools(); i++) {
    Tool * tool = manager_->getToolManager()->getTool(i);
    if (tool->getName() == name) {
      manager_->getToolManager()->removeTool(i);
      return;
    }
  }
}

void FlatlandWindow::removeTool(Tool * tool)
{
  RCLCPP_ERROR(rclcpp::get_logger("flatland_viz"), "removeTool called");
  QAction * action = tool_to_action_map_[tool];
  if (action) {
    toolbar_actions_->removeAction(action);
    toolbar_->removeAction(action);
    tool_to_action_map_.erase(tool);
    action_to_tool_map_.erase(action);
  }
  QString tool_name = tool->getName();
  QList<QAction *> remove_tool_actions = remove_tool_menu_->actions();
  for (int i = 0; i < remove_tool_actions.size(); i++) {
    QAction * removal_action = remove_tool_actions.at(i);
    if (removal_action->text() == tool_name) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("flatland_viz"), "Removing --------> " << tool_name.toStdString());
      remove_tool_menu_->removeAction(removal_action);
      break;
    }
  }
}

void FlatlandWindow::refreshTool(Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
}

void FlatlandWindow::indicateToolIsCurrent(Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  if (action) {
    action->setChecked(true);
  }
}

void FlatlandWindow::onHelpWiki()
{
  QDesktopServices::openUrl(QUrl("https://flatland-simulator.readthedocs.io/en/latest/"));
}

void FlatlandWindow::onHelpAbout()
{
  QString about_text = QString(
      "This is Flatland for ROS 2.\n"
      "Using RViz version %1 (%2).\n"
      "\n"
      "Compiled against Qt version %3."
      "\n"
      "Compiled against OGRE version %4.%5.%6%7 (%8).")
      .arg(QT_VERSION_STR)
      .arg(OGRE_VERSION_MAJOR)
      .arg(OGRE_VERSION_MINOR)
      .arg(OGRE_VERSION_PATCH)
      .arg(OGRE_VERSION_SUFFIX)
      .arg(OGRE_VERSION_NAME);

  QMessageBox::about(QApplication::activeWindow(), "About", about_text);
}

QWidget * FlatlandWindow::getParentWindow()
{
  return this;
}

void FlatlandWindow::setFullScreen(bool full_screen)
{
  auto state = windowState();
  if (full_screen == state.testFlag(Qt::WindowFullScreen)) {
    return;
  }
  Q_EMIT (fullScreenChange(full_screen));

  // When switching to fullscreen, remember visibility state of toolbar
  if (full_screen) {
    toolbar_visible_ = toolbar_->isVisible();
  }
  menuBar()->setVisible(!full_screen);
  toolbar_->setVisible(!full_screen && toolbar_visible_);
  statusBar()->setVisible(!full_screen);

  if (full_screen) {
    setWindowState(state | Qt::WindowFullScreen);
  } else {
    setWindowState(state & ~Qt::WindowFullScreen);
  }
  show();
}

void FlatlandWindow::exitFullScreen()
{
  setFullScreen(false);
}


PanelDockWidget * FlatlandWindow::addPane(
    const QString & name, QWidget * panel,
    Qt::DockWidgetArea area, bool floating)
{
  PanelDockWidget * dock;
  dock = new PanelDockWidget(name);
  dock->setContentWidget(panel);
  dock->setFloating(floating);
  dock->setObjectName(name);   // QMainWindow::saveState() needs objectName to be set.
  addDockWidget(area, dock);

  // we want to know when that panel becomes visible
  connect(this, SIGNAL(fullScreenChange(bool)), dock, SLOT(overrideVisibility(bool)));

  QAction * toggle_action = dock->toggleViewAction();
  view_menu_->addAction(toggle_action);

  connect(toggle_action, SIGNAL(triggered(bool)), this, SLOT(setDisplayConfigModified()));
  connect(dock, SIGNAL(closed()), this, SLOT(setDisplayConfigModified()));

  return dock;
}
