#include "rviz_common/visualization_frame.hpp"

#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include <OgreRenderWindow.h>
#include <OgreMeshManager.h>
#include <OgreMaterialManager.h>

#include <QApplication>  // NOLINT cpplint cannot handle include order here
#include <QCloseEvent>  // NOLINT cpplint cannot handle include order here
#include <QDesktopServices>  // NOLINT cpplint cannot handle include order here
#include <QHBoxLayout>  // NOLINT cpplint cannot handle include order here
#include <QMenu>  // NOLINT cpplint cannot handle include order here
#include <QMenuBar>  // NOLINT cpplint cannot handle include order here
#include <QShortcut>  // NOLINT cpplint cannot handle include order here
#include <QStatusBar>  // NOLINT cpplint cannot handle include order here
#include <QTimer>  // NOLINT cpplint cannot handle include order here
#include <QToolBar>  // NOLINT cpplint cannot handle include order here
#include <QToolButton>  // NOLINT cpplint cannot handle include order here
#include <QWidget>

#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include "flatland_viz/flatland_window2.h"

FlatlandWindow2::FlatlandWindow2(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent)
: QMainWindow(parent),
  app_(nullptr),
  render_panel_(nullptr),
  toolbar_(nullptr),
  manager_(nullptr),
  toolbar_visible_(true),
  rviz_ros_node_(rviz_ros_node)
{
  setWindowTitle("Flatland Viz");
}

FlatlandWindow2::~FlatlandWindow2()
{
  delete manager_;
  delete render_panel_;

  /*
  for (auto & custom_panel : custom_panels_) {
    delete custom_panel.dock;
  }

  delete panel_factory_;
*/
}

rviz_rendering::RenderWindow * FlatlandWindow2::getRenderWindow()
{
  return render_panel_->getRenderWindow();
}

void FlatlandWindow2::setApp(QApplication * app)
{
  app_ = app;
}

void FlatlandWindow2::setStatus(const QString & message)
{
  Q_EMIT statusUpdate(message);
}

void FlatlandWindow2::initialize(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node) {
  /*
  QDir app_icon_path(QString::fromStdString(package_path_) + "/icons/package.png");
  QIcon app_icon(app_icon_path.absolutePath());
  app_->setWindowIcon(app_icon);
   */

  Q_EMIT statusUpdate("Initializing");

  QWidget * central_widget = new QWidget(this);
  QHBoxLayout * central_layout = new QHBoxLayout;
  central_layout->setSpacing(0);
  central_layout->setMargin(0);

  render_panel_ = new rviz_common::RenderPanel(central_widget);
  central_layout->addWidget(render_panel_, 1);
  central_widget->setLayout(central_layout);

  initMenus();

  initToolbars();

  setCentralWidget(central_widget);

  render_panel_->getRenderWindow()->initialize();

  auto clock = rviz_ros_node.lock()->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(render_panel_, rviz_ros_node, this, clock);
  //panel_factory_ = new rviz_common::PanelFactory(rviz_ros_node_, manager_);

  render_panel_->initialize(manager_);

  rviz_common::ToolManager * tool_man = manager_->getToolManager();

  connect(manager_, SIGNAL(configChanged()), this, SLOT(setDisplayConfigModified()));
  connect(tool_man, SIGNAL(toolAdded(Tool*)), this, SLOT(addTool(Tool*)));
  connect(tool_man, SIGNAL(toolRemoved(Tool*)), this, SLOT(removeTool(Tool*)));
  connect(tool_man, SIGNAL(toolRefreshed(Tool*)), this, SLOT(refreshTool(Tool*)));
  connect(tool_man, SIGNAL(toolChanged(Tool*)), this, SLOT(indicateToolIsCurrent(Tool*)));

  manager_->initialize();

  Q_EMIT statusUpdate("RViz is ready.");

  connect(
    manager_, SIGNAL(statusUpdate(const QString&)), this,
    SIGNAL(statusUpdate(const QString&)));
}

rviz_common::VisualizationManager *
FlatlandWindow2::getManager()
{
  return manager_;
}

void FlatlandWindow2::initMenus()
{
  QAction * fullscreen_action = view_menu_->addAction(
    "&Fullscreen", this, SLOT(
      setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  this->addAction(fullscreen_action);  // Also add to window, or the shortcut doest work
                                       // when the menu is hidden.
  connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action, SLOT(setChecked(bool)));
  new QShortcut(Qt::Key_Escape, this, SLOT(exitFullScreen()));
  view_menu_->addSeparator();
}

void FlatlandWindow2::initToolbars()
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
  add_tool_action_->setIcon(rviz_common::loadPixmap("package://rviz_common/icons/plus.png"));
  toolbar_->addAction(add_tool_action_);
  connect(add_tool_action_, SIGNAL(triggered()), this, SLOT(openNewToolDialog()));

  remove_tool_menu_ = new QMenu();
  QToolButton * remove_tool_button = new QToolButton();
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip("Remove a tool from the toolbar");
  remove_tool_button->setIcon(rviz_common::loadPixmap("package://rviz_common/icons/minus.png"));
  toolbar_->addWidget(remove_tool_button);
  connect(
    remove_tool_menu_, SIGNAL(triggered(QAction*)), this, SLOT(
      onToolbarRemoveTool(QAction*)));
}

/*
void FlatlandWindow2::openNewPanelDialog()
{
  QString class_id;
  QString display_name;
  QStringList empty;

  NewObjectDialog * dialog = new NewObjectDialog(
    panel_factory_,
    "Panel",
    empty,
    empty,
    &class_id,
    &display_name,
    this);
  if (dialog->exec() == QDialog::Accepted) {
    addPanelByName(display_name, class_id);
  }
}
 */

/*
void FlatlandWindow2::openNewToolDialog()
{
  QString class_id;
  QStringList empty;
  ToolManager * tool_man = manager_->getToolManager();

  NewObjectDialog * dialog = new NewObjectDialog(
    tool_man->getFactory(),
    "Tool",
    empty,
    tool_man->getToolClasses(),
    &class_id);
  if (dialog->exec() == QDialog::Accepted) {
    tool_man->addTool(class_id);
  }
  activateWindow();  // Force keyboard focus back on main window.
}
 */

rviz_common::PanelDockWidget * FlatlandWindow2::addPane(
    const QString & name, QWidget * panel,
    Qt::DockWidgetArea area, bool floating)
{
  rviz_common::PanelDockWidget * dock = new rviz_common::PanelDockWidget(name);
  dock->setContentWidget(panel);
  dock->setFloating(floating);
  dock->setObjectName(name);   // QMainWindow::saveState() needs objectName to be set.
  addDockWidget(area, dock);

  // we want to know when that panel becomes visible
  connect(dock, SIGNAL(visibilityChanged(bool)), this, SLOT(onDockPanelVisibilityChange(bool)));
  connect(this, SIGNAL(fullScreenChange(bool)), dock, SLOT(overrideVisibility(bool)));

  QAction * toggle_action = dock->toggleViewAction();
  view_menu_->addAction(toggle_action);

  connect(toggle_action, SIGNAL(triggered(bool)), this, SLOT(setDisplayConfigModified()));
  connect(dock, SIGNAL(closed()), this, SLOT(setDisplayConfigModified()));

  return dock;
}

void FlatlandWindow2::addTool(rviz_common::Tool * tool)
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

void FlatlandWindow2::onToolbarActionTriggered(QAction * action)
{
  rviz_common::Tool * tool = action_to_tool_map_[action];

  if (tool) {
    manager_->getToolManager()->setCurrentTool(tool);
  }
}

void FlatlandWindow2::onToolbarRemoveTool(QAction * remove_tool_menu_action)
{
  QString name = remove_tool_menu_action->text();
  for (int i = 0; i < manager_->getToolManager()->numTools(); i++) {
    rviz_common::Tool * tool = manager_->getToolManager()->getTool(i);
    if (tool->getName() == name) {
      manager_->getToolManager()->removeTool(i);
      return;
    }
  }
}

void FlatlandWindow2::removeTool(rviz_common::Tool * tool)
{
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
      remove_tool_menu_->removeAction(removal_action);
      break;
    }
  }
}

void FlatlandWindow2::refreshTool(rviz_common::Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
}

void FlatlandWindow2::indicateToolIsCurrent(rviz_common::Tool * tool)
{
  QAction * action = tool_to_action_map_[tool];
  if (action) {
    action->setChecked(true);
  }
}

QWidget * FlatlandWindow2::getParentWindow()
{
  return this;
}

void FlatlandWindow2::onDeletePanel()
{
  // This should only be called as a SLOT from a QAction in the
  // "delete panel" submenu, so the sender will be one of the QActions
  // stored as "delete_action" in a PanelRecord.  This code looks for
  // a delete_action in custom_panels_ matching sender() and removes
  // the panel associated with it.
  if (QAction * action = qobject_cast<QAction *>(sender())) {
    for (int i = 0; i < custom_panels_.size(); i++) {
      if (custom_panels_[i].delete_action == action) {
        delete custom_panels_[i].dock;
        custom_panels_.removeAt(i);
        action->deleteLater();
        if (delete_view_menu_->actions().size() == 1 &&
          delete_view_menu_->actions().first() == action)
        {
          delete_view_menu_->setEnabled(false);
        }
        return;
      }
    }
  }
}

void FlatlandWindow2::setFullScreen(bool full_screen)
{
  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";
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

void FlatlandWindow2::exitFullScreen()
{
  setFullScreen(false);
}