#include <chrono>
#include <deque>
#include <map>
#include <string>

#include <QList>  // NOLINT: cpplint is unable to handle the include order here
#include <QMainWindow>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <Qt>  // NOLINT: cpplint is unable to handle the include order here

#include <rviz_common/config.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

/// The main window.
class FlatlandWindow2 : public QMainWindow, public rviz_common::WindowManagerInterface
{
  Q_OBJECT

public:
  explicit FlatlandWindow2(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent = nullptr);

  ~FlatlandWindow2() override;

  rviz_rendering::RenderWindow * getRenderWindow();

  /// Set the QApplication, this should be called directly after construction.
  void
  setApp(QApplication * app);

  /// Initialize the VisualizationFrame and create the VisualizationManager.
  /**
   * This function must be called before load(), save(), getManager(),
   * or addPanelByName(), because it creates the VisualizationManager
   * instance which those calls depend on.
   *
   * This function also calls VisualizationManager::initialize(),
   * which means it will start the update timer and generally get
   * things rolling.
   */
  void
  initialize(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  /// Return the visualization manager.
  rviz_common::VisualizationManager *
  getManager();

  // Overriden from WindowManagerInterface:
  QWidget *
  getParentWindow() override;

  // Overriden from WindowManagerInterface:
  rviz_common::PanelDockWidget *
  addPane(
    const QString & name,
    QWidget * panel,
    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
    bool floating = true) override;

  /// Add a panel by a given name and class name.
  QDockWidget *
  addPanelByName(
    const QString & name,
    const QString & class_lookup_name,
    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
    bool floating = true);

public Q_SLOTS:
  /// Set the message displayed in the status bar.
  void
  setStatus(const QString & message) override;

Q_SIGNALS:
  /// Emitted during file-loading and initialization to indicate progress.
  void
  statusUpdate(const QString & message);

  /// Emitted when the interface enters or leaves full screen mode.
  void
  fullScreenChange(bool hidden);

protected Q_SLOTS:
  /// Remove a the tool whose name is given by remove_tool_menu_action->text().
  void
  onToolbarRemoveTool(QAction * remove_tool_menu_action);

  /// Look up the Tool for this action and call VisualizationManager::setCurrentTool().
  void
  onToolbarActionTriggered(QAction * action);

  /// Add the given tool to this frame's toolbar.
  /**
   * This creates a QAction internally which listens for the Tool's
   * shortcut key.
   * When the action is triggered by the toolbar or by the shortcut key,
   * onToolbarActionTriggered() is called.
   */
  void
  addTool(rviz_common::Tool * tool);

  /// Remove the given tool from the frame's toolbar.
  void
  removeTool(rviz_common::Tool * tool);

  /// Refresh the given tool in this frame's toolbar.
  /**
   * This will update the icon and the text of the corresponding QAction.
   */
  void
  refreshTool(rviz_common::Tool * tool);

  /// Mark the given tool as the current one.
  /**
   * This is purely a visual change in the GUI, it does not call any
   * tool functions.
   */
  void
  indicateToolIsCurrent(rviz_common::Tool * tool);

  /// Delete a panel widget.
  /**
   * The sender() of the signal should be a QAction whose text() is
   * the name of the panel.
   */
  void
  onDeletePanel();

  /// Set full screen mode.
  void
  setFullScreen(bool full_screen);

  /// Exit full screen mode.
  void
  exitFullScreen();

protected:
  /// Setup the menu bar and menus.
  void
  initMenus();

  /// Setup the toolbar and the tools in it.
  void
  initToolbars();

  /// Parent QApplication, set by setApp().
  QApplication * app_;

  /// Actual panel where the main 3D scene is rendered.
  rviz_common::RenderPanel * render_panel_;

  std::string config_dir_;
  std::string persistent_settings_file_;
  std::string display_config_file_;
  std::string default_display_config_file_;
  std::string last_config_dir_;
  std::string last_image_dir_;
  std::string home_dir_;

  QMenu * view_menu_;
  QMenu * delete_view_menu_;

  QToolBar * toolbar_;

  rviz_common::VisualizationManager * manager_;

  QActionGroup * toolbar_actions_;
  std::map<QAction *, rviz_common::Tool *> action_to_tool_map_;
  std::map<rviz_common::Tool *, QAction *> tool_to_action_map_;

  rviz_common::PanelFactory * panel_factory_;

  struct PanelRecord
  {
    rviz_common::Panel * panel;
    rviz_common::PanelDockWidget * dock;
    QString name;
    QString class_id;
    QAction * delete_action;
  };
  QList<PanelRecord> custom_panels_;

  QAction * add_tool_action_;
  QMenu * remove_tool_menu_;

  /// Indicates if the toolbar should be visible outside of fullscreen mode.
  bool toolbar_visible_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};