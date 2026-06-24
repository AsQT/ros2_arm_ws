#include "robot_gui/rviz_panel.hpp"

#include <QVBoxLayout>
#include <QWidget>

#include "rviz_common/display.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"

namespace robot_gui
{

RvizPanel::RvizPanel(QWidget * parent_widget)
: QObject(parent_widget), parent_widget_(parent_widget)
{
}

RvizPanel::~RvizPanel() = default;

bool RvizPanel::initialize(const std::string & fixed_frame)
{
  if (parent_widget_ == nullptr) {
    last_error_ = "embeddedRvizWidget not found";
    return false;
  }

  auto * layout = parent_widget_->layout();
  if (layout == nullptr) {
    layout = new QVBoxLayout(parent_widget_);
  }
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  parent_widget_->setAttribute(Qt::WA_NativeWindow, true);
  parent_widget_->setAttribute(Qt::WA_DontCreateNativeAncestors, false);
  render_panel_ = new rviz_common::RenderPanel(parent_widget_);
  render_panel_->setAttribute(Qt::WA_NativeWindow, true);
  render_panel_->setAttribute(Qt::WA_DontCreateNativeAncestors, false);
  render_panel_->setMinimumSize(parent_widget_->size());
  layout->addWidget(render_panel_);
  render_panel_->show();
  render_panel_->getRenderWindow()->initialize();
  rviz_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
    "robot_gui_rviz");
  manager_ = std::make_unique<rviz_common::VisualizationManager>(
    render_panel_, rviz_node_, nullptr, rviz_node_->get_raw_node()->get_clock());
  render_panel_->initialize(manager_.get());
  manager_->initialize();
  manager_->setFixedFrame(QString::fromStdString(fixed_frame));
  manager_->startUpdate();

  auto * grid = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  if (grid != nullptr) {
    grid->subProp("Plane Cell Count")->setValue(20);
  }
  manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel", true);

  return true;
}

QWidget * RvizPanel::widget() const
{
  return render_panel_;
}

QString RvizPanel::last_error() const
{
  return last_error_;
}

}  // namespace robot_gui
