#pragma once

#include <memory>
#include <string>

#include <QObject>

class QWidget;

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;
namespace ros_integration
{
class RosNodeAbstraction;
}
}  // namespace rviz_common

namespace robot_gui
{

class RvizPanel : public QObject
{
  Q_OBJECT

public:
  explicit RvizPanel(QWidget * parent_widget);
  ~RvizPanel() override;

  bool initialize(const std::string & fixed_frame = "base_link");
  QWidget * widget() const;
  QString last_error() const;

private:
  QWidget * parent_widget_{nullptr};
  rviz_common::RenderPanel * render_panel_{nullptr};
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_node_;
  std::unique_ptr<rviz_common::VisualizationManager> manager_;
  QString last_error_;
};

}  // namespace robot_gui
