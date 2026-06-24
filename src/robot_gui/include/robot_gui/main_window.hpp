#pragma once

#include <array>
#include <memory>
#include <vector>

#include <QImage>
#include <QMainWindow>
#include <QStringList>
#include <QVector>

#include "sensor_msgs/msg/image.hpp"
#include "robot_gui/robot_gui_node.hpp"

class QApplication;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;

QT_BEGIN_NAMESPACE
namespace Ui
{
class RobotGUI_MainWindow;
}
QT_END_NAMESPACE

namespace robot_gui
{

class RvizPanel;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QApplication * app, const std::shared_ptr<RobotGuiNode> & node, QWidget * parent = nullptr);
  ~MainWindow() override;

Q_SIGNALS:
  void image_received(QString panel, QImage image);
  void joint_state_received(QStringList names, QVector<double> positions_rad, QVector<double> velocities_rad_s);
  void flags_received(std::vector<uint32_t> flags);
  void ros_log_received(QString message);

private Q_SLOTS:
  void update_image_panel(QString panel, QImage image);
  void update_joint_state_display(QStringList names, QVector<double> positions_rad, QVector<double> velocities_rad_s);
  void update_axis_flags(std::vector<uint32_t> flags);
  void append_ros_log(QString message);

private:
  void setup_defaults();
  void setup_navigation();
  void setup_robot_controls();
  void setup_image_panels();
  void setup_rviz(QApplication * app);
  void set_current_page(int index);
  void set_label_text(const QString & object_name, const QString & text);
  QLabel * label(const QString & object_name) const;
  QLineEdit * line_edit(const QString & object_name) const;
  QPushButton * button(const QString & object_name) const;
  QTextEdit * text_edit(const QString & object_name) const;
  void show_image_placeholder(const QString & panel);
  QImage image_msg_to_qimage(const sensor_msgs::msg::Image & msg) const;
  void set_led(QLabel * widget, bool active, const QString & inactive, const QString & active_color);
  void set_axis_leds(int axis, uint32_t status);
  void update_robot_enable_button();

  std::unique_ptr<Ui::RobotGUI_MainWindow> ui_;
  std::shared_ptr<RobotGuiNode> node_;
  std::unique_ptr<RvizPanel> rviz_panel_;
  std::array<uint32_t, 6> last_flags_{};
  std::array<double, 6> commanded_deg_{};
  bool robot_servo_on_{false};
};

}  // namespace robot_gui
