#include "robot_gui/main_window.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>

#include <QApplication>
#include <QDir>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QPushButton>
#include <QStackedWidget>
#include <QTextEdit>
#include <QTimer>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/logging.hpp"
#include "robot_gui/rviz_panel.hpp"
#include "ui_robot_gui.h"

namespace robot_gui
{
namespace
{
constexpr int kJointCount = 6;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr uint32_t SERVO_ON = 0x00100000;
constexpr uint32_t RUNNING = 0x08000000;
constexpr uint32_t ORG_SET_OK = 0x02000000;
constexpr uint32_t SOF_LIMIT_P = 0x00000008;
constexpr uint32_t SOF_LIMIT_M = 0x00000010;
constexpr uint32_t ALARM = 0x00200000;
constexpr uint32_t EMG = 0x00010000;
constexpr uint32_t ERROR_ALL = 0x00000001;

QString timestamp()
{
  const auto now = std::time(nullptr);
  char buffer[32]{};
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&now));
  return QString::fromLatin1(buffer);
}
}  // namespace

MainWindow::MainWindow(QApplication * app, const std::shared_ptr<RobotGuiNode> & node, QWidget * parent)
: QMainWindow(parent), ui_(std::make_unique<Ui::RobotGUI_MainWindow>()), node_(node)
{
  const auto share_dir =
    QString::fromStdString(ament_index_cpp::get_package_share_directory("robot_gui"));
  const auto old_dir = QDir::current();
  QDir::setCurrent(share_dir + "/ui");
  ui_->setupUi(this);
  QDir::setCurrent(old_dir.absolutePath());

  qRegisterMetaType<std::vector<uint32_t>>("std::vector<uint32_t>");
  qRegisterMetaType<QStringList>("QStringList");
  qRegisterMetaType<QVector<double>>("QVector<double>");
  qRegisterMetaType<QImage>("QImage");

  connect(this, &MainWindow::image_received, this, &MainWindow::update_image_panel);
  connect(this, &MainWindow::joint_state_received, this, &MainWindow::update_joint_state_display);
  connect(this, &MainWindow::flags_received, this, &MainWindow::update_axis_flags);
  connect(this, &MainWindow::ros_log_received, this, &MainWindow::append_ros_log);

  node_->set_image_callback(
    [this](const std::string & panel, const sensor_msgs::msg::Image::SharedPtr & msg) {
      try {
        Q_EMIT image_received(QString::fromStdString(panel), image_msg_to_qimage(*msg));
      } catch (const std::exception & exc) {
        Q_EMIT ros_log_received(QString("Image conversion failed: %1").arg(exc.what()));
      }
    });
  node_->set_joint_state_callback(
    [this](const std::vector<std::string> & names, const std::vector<double> & pos, const std::vector<double> & vel) {
      QStringList qnames;
      for (const auto & name : names) {
        qnames.push_back(QString::fromStdString(name));
      }
      Q_EMIT joint_state_received(qnames, QVector<double>(pos.begin(), pos.end()), QVector<double>(vel.begin(), vel.end()));
    });
  node_->set_flag_callback([this](const std::vector<uint32_t> & flags) {Q_EMIT flags_received(flags);});
  node_->set_log_callback([this](const std::string & message) {Q_EMIT ros_log_received(QString::fromStdString(message));});

  setup_defaults();
  setup_navigation();
  setup_robot_controls();
  setup_image_panels();
  setup_rviz(app);
}

MainWindow::~MainWindow() = default;

void MainWindow::setup_defaults()
{
  set_label_text("ConnectStatus_label", "Disconnected");
  set_label_text("PingStatus_value", "-- ms");
  set_label_text("PlannerStatus_value", "RL|Cartesian");
  for (int axis = 1; axis <= kJointCount; ++axis) {
    set_label_text(QString("lblAxis%1ActualPosUnit").arg(axis), "deg");
    set_label_text(QString("lblAxis%1ActualVelUnit").arg(axis), "deg/s");
    if (auto * pos = line_edit(QString("txtAxis%1ActualPos").arg(axis))) {
      pos->setReadOnly(true);
      pos->setText("0.000");
    }
    if (auto * vel = line_edit(QString("txtAxis%1ActualVel").arg(axis))) {
      vel->setReadOnly(true);
      vel->setText("0.000");
    }
    set_axis_leds(axis, 0);
  }
  update_robot_enable_button();
}

void MainWindow::setup_navigation()
{
  const std::vector<std::pair<QString, QWidget *>> page_widgets = {
    {"btnHome", ui_->pageHome}, {"btnMain", ui_->pageMain}, {"btnRobot", ui_->pageRobot},
    {"btnVision", ui_->pageVision}, {"btnSetting", ui_->pageSetting}, {"btnLog", ui_->pageLog}};
  std::vector<std::pair<QString, int>> pages;
  for (const auto & [name, widget] : page_widgets) {
    if (ui_->stackedWidget_MainPages != nullptr && widget != nullptr) {
      pages.push_back({name, ui_->stackedWidget_MainPages->indexOf(widget)});
    }
  }
  for (const auto & [name, index] : pages) {
    if (auto * btn = button(name)) {
      btn->setCheckable(true);
      connect(btn, &QPushButton::clicked, this, [this, index]() {set_current_page(index);});
    }
  }
  if (ui_->stackedWidget_MainPages != nullptr && node_->initial_page() >= 0) {
    set_current_page(node_->initial_page());
  }
}

void MainWindow::setup_robot_controls()
{
  if (auto * btn = button("btnRobotEnable")) {
    connect(btn, &QPushButton::clicked, this, [this]() {node_->set_servo_all(!robot_servo_on_);});
  }
  if (auto * btn = button("btnRobotDisable")) {
    connect(btn, &QPushButton::clicked, this, [this]() {node_->set_servo_all(false);});
  }
}

void MainWindow::setup_image_panels()
{
  show_image_placeholder("raw");
  show_image_placeholder("detection");
  show_image_placeholder("yolo");
}

void MainWindow::setup_rviz(QApplication *)
{
  if (!node_->embed_rviz()) {
    if (ui_->labelRvizPlaceholder) {
      ui_->labelRvizPlaceholder->setText("RViz disabled");
    }
    return;
  }
  rviz_panel_ = std::make_unique<RvizPanel>(ui_->embeddedRvizWidget);
  QTimer::singleShot(100, this, [this]() {
    if (!rviz_panel_->initialize("base_link")) {
      append_ros_log(rviz_panel_->last_error());
    } else if (ui_->labelRvizPlaceholder) {
      ui_->labelRvizPlaceholder->hide();
    }
  });
}

void MainWindow::set_current_page(int index)
{
  if (ui_->stackedWidget_MainPages && index >= 0 && index < ui_->stackedWidget_MainPages->count()) {
    ui_->stackedWidget_MainPages->setCurrentIndex(index);
  }
}

void MainWindow::show_image_placeholder(const QString & panel)
{
  const QString object_name =
    panel == "raw" ? "rawImageView" : panel == "detection" ? "detectionImageView" : "yoloPreviewWidget";
  const QString title =
    panel == "raw" ? "Raw Image" : panel == "detection" ? "Detection Image" : "YOLO Image";
  const QString topic =
    panel == "raw" ? QString::fromStdString(node_->raw_image_topic()) :
    panel == "detection" ? QString::fromStdString(node_->detection_image_topic()) :
    QString::fromStdString(node_->yolo_image_topic());
  if (auto * image_label = label(object_name)) {
    image_label->setAlignment(Qt::AlignCenter);
    image_label->setWordWrap(true);
    image_label->clear();
    image_label->setText(topic.isEmpty() ? title + "\nNo topic configured" : title + "\n" + topic + "\nWaiting for image...");
  }
}

QImage MainWindow::image_msg_to_qimage(const sensor_msgs::msg::Image & msg) const
{
  const auto encoding = QString::fromStdString(msg.encoding).toLower();
  const auto * data = msg.data.data();
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  const int step = static_cast<int>(msg.step);
  if (encoding == "rgb8") {
    return QImage(data, width, height, step, QImage::Format_RGB888).copy();
  }
  if (encoding == "bgr8") {
    return QImage(data, width, height, step, QImage::Format_RGB888).rgbSwapped().copy();
  }
  if (encoding == "mono8") {
    return QImage(data, width, height, step, QImage::Format_Grayscale8).copy();
  }
  if (encoding == "rgba8") {
    return QImage(data, width, height, step, QImage::Format_RGBA8888).copy();
  }
  if (encoding == "bgra8") {
    return QImage(data, width, height, step, QImage::Format_ARGB32).copy();
  }
  throw std::runtime_error("unsupported image encoding: " + msg.encoding);
}

void MainWindow::update_image_panel(QString panel, QImage image)
{
  const QString object_name =
    panel == "raw" ? "rawImageView" : panel == "detection" ? "detectionImageView" : "yoloPreviewWidget";
  if (auto * image_label = label(object_name)) {
    const auto pixmap = QPixmap::fromImage(image);
    image_label->setText("");
    image_label->setPixmap(pixmap.scaled(image_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
}

void MainWindow::update_joint_state_display(QStringList names, QVector<double> positions_rad, QVector<double> velocities_rad_s)
{
  for (int axis = 1; axis <= kJointCount; ++axis) {
    const auto & joint_names = node_->joint_names();
    const QString joint_name = axis - 1 < static_cast<int>(joint_names.size()) ?
      QString::fromStdString(joint_names[axis - 1]) : QString("joint_%1").arg(axis);
    const int index = names.indexOf(joint_name);
    if (index < 0 || index >= positions_rad.size()) {
      continue;
    }
    const double pos_deg = positions_rad[index] * kRadToDeg;
    commanded_deg_[axis - 1] = pos_deg;
    if (auto * pos = line_edit(QString("txtAxis%1ActualPos").arg(axis))) {
      pos->setText(QString::number(pos_deg, 'f', 3));
    }
    if (auto * vel = line_edit(QString("txtAxis%1ActualVel").arg(axis))) {
      vel->setText(QString::number(index < velocities_rad_s.size() ? velocities_rad_s[index] * kRadToDeg : 0.0, 'f', 3));
    }
  }
}

void MainWindow::update_axis_flags(std::vector<uint32_t> flags)
{
  for (int axis = 1; axis <= kJointCount; ++axis) {
    const uint32_t status = axis - 1 < static_cast<int>(flags.size()) ? flags[axis - 1] : 0;
    last_flags_[axis - 1] = status;
    set_axis_leds(axis, status);
  }
  robot_servo_on_ = std::all_of(last_flags_.begin(), last_flags_.end(), [](uint32_t status) {
    return (status & SERVO_ON) != 0;
  });
  update_robot_enable_button();
}

void MainWindow::set_axis_leds(int axis, uint32_t status)
{
  set_led(label(QString("ledAxis%1ServoOn").arg(axis)), status & SERVO_ON, "#808080", "#00cc66");
  set_led(label(QString("ledAxis%1Running").arg(axis)), status & RUNNING, "#808080", "#00cc66");
  set_led(label(QString("ledAxis%1OrgOK").arg(axis)), status & ORG_SET_OK, "#808080", "#00cc66");
  set_led(label(QString("ledAxis%1LimitPositive").arg(axis)), status & SOF_LIMIT_P, "#808080", "#00cc66");
  set_led(label(QString("ledAxis%1LimitNegative").arg(axis)), status & SOF_LIMIT_M, "#808080", "#00cc66");
  set_led(label(QString("ledAxis%1Alarm").arg(axis)), status & ALARM, "#95c7ea", "#ff3333");
  set_led(label(QString("ledAxis%1EMG").arg(axis)), status & EMG, "#95c7ea", "#ff3333");
  set_led(label(QString("ledAxis%1ErrorAll").arg(axis)), status & ERROR_ALL, "#95c7ea", "#ff3333");
}

void MainWindow::set_led(QLabel * widget, bool active, const QString & inactive, const QString & active_color)
{
  if (widget == nullptr) {
    return;
  }
  widget->setText("");
  widget->setStyleSheet(QString("background:%1; border-radius:7px; border:1px solid #3a3a3a;")
    .arg(active ? active_color : inactive));
}

void MainWindow::update_robot_enable_button()
{
  set_led(label("RobotEnableStatus_led"), robot_servo_on_, "#d50000", "#00c853");
  if (auto * btn = button("btnRobotEnable")) {
    btn->setText(robot_servo_on_ ? "Disable" : "Enable");
  }
}

void MainWindow::set_label_text(const QString & object_name, const QString & text)
{
  if (auto * widget = label(object_name)) {
    widget->setText(text);
  }
}

QLabel * MainWindow::label(const QString & object_name) const {return findChild<QLabel *>(object_name);}
QLineEdit * MainWindow::line_edit(const QString & object_name) const {return findChild<QLineEdit *>(object_name);}
QPushButton * MainWindow::button(const QString & object_name) const {return findChild<QPushButton *>(object_name);}
QTextEdit * MainWindow::text_edit(const QString & object_name) const {return findChild<QTextEdit *>(object_name);}

void MainWindow::append_ros_log(QString message)
{
  const QString line = QString("[%1] %2").arg(timestamp(), message);
  if (auto * log = text_edit("txtROS2Log")) {
    log->append(line);
  }
  set_label_text("txtMainLog", line);
}

}  // namespace robot_gui
