#ifndef gnss_ros_gui_QNODE_HPP_
#define gnss_ros_gui_QNODE_HPP_


#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

namespace gnss_ros_gui {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
Q_SIGNALS:
private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
};

}  // namespace gnss_ros_gui

#endif /* gnss_ros_gui_QNODE_HPP_ */
