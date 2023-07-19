#include <opencv/core>

class CloudViewer
{

  public:
    CloudViewer() = delete;

    static void ViewerByOpencv();
    static void ViewerByPcl();
    static void ViewerByRos();

    ~CloudViewer();
};