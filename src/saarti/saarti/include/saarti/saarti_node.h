#ifndef SAARTI_NODE_H
#define SAARTI_NODE_H

// ros
#include "ros/ros.h"
#include "ros/time.h"
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
//#include "std_msgs/ColorRGBA.h"

// visualization
#include "visualization_msgs/MarkerArray.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/PlotData.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"

// wrapper libs
#include "containers.h"
#include "saarti/rtisqp_wrapper.h"

// custom messages
#include <common/Path.h>
#include <common/Obstacles.h>
#include <common/Trajectory.h>
#include <common/State.h>
#include <common/SaartiStatus.h>

// timing and threads
#include <chrono>
#include <thread>
#include <future>

// misc
#include <sstream>
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant" // supress warning at ros prints

// namespaces
using std::vector;
using std::cout;
using std::endl;

namespace saarti_node{
class SAARTI{
public:
    // constructor
    SAARTI(ros::NodeHandle nh);

private:
    // general
    float dt_;
    float dt_integrator_ = 0.1; // todo get from param
    bool planner_activated_;

    // modes
    int ref_mode_;
    int sampling_augmentation_;
    int traction_adaptive_;

    // params
    float mu_nominal_; // only used for nonadaptive case
    float vxref_cc_; // only used for velocity keeping (refmode 1)
    float dref_cc_; // only used for velocity keeping (refmode 1)
    float Ff_util_;
    float Fr_util_;
    int Nd_rollout_;
    int Nvx_rollout_;
    float vxub_rollout_;
    vector<float> Wx_;
    vector<float> WNx_;
    vector<float> Wu_;
    float Wslack_;
    vector<float> K_;

    // internal variables
    ros::NodeHandle nh_;
    ros::Subscriber pathlocal_sub_;
    ros::Subscriber obstacles_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber ctrlmode_sub_;
    ros::Subscriber vxref_sub_;
    ros::Publisher trajstar_pub_;
    ros::Publisher trajhat_pub_;
    ros::Publisher trajset_vis_pub_;
    ros::Publisher trajhat_vis_pub_;
    ros::Publisher trajstar_vis_pub_;
    ros::Publisher trajstar_polarr_vis_pub_;
    ros::Publisher posconstr_vis_pub_;
    ros::Publisher vectordebug_pub_;
    ros::Publisher status_pub_;
    containers::statestruct state_;
    int ctrlmode_;
    containers::pathstruct pathlocal_;
    vector<containers::trajstruct> trajset_;
    containers::obstastruct obst_;
    containers::refstruct refs_;
    containers::staticparamstruct sp_;
    RtisqpWrapper rtisqp_wrapper_;

    // functions

    // move
    common::Trajectory traj2msg(containers::trajstruct traj);
    nav_msgs::Path traj2navpath(containers::trajstruct traj);
    jsk_recognition_msgs::PolygonArray traj2polarr(containers::trajstruct traj, containers::staticparamstruct sp);
    void trajset2ma();
    visualization_msgs::Marker trajset2cubelist();
    jsk_recognition_msgs::PolygonArray stateconstr2polarr(containers::posconstrstruct pc);

    // keep
    containers::refstruct setRefs(int ref_mode_, int traction_adaptive_, float mu_nominal_, float vxref_cc, float dref_cc, containers::staticparamstruct sp_, containers::pathstruct pathlocal_);
    std::tuple<int, int> trajset_eval_cost();
    void state_callback(const common::State::ConstPtr& msg);
    void ctrlmode_callback(const std_msgs::Int16::ConstPtr& msg);
    void vxref_callback(const std_msgs::Float32::ConstPtr& msg);
    void pathlocal_callback(const common::Path::ConstPtr& msg);
    void obstacles_callback(const common::Obstacles::ConstPtr& msg);
    void get_rosparams();
    void run_optimization();
};
} // end namespace saarti_node
#endif // SAARTI_NODE_H
