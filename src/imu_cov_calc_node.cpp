#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <csignal>

using namespace std;

// >>>>> Globals
struct sigaction sigAct;
bool stop = false;

int imu_data_count = 0;

// Parameters
int data_count = 1000;

//Data Statistics
vector<double> vec_acc_x;
vector<double> vec_acc_y;
vector<double> vec_acc_z;

vector<double> vec_ang_x;
vector<double> vec_ang_y;
vector<double> vec_ang_z;

vector<double> vec_quat_w;
vector<double> vec_quat_x;
vector<double> vec_quat_y;
vector<double> vec_quat_z;
// <<<<< Globals

// >>>>> Ctrl+C handler
/*! Ctrl+C handler
 */
static void sighandler(int signo)
{
    ROS_INFO_STREAM( "!!!! Ctrl+C pressed by user !!!!" );
    stop = true;
}
// <<<<< Ctrl+C handler

// >>>>> Structs
typedef struct _dataStat
{
    double mean;
    double var;
} DataStat;
// <<<<< Structs

// >>>>> Functions
void load_params(ros::NodeHandle& nh);
void processImuData( const sensor_msgs::Imu::ConstPtr& imu );
DataStat calcStats( vector<double>& data );
// <<<<< Functions

int main(int argc, char** argv)
{
    ros::init( argc, argv, "imu_cov_calc_node" );

    ROS_INFO_STREAM( "******************************************" );
    ROS_INFO_STREAM( "*** Variance estimation for IMU sensor ***" );
    ROS_INFO_STREAM( "******************************************" );

    cout << endl;

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~"); // Private node handler to retrieve parameters

    // >>>>> Ctrl+C handling
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = sighandler;
    sigaction(SIGINT, &sigAct, 0);
    // <<<<< Ctrl+C handling

    load_params( nhPriv );

    // >>>>> Vector initialization
    vec_acc_x.reserve(data_count);
    vec_acc_y.reserve(data_count);
    vec_acc_z.reserve(data_count);

    vec_ang_x.reserve(data_count);
    vec_ang_y.reserve(data_count);
    vec_ang_z.reserve(data_count);

    vec_quat_w.reserve(data_count);
    vec_quat_x.reserve(data_count);
    vec_quat_y.reserve(data_count);
    vec_quat_z.reserve(data_count);
    // <<<<< Vector initialization

    // >>>>> Subscribers
    ros::Subscriber imuSub;
    imuSub = nh.subscribe<sensor_msgs::Imu>("imu_data",1,&processImuData);
    // <<<<< Subscribers

    ros::Rate r(30);

    while(nh.ok())
    {
        if(stop)
        {
            ROS_INFO("... acquisition stopped");
            cout << endl;

            break;
        }

        ros::spinOnce();
        r.sleep();
    }

    // >>>>> Variance calculation
    ROS_INFO_STREAM( "Samples acquired " << vec_acc_x.size() );
    cout << endl;

    ROS_INFO_STREAM( "************************" );
    ROS_INFO_STREAM( "* Variance calculation *" );
    ROS_INFO_STREAM( "************************" );
    cout << endl;

    DataStat stat_acc_x = calcStats(vec_acc_x);
    DataStat stat_acc_y = calcStats(vec_acc_y);
    DataStat stat_acc_z = calcStats(vec_acc_z);

    DataStat stat_ang_x = calcStats(vec_ang_x);
    DataStat stat_ang_y = calcStats(vec_ang_y);
    DataStat stat_ang_z = calcStats(vec_ang_z);

    DataStat stat_quat_w = calcStats(vec_quat_w);
    DataStat stat_quat_x = calcStats(vec_quat_x);
    DataStat stat_quat_y = calcStats(vec_quat_y);
    DataStat stat_quat_z = calcStats(vec_quat_z);

    ROS_INFO_STREAM( "Linear acceleration: ( mean, var )");
    ROS_INFO_STREAM( " * X: ( "<< stat_acc_x.mean << " , " << stat_acc_x.var << " )" );
    ROS_INFO_STREAM( " * Y: ( "<< stat_acc_y.mean << " , " << stat_acc_y.var << " )" );
    ROS_INFO_STREAM( " * Z: ( "<< stat_acc_z.mean << " , " << stat_acc_z.var << " )" );
    cout << endl;

    ROS_INFO_STREAM( "Angular velocity: ( mean, var )");
    ROS_INFO_STREAM( " * X: ( "<< stat_ang_x.mean << " , " << stat_ang_x.var << " )" );
    ROS_INFO_STREAM( " * Y: ( "<< stat_ang_y.mean << " , " << stat_ang_y.var << " )" );
    ROS_INFO_STREAM( " * Z: ( "<< stat_ang_z.mean << " , " << stat_ang_z.var << " )" );
    cout << endl;

    ROS_INFO_STREAM( "Orientation: ( mean, var )");
    ROS_INFO_STREAM( " * X: ( "<< stat_quat_x.mean << " , " << stat_quat_x.var << " )" );
    ROS_INFO_STREAM( " * Y: ( "<< stat_quat_y.mean << " , " << stat_quat_y.var << " )" );
    ROS_INFO_STREAM( " * Z: ( "<< stat_quat_z.mean << " , " << stat_quat_z.var << " )" );
    ROS_INFO_STREAM( " * W: ( "<< stat_quat_w.mean << " , " << stat_quat_w.var << " )" );
    cout << endl;
    // <<<<< Variance calculation

    ROS_INFO_STREAM( "Done.");

    return EXIT_SUCCESS;
}

void load_params(ros::NodeHandle& nh)
{
    ROS_INFO_STREAM( "***** Loading parameters *****");

    if( nh.hasParam( "data_count" ) )
    {
        nh.getParam( "data_count", data_count );
        ROS_INFO_STREAM( "* data_count: " << data_count );
    }
    else
    {
        nh.setParam( "data_count", data_count );
        ROS_INFO_STREAM( "* data_count" << " not present. Default value set: " << data_count );
    }

    ROS_INFO_STREAM( "***** Parameters loaded *****");

    cout << endl;
}

void processImuData( const sensor_msgs::Imu::ConstPtr& imu )
{
    vec_ang_x.push_back( imu->angular_velocity.x );
    vec_ang_y.push_back( imu->angular_velocity.y );
    vec_ang_z.push_back( imu->angular_velocity.z );

    vec_acc_x.push_back( imu->linear_acceleration.x );
    vec_acc_y.push_back( imu->linear_acceleration.y );
    vec_acc_z.push_back( imu->linear_acceleration.z );

    vec_quat_w.push_back( imu->orientation.w );
    vec_quat_x.push_back( imu->orientation.x );
    vec_quat_y.push_back( imu->orientation.y );
    vec_quat_z.push_back( imu->orientation.z );

    imu_data_count++;

    ROS_INFO_STREAM( "Acquired " << imu_data_count << " data samples" );

    if( imu_data_count == data_count)
        stop = true;
}

DataStat calcStats( vector<double>& data )
{
    double sum = 0.0;
    size_t count = data.size();

    //ROS_INFO_STREAM( "Count: " << count );

    double mean = 0.0;

    for( size_t i=0; i<count; i++ )
    {
        sum += data[i];
    }

    mean = sum/count;

    //ROS_INFO_STREAM( "Mean: " << mean );

    double sum_sq = 0.0;

    for( size_t i=0; i<count; i++ )
    {
        sum_sq += (data[i]-mean)*(data[i]-mean);
    }

    double sigma_sq = sum_sq/count;

    //ROS_INFO_STREAM( "Variance: " << sigma_sq );
    DataStat reply;
    reply.mean = mean;
    reply.var = sigma_sq;

    return reply;
}
