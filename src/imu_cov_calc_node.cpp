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
int data_count=1000;

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

// >>>>> Functions
void load_params(ros::NodeHandle& nh);
void processImuData( const sensor_msgs::Imu::ConstPtr& imu );
double calcVariance( vector<double>& data );
// <<<<< Functions

// >>>>> Ctrl+C handler
/*! Ctrl+C handler
 */
static void sighandler(int signo)
{
    ROS_INFO_STREAM( "!!!! Ctrl+C pressed by user !!!!" );
    stop = true;
}
// <<<<< Ctrl+C handler

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

    double var_acc_x = calcVariance(vec_acc_x);
    double var_acc_y = calcVariance(vec_acc_y);
    double var_acc_z = calcVariance(vec_acc_z);

    double var_ang_x = calcVariance(vec_ang_x);
    double var_ang_y = calcVariance(vec_ang_y);
    double var_ang_z = calcVariance(vec_ang_z);

    double var_quat_w = calcVariance(vec_quat_w);
    double var_quat_x = calcVariance(vec_quat_x);
    double var_quat_y = calcVariance(vec_quat_y);
    double var_quat_z = calcVariance(vec_quat_z);

    ROS_INFO_STREAM( "Linear acceleration:");
    ROS_INFO_STREAM( " * X: "<< var_acc_x);
    ROS_INFO_STREAM( " * Y: "<< var_acc_y);
    ROS_INFO_STREAM( " * Z: "<< var_acc_z);
    cout << endl;

    ROS_INFO_STREAM( "Angular velocity:");
    ROS_INFO_STREAM( " * X: "<< var_ang_x);
    ROS_INFO_STREAM( " * Y: "<< var_ang_y);
    ROS_INFO_STREAM( " * Z: "<< var_ang_z);
    cout << endl;

    ROS_INFO_STREAM( "Orientation:");
    ROS_INFO_STREAM( " * W: "<< var_quat_w);
    ROS_INFO_STREAM( " * X: "<< var_quat_x);
    ROS_INFO_STREAM( " * Y: "<< var_quat_y);
    ROS_INFO_STREAM( " * Z: "<< var_quat_z);
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

double calcVariance( vector<double>& data )
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

    return sigma_sq;
}
