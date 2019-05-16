// For efficiency in compiling, this is the only file that uses the
// Eigen lirbary and the 'vector' type we made from it.

#include "minimu9-ahrs/vector.h"
#include "minimu9-ahrs/version.h"
#include "minimu9-ahrs/prog_options.h"
#include "minimu9-ahrs/minimu9.h"
#include "minimu9-ahrs/exceptions.h"
#include "minimu9-ahrs/pacer.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <system_error>
#include <chrono>

// Include ROS files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "tf/transform_broadcaster.h"


// TODO: print warning if accelerometer magnitude is not close to 1 when starting up

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
float field_width = 6;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector)
{
  return os << FLOAT_FORMAT << vector(0) << ' '
            << FLOAT_FORMAT << vector(1) << ' '
            << FLOAT_FORMAT << vector(2);
}

std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
  return os << (vector)matrix.row(0) << ' '
            << (vector)matrix.row(1) << ' '
            << (vector)matrix.row(2);
}

std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
  return os << FLOAT_FORMAT << quat.w() << ' '
            << FLOAT_FORMAT << quat.x() << ' '
            << FLOAT_FORMAT << quat.y() << ' '
            << FLOAT_FORMAT << quat.z();
}

typedef void rotation_output_function(quaternion & rotation);

void output_quaternion(quaternion & rotation)
{
  std::cout << rotation;
}

void output_matrix(quaternion & rotation)
{
  std::cout << rotation.toRotationMatrix();
}

void output_euler(quaternion & rotation)
{
  std::cout << (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0)
                        * (180 / M_PI));
}

void stream_raw_values(imu & imu)
{
  ROS_INFO("Entering RAW mode");	

  //Declare node handler
  ros::NodeHandle n;

  //Publisher Initialization of Magnetometer, Gyroscope and Accelerometer
  ros::Publisher mag_pub = n.advertise<geometry_msgs::Vector3>("imu/raw/magnetometer", 1000);
  ros::Publisher gyro_pub = n.advertise<geometry_msgs::Vector3>("imu/raw/gyroscope", 1000);
  ros::Publisher accel_pub = n.advertise<geometry_msgs::Vector3>("imu/raw/accelerometer", 1000);

  ros::Rate loop_rate(20);   

  imu.enable();
	
  //data handlers
  geometry_msgs::Vector3 mag;
  geometry_msgs::Vector3 gyro;
  geometry_msgs::Vector3 accel;
  
  
  while(ros::ok())
  {
    imu.read_raw();
	//print raw data
    /*printf("%7d %7d %7d  %7d %7d %7d  %7d %7d %7d\n",
           imu.m[0], imu.m[1], imu.m[2],
           imu.a[0], imu.a[1], imu.a[2],
           imu.g[0], imu.g[1], imu.g[2]
      );*/
    //get msg data
	mag.x   = imu.m[0]; mag.y   = imu.m[1]; mag.z   = imu.m[2];
	gyro.x  = imu.g[0]; gyro.y  = imu.g[1]; gyro.z  = imu.g[2];
	accel.x = imu.a[0]; accel.y = imu.a[1]; accel.z = imu.a[2];
	//mag    =  geometry_msgs::Vector3(imu.m[0],imu.m[1],imu.m[2]);
    //gyro   =  geometry_msgs::Vector3(imu.g[0],imu.g[1],imu.g[2]);
    //accel  =  geometry_msgs::Vector3(imu.a[0],imu.a[1],imu.a[2]);
	//publish data
	mag_pub.publish(mag);
	gyro_pub.publish(gyro);
	accel_pub.publish(accel);
	//spin & loop
	ros::spinOnce();
	loop_rate.sleep();
    //usleep(20*1000);
  }
}

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix rotation_from_compass(const vector & acceleration, const vector & magnetic_field)
{
  vector down = -acceleration;     // usually true
  vector east = down.cross(magnetic_field); // actually it's magnetic east
  vector north = east.cross(down);

  east.normalize();
  north.normalize();
  down.normalize();

  matrix r;
  r.row(0) = north;
  r.row(1) = east;
  r.row(2) = down;
  return r;
}

typedef void fuse_function(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field);

void fuse_compass_only(quaternion & rotation, float dt, const vector& angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  // Implicit conversion of rotation matrix to quaternion.
  rotation = rotation_from_compass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion & rotation, const vector & w, float dt)
{
  // Multiply by first order approximation of the
  // quaternion representing this rotation.
  rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
  rotation.normalize();
}

void fuse_gyro_only(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  rotate(rotation, angular_velocity, dt);
}

void fuse_default(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  vector correction = vector(0, 0, 0);

  if (fabs(acceleration.norm() - 1) <= 0.3)
  {
    // The magnetidude of acceleration is close to 1 g, so
    // it might be pointing up and we can do drift correction.

    const float correction_strength = 1;

    matrix rotation_compass = rotation_from_compass(acceleration, magnetic_field);
    matrix rotation_matrix = rotation.toRotationMatrix();

    correction = (
      rotation_compass.row(0).cross(rotation_matrix.row(0)) +
      rotation_compass.row(1).cross(rotation_matrix.row(1)) +
      rotation_compass.row(2).cross(rotation_matrix.row(2))
      ) * correction_strength;

  }

  rotate(rotation, angular_velocity + correction, dt);
}

void ahrs(imu & imu, fuse_function * fuse, rotation_output_function * output)
{

  ROS_INFO("Entering NORMAL mode"); 

  //Declare node handler
  ros::NodeHandle n;

  //Declare ROS Publisher
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/imu",1000);

  //Set ROS Rate (20 Hz)
  ros::Rate loop_rate(20);

  //data handlers
  sensor_msgs::Imu Imu;

  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();

  // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(20000000);

  //Declare static tf
  tf::TransformBroadcaster br;
  tf::Transform transform;

  auto start = std::chrono::steady_clock::now();
  while(ros::ok())
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);
    ///DEBUG AREA
    //output_euler(rotation);
    //std::cout << magnetic_field << std::endl;
    //std::cout << std::endl;
    
    //Load time stamp
    //Get NOW
    ros::Time now = ros::Time::now();
    double secs = now.toSec();
    Imu.header.stamp.sec  = secs;
    double nsecs = (secs-(double)(Imu.header.stamp.sec))*1e9;
    Imu.header.stamp.nsec = nsecs;

    //Load frame id
    Imu.header.frame_id = "map";

    //Load rotation as a quaternion
    Imu.orientation.w = rotation.w();
    Imu.orientation.z = rotation.z();
    Imu.orientation.y = rotation.y();
    Imu.orientation.x = rotation.x();

    //Load acceleration
    Imu.linear_acceleration.x = acceleration.x();
    Imu.linear_acceleration.y = acceleration.y();
    Imu.linear_acceleration.z = acceleration.z();

    //Load angular velocity
    Imu.angular_velocity.x = angular_velocity.x();
    Imu.angular_velocity.y = angular_velocity.y();
    Imu.angular_velocity.z = angular_velocity.z();
    
    //tf transform broadcaster
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion( rotation.x(), rotation.y(), rotation.z(), rotation.w() ) );
    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "imu") );

    //Publish msg
    imu_pub.publish(Imu);
    //Ros Spin
    ros::spinOnce();

    loop_pacer.pace();
  }
}

int main_with_exceptions(int argc, char **argv)
{
  prog_options options = get_prog_options(argc, argv);

  if(options.show_help)
  {
    print_command_line_options_desc();
    std::cout << "For more information, run: man minimu9-ahrs" << std::endl;
    return 0;
  }

  if (options.show_version)
  {
    std::cout << VERSION << std::endl;
    return 0;
  }

  // Decide what sensors we want to use.
  sensor_set set;
  set.mag = set.acc = set.gyro = true;

  minimu9::comm_config config = minimu9::auto_detect(options.i2c_bus_name);

  sensor_set missing = set - minimu9::config_sensor_set(config);
  if (missing)
  {
    if (missing.mag)
    {
      std::cerr << "Error: No magnetometer found." << std::endl;
    }
    if (missing.acc)
    {
      std::cerr << "Error: No accelerometer found." << std::endl;
    }
    if (missing.gyro)
    {
      std::cerr << "Error: No gyro found." << std::endl;
    }
    std::cerr << "Error: Needed sensors are missing." << std::endl;
    return 1;
  }

  config = minimu9::disable_redundant_sensors(config, set);

  minimu9::handle imu;
  imu.open(config);

  rotation_output_function * output;

  // Figure out the output mode.
  if (options.output_mode == "matrix")
  {
    output = &output_matrix;
  }
  else if (options.output_mode == "quaternion")
  {
    output = &output_quaternion;
  }
  else if (options.output_mode == "euler")
  {
    field_width += 2;  // See comment above for field_width.
    output = &output_euler;
  }
  else
  {
    std::cerr << "Unknown output mode '" << options.output_mode << "'" << std::endl;
    return 1;
  }

  // Figure out the basic operating mode and start running.
  if (options.mode == "raw")
  {
    stream_raw_values(imu);
  }
  else if (options.mode == "gyro-only")
  {
    ahrs(imu, &fuse_gyro_only, output);
  }
  else if (options.mode == "compass-only")
  {
    ahrs(imu, &fuse_compass_only, output);
  }
  else if (options.mode == "normal")
  {
    ahrs(imu, &fuse_default, output);
  }
  else
  {
    std::cerr << "Unknown mode '" << options.mode << "'" << std::endl;
    return 1;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  try
  {
	//Initialize ROS node
	ROS_INFO("Initializing node");
	ros::init(argc, argv, "imu");
    main_with_exceptions(argc, argv);
  }
  catch(const std::system_error & error)
  {
    std::string what = error.what();
    const std::error_code & code = error.code();
    std::cerr << "Error: " << what << " (" << code << ")" << std::endl;
    return 2;
  }
  catch(const std::exception & error)
  {
    std::cerr << "Error: " << error.what() << std::endl;
    return 9;
  }
}
