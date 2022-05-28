#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/touch_sensor.h>
#include <webots/compass.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
#define WHEEL_RADIUS 0.05
#define AXLE_LENGTH 0.3
#define RANGE (1024 / 2)
#define TIME_STEP 64
#define SPEED 3

enum STATE_TYPE
{
  INIT_SEARCH,
  SEARCH,
  TARGET, // To go near target and avoiding collision.
  SUCCESS, 
  NONE, 
  WEIGHT_MEASURE
};

double get_bearing_in_degrees(WbDeviceTag compass)
{
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

double get_turning_in_degrees(WbDeviceTag compass, double prevDir)
{
  double currentDir = get_bearing_in_degrees(compass);
  double result = 0;

  if (prevDir < currentDir && ((currentDir - prevDir) > 300))
    result = prevDir + (360 - currentDir);
  else if (prevDir > currentDir && ((prevDir - currentDir) > 300))
    result = currentDir + (360 - prevDir);
  else if (prevDir < currentDir)
    result = currentDir - prevDir;
  else if (prevDir > currentDir)
    result = prevDir - currentDir;

  return result;
}

bool isRedVisible(WbDeviceTag camera, int height, int width, float searchIntensity)
{
  if (wb_camera_recognition_get_number_of_objects(camera) == 0)
    return false;

  const unsigned char *image;

  if (wb_camera_recognition_has_segmentation(camera))
    image = wb_camera_recognition_get_segmentation_image(camera);
  else
    image = wb_camera_get_image(camera);

  int red, green, blue;
  int i, j;
  if (image != NULL)
  {
    red = 0;
    green = 0;
    blue = 0;

    for (i = width / 3; i < 2 * width / 3; i++)
    {
      for (j = height / 2; j < 3 * height / 4; j++)
      {
        red += wb_camera_image_get_red(image, width, i, j);
        green += wb_camera_image_get_green(image, width, i, j);
        blue += wb_camera_image_get_blue(image, width, i, j);
      }
    }

    if ((red > searchIntensity * green) && (red > searchIntensity * blue))
    {
      return true;
    }
  }

  return false;
}

static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor)
{
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;        // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;        // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH; // delta orientation
  //  printf("estimated distance covered by left wheel: %g m.\n", dl);
  //  printf("estimated distance covered by right wheel: %g m.\n", dr);
  //  printf("estimated change of orientation: %g rad.\n", da);
}


int main(int argc, char *argv[])
{
  /* define variables */
  WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i;
  int h = 0;
  int z = 0;
  int b = 0;
  double p = 2;
  double m = 0;
  bool avoid_obstacle_counter = 0;
  double d , f;
  /* initialize Webots */
  WbDeviceTag camera, compass,
      left_dist_sensor, right_dist_sensor, forward_dist_sensor;
  int width, height;

  enum STATE_TYPE STATE;

  //
  double totalRot, prevDir;
  // float searchIntensity;
  float searchHighIntensity = 1.5f; // safest high intensity value - 1.5f
  float searchLowIntensity = 3.0f;

  float left_ds_dist, right_ds_dist, forward_ds_dist;
  bool left_obstracle, right_obstracle, forward_obstracle;
  //int left_speed = SPEED, right_speed = SPEED;
  wb_robot_init();

  /* get and enable the camera */

  camera = wb_robot_get_device("center_camera");
  wb_camera_recognition_enable(camera, TIME_STEP);
  compass = wb_robot_get_device("cpass");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  wb_camera_recognition_enable_segmentation(camera);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  wb_compass_enable(compass, TIME_STEP);

  /* Set up GPS and enable it */
  WbDeviceTag gps = wb_robot_get_device("global");
  wb_gps_enable(gps, TIME_STEP);

  /*get and enable the wight sensor */
  WbDeviceTag weight = wb_robot_get_device("weight_sensor");
  wb_touch_sensor_enable(weight, TIME_STEP);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("wheel1");
  right_motor = wb_robot_get_device("wheel2");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");

  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  left_dist_sensor = wb_robot_get_device("ds_left");
  right_dist_sensor = wb_robot_get_device("ds_right");
  forward_dist_sensor = wb_robot_get_device("ds_forward");

  wb_distance_sensor_enable(left_dist_sensor, TIME_STEP);
  wb_distance_sensor_enable(right_dist_sensor, TIME_STEP);
  wb_distance_sensor_enable(forward_dist_sensor, TIME_STEP);

  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++)
  {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  STATE = NONE;

  /* main loop */

  while (wb_robot_step(TIME_STEP) != -1)
  {
    // printf("Value of h: %d.\n", h);
    compute_odometry(left_position_sensor, right_position_sensor);
    const double *gps_values = wb_gps_get_values(gps);
    // printf("Using the GPS device: %.3f %.3f\n", gps_values[0], gps_values[2]);
    /* get sensors values */
    if (h < 1)
    {

      //////// Christian Start  ////////////
      const double *gps_values = wb_gps_get_values(gps);
      double l = wb_position_sensor_get_value(left_position_sensor);
      double r = wb_position_sensor_get_value(right_position_sensor);
      double dl = l * WHEEL_RADIUS;        // distance covered by left wheel in meter
      double dr = r * WHEEL_RADIUS;        // distance covered by right wheel in meter
      double da = (dr - dl) / AXLE_LENGTH; // delta orientation
      double sa = da * (-180 / PI);
      double gps0 = gps_values[0];
      double gps2 = gps_values[2];
      double k = (pow(gps0, p)) + (pow(gps2, p));
      double gh = sqrt(k);
      // printf("orientation of robot in degrees: %g degrees.\n", sa);
      //  printf("distance from centre: %g m.\n", gh);
      double al = (atan((gps2) / (gps0)));
      double aal = (PI / 2) - al;
      double a = al * (180 / PI);
      double aa = aal * (180 / PI);

      double gkd = 180 + aa;
      double gd = 360 - gkd;
      double m = +sa - gd - 180;

      double left_speed = 1;
      double right_speed = 1;

      // Drive in Square
      int Square = 0;
      int Eight = 0;

      if (gps2 > 1.0 && Square == 0)
      {
        left_speed = 0.3;
        right_speed = -0.3;
      }
      if (sa > 90 && Square == 0)
      {
        Square = 1;
      }
      if (Square == 1)
      {
        left_speed = 1.0;
        right_speed = 1.0;
      }
      if (gps0 < -1.0 && Square == 1)
      {
        left_speed = 0.3;
        right_speed = -0.3;
      }
      if (sa > 180.0 && Square == 1)
      {
        Square = 2;
      }
      if (Square == 2)
      {
        left_speed = 1.0;
        right_speed = 1.0;
      }
      if (gps2 < 0 && Square == 2)
      {
        left_speed = 0.3;
        right_speed = -0.3;
      }

      if (sa > 270 && Square == 2)
      {
        Square = 3;
      }
      if (Square == 3)
      {
        left_speed = 1.0;
        right_speed = 1.0;
      }
      if (gps0 > -0.1 && Square == 3)
      {
        left_speed = 0.3;
        right_speed = -0.3;
      }
      if (sa > 358 && Square == 3)
      {
        Square = 4;
      }
      if (Square == 4)
      {
        left_speed = 0;
        right_speed = 0;
        Square = 5; //End Square Protocal
        Eight = 1;  //Initiate Figure of Eight
      }

      //Drive in figure of eight
      if (Eight == 1)
      {
        left_speed = 1;
        right_speed = 0.2;
      }
      if (Eight == 1 && dl > 7.6)
      {
        Eight = 2;
      }
      if (Eight == 2)
      {
        left_speed = 0.2;
        right_speed = 1;
      }
      if (Eight == 2 && dl > 8.55)
      {
        left_speed = 1;
        right_speed = 1;
        Eight = 3; //End Eight Protocal
      }

      // printf("dl: %g m.\n", dl);
      //  printf("dr: %g m.\n", dr);
      //  printf("Square: %d m.\n", Square);
      //  printf("Eight: %d m.\n", Eight);

      ////////// Christian End  ////////

      if (isRedVisible(camera, height, width, searchLowIntensity) && STATE == NONE)
      {
        STATE = TARGET;
        //left_speed = 0;
        //right_speed = 0;
        printf("found target\n");
        // wb_motor_set_velocity(left_motor, 0);
        // wb_motor_set_velocity(right_motor, 0);
        //goto label;
      }

      if (STATE == INIT_SEARCH)
      {
        totalRot = 0;
        prevDir = get_bearing_in_degrees(compass);

        STATE = SEARCH;
      }
      else if (STATE == SEARCH)
      {
        //turn left
        wb_motor_set_velocity(left_motor, SPEED);
        wb_motor_set_velocity(right_motor, -SPEED);

        totalRot += get_turning_in_degrees(compass, prevDir);
        prevDir = get_bearing_in_degrees(compass);
        // printf("totalRot %f\n", totalRot);

        if (isRedVisible(camera, height, width, searchLowIntensity))
        {
          STATE = TARGET;
          left_speed = 0;
          right_speed = 0;
          printf("found target\n");
        }
        // else if(totalRot > 360.0f)
        // {
        // STATE = NONE;
        // left_speed = 0;
        // right_speed = 0;
        // printf("not found target\n");
        // }
      }
      else if (STATE == TARGET)
      {
        left_ds_dist = wb_distance_sensor_get_value(left_dist_sensor);
        right_ds_dist = wb_distance_sensor_get_value(right_dist_sensor);
        forward_ds_dist = wb_distance_sensor_get_value(forward_dist_sensor);

        left_obstracle = left_ds_dist < 200 ? true : false;
        right_obstracle = right_ds_dist < 200 ? true : false;
        forward_obstracle = forward_ds_dist < 200 ? true : false;
        // printf("right dist %f", right_ds_dist);

        wb_motor_set_velocity(left_motor, SPEED);
        wb_motor_set_velocity(right_motor, SPEED);

        bool anyObstacle = forward_obstracle || left_obstracle || right_obstracle;
        if (anyObstacle && isRedVisible(camera, height, width, searchLowIntensity))
        {
          STATE = WEIGHT_MEASURE;
          wb_motor_set_velocity(left_motor, 0);
          wb_motor_set_velocity(right_motor, 0);
          
          printf("success\n");
        }
        else if (left_obstracle)
        {
          //turn right
          wb_motor_set_velocity(left_motor, SPEED);
          wb_motor_set_velocity(right_motor, -SPEED);
          printf("left obstracle\n");
        }
        else if (right_obstracle)
        {
          //turn left
          wb_motor_set_velocity(left_motor, -SPEED);
          wb_motor_set_velocity(right_motor, SPEED);

          printf("right obstracle\n");
        }
        else if (!isRedVisible(camera, height, width, searchLowIntensity) && !anyObstacle)
        {
          STATE = INIT_SEARCH;
          printf("refocus\n");
        }
        
      }
      else if(STATE == WEIGHT_MEASURE){
            f = wb_touch_sensor_get_value(weight);
            d = f - 0.0247686;
            if (d>0.1f && d<10)
            {
             printf("weight sensor reading: %g N.\n", d);
             STATE = SUCCESS;
            
             goto label;
            }
            

      }

      if (STATE != NONE)
        continue;

      //////////////

      if (avoid_obstacle_counter > 0)
      {
        avoid_obstacle_counter--;
        left_speed = 1.0;
        right_speed = -1.0;
      }
      else
      { // read sensors
        double ds_values[2];
        for (i = 0; i < 2; i++)
          ds_values[i] = wb_distance_sensor_get_value(ds[i]);
        if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
          avoid_obstacle_counter = 100;
      }

      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);
    }

    /* set speed values */
    if (d > 2)
    {
      h = 1;
    }
    if (h > 0)
    {
      const double *gps_values = wb_gps_get_values(gps);
      double l = wb_position_sensor_get_value(left_position_sensor);
      double r = wb_position_sensor_get_value(right_position_sensor);
      double dl = l * WHEEL_RADIUS;        // distance covered by left wheel in meter
      double dr = r * WHEEL_RADIUS;        // distance covered by right wheel in meter
      double da = (dr - dl) / AXLE_LENGTH; // delta orientation
      double sa = da * (-180 / PI);
      double gps0 = gps_values[0];
      double gps2 = gps_values[2];
      double k = (pow(gps0, p)) + (pow(gps2, p));
      double gh = sqrt(k);
      printf("orientation of robot in degrees: %g degrees.\n", sa);
      printf("distance from centre: %g m.\n", gh);
      double al = (atan((gps2) / (gps0)));
      double aal = (PI / 2) - al;
      double a = al * (180 / PI);
      double aa = aal * (180 / PI);
      printf("inside angle: %g degrees.\n", aa);
      printf("outside angle: %g degrees.\n", a);
      // printf("angle from centre: %g degrees.\n",a);
      if (gps0 > 0)
      {
        if (gps2 > 0)
        {
          z = 1;
        }
        else if (gps2 < 0)
        {
          z = 3;
        }
      }
      if (gps0 < 0)
      {
        if (gps2 > 0)
        {
          z = 2;
        }
        else if (gps2 < 0)
        {
          z = 4;
        }
      }
      if (z = 1)
      {
        double gkd = 180 + aa;
        double gd = 360 - gkd;
        double m = sa - gd;
        printf("angle to centre: %g degrees.\n", m);
        if (avoid_obstacle_counter > 0)
        {
          if (aa <= 45)
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 3.0);
            wb_motor_set_velocity(right_motor, 0.5);
          }
          else
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 0.5);
            wb_motor_set_velocity(right_motor, 3.0);
          }
        }
        else
        { // read sensors
          double ds_values[2];
          for (i = 0; i < 2; i++)
            ds_values[i] = wb_distance_sensor_get_value(ds[i]);
          if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
            avoid_obstacle_counter = 100;
        }
        if (avoid_obstacle_counter < 1)
        {
          if (m < -1)
          {
            wb_motor_set_velocity(left_motor, +0.3);
            wb_motor_set_velocity(right_motor, -0.3);
          }
          else if (m > 1)
          {
            wb_motor_set_velocity(left_motor, -0.3);
            wb_motor_set_velocity(right_motor, +0.3);
          }
          else if (m >= -1)
          {
            if (m <= 1)
            {
              wb_motor_set_velocity(left_motor, 1);
              wb_motor_set_velocity(right_motor, 1);
            }
          }
          if (gps0 < 0)
          {
            if (gps2 < 0)
            {
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            }
          }
        }
      }
      if (z = 2)
      {
        double gkd = 180 + aa;
        double gd = 360 - gkd;
        double m = +sa - gd - 180;
        printf("angle to centre: %g degrees.\n", m);
        if (avoid_obstacle_counter > 0)
        {
          if (aa <= 135)
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 3.0);
            wb_motor_set_velocity(right_motor, 0.5);
          }
          else
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 0.5);
            wb_motor_set_velocity(right_motor, 3.0);
          }
        }
        else
        { // read sensors
          double ds_values[2];
          for (i = 0; i < 2; i++)
            ds_values[i] = wb_distance_sensor_get_value(ds[i]);
          if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
            avoid_obstacle_counter = 100;
        }
        if (avoid_obstacle_counter < 1)
        {
          if (m < -1)
          {
            wb_motor_set_velocity(left_motor, +0.3);
            wb_motor_set_velocity(right_motor, -0.3);
          }
          else if (m > 1)
          {
            wb_motor_set_velocity(left_motor, -0.3);
            wb_motor_set_velocity(right_motor, +0.3);
          }
          else if (m >= -1)
          {
            if (m <= 1)
            {
              wb_motor_set_velocity(left_motor, 1);
              wb_motor_set_velocity(right_motor, 1);
            }
          }
          if (gps0 > 0)
          {
            if (gps2 < 0)
            {
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            }
          }
        }
      }
      if (z = 3)
      {
        double gkd = 180 + aa;
        double gd = 360 - gkd;
        double m = +sa - gd;
        printf("angle to centre: %g degrees.\n", m);
        if (avoid_obstacle_counter > 0)
        {
          if (aa <= 135)
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 3.0);
            wb_motor_set_velocity(right_motor, 0.5);
          }
          else
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 0.5);
            wb_motor_set_velocity(right_motor, 3.0);
          }
        }
        else
        { // read sensors
          double ds_values[2];
          for (i = 0; i < 2; i++)
            ds_values[i] = wb_distance_sensor_get_value(ds[i]);
          if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
            avoid_obstacle_counter = 100;
        }
        if (avoid_obstacle_counter < 1)
        {
          if (m < -1)
          {
            wb_motor_set_velocity(left_motor, +0.3);
            wb_motor_set_velocity(right_motor, -0.3);
          }
          else if (m > 1)
          {
            wb_motor_set_velocity(left_motor, -0.3);
            wb_motor_set_velocity(right_motor, +0.3);
          }
          else if (m >= -1)
          {
            if (m <= 1)
            {
              wb_motor_set_velocity(left_motor, 1);
              wb_motor_set_velocity(right_motor, 1);
            }
          }
          if (gps0 < 0)
          {
            if (gps2 > 0)
            {
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            }
          }
        }
      }
      if (z = 4)
      {
        double gkd = 180 + aa;
        double gd = 360 - gkd;
        double m = +sa - gd - 180;
        printf("angle to centre: %g degrees.\n", m);
        if (avoid_obstacle_counter > 0)
        {
          if (aa <= 45)
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 4.0);
            wb_motor_set_velocity(right_motor, 0.5);
          }
          else
          {
            avoid_obstacle_counter--;
            wb_motor_set_velocity(left_motor, 0.5);
            wb_motor_set_velocity(right_motor, 4.0);
          }
        }
        else
        { // read sensors
          double ds_values[2];
          for (i = 0; i < 2; i++)
            ds_values[i] = wb_distance_sensor_get_value(ds[i]);
          if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
            avoid_obstacle_counter = 100;
        }
        if (avoid_obstacle_counter < 1)
        {
          if (m < -1)
          {
            wb_motor_set_velocity(left_motor, +0.3);
            wb_motor_set_velocity(right_motor, -0.3);
          }
          else if (m > 1)
          {
            wb_motor_set_velocity(left_motor, -0.3);
            wb_motor_set_velocity(right_motor, +0.3);
          }
          else if (m >= -1)
          {
            if (m <= 1)
            {
              wb_motor_set_velocity(left_motor, 1);
              wb_motor_set_velocity(right_motor, 1);
            }
          }
          if (gps0 > 0)
          {
            if (gps2 > 0)
            {
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            }
          }
        }
      }
    }
  }
label:

  wb_robot_cleanup();
  return 0;
}