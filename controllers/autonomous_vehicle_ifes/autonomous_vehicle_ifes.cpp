/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Autonoumous vehicle controller example
 */

#include <webots/Camera.hpp>
#include <webots/device.hpp>
#include <webots/display.hpp>
#include <webots/gps.hpp>
//#include <webots/keyboard.hpp>
#include <webots/Keyboard.hpp>
#include <webots/lidar.hpp>
#include <webots/robot.hpp>
//#include <webots/vehicle/driver.hpp>
#include <webots/vehicle/Driver.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

webots::Keyboard teclado;
webots::Driver motorista;


// to be used as array indices
enum { X, Y, Z };



//const int TIME_STEP = 50;
#define TIME_STEP 50

#define UNKNOWN 99999.99

// Line following PID
#define KP 0.25
#define KI 0.006
#define KD 2

bool PID_need_reset = false;

// Size of the yellow line angle filter
#define FILTER_SIZE 3

//nabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

//camera
//WbDeviceTag camera;
webots::Camera *camera;

int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
//WbDeviceTag sick; ----->
webots::Lidar *sick_laser_sensor;

int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
//WbDeviceTag display; ----->
webots::Display *display;

int display_width = 0;
int display_height = 0;
//WbImageRef speedometer_image = NULL; ----->
webots::ImageRef *speedometer_image;

// GPS
//WbDeviceTag gps; ----->
webots::GPS *gps;

double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;


// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

void print_help() {
  std::cout << "You can drive this car!\n";
  std::cout << "Select the 3D window and then use the cursor keys to:\n";
  std::cout << "[LEFT]/[RIGHT] - steer\n";
  std::cout << "[UP]/[DOWN] - accelerate/slow down\n";
}

void set_autodrive(bool onoff) {
  if (autodrive == onoff)
    return;
  autodrive = onoff;
  switch (autodrive) {
    case false:
      std::cout << "switching to manual drive...\n";
      std::cout << "hit [A] to return to auto-drive.\n";
      break;
    case true:
      if (has_camera)
        std::cout << "switching to auto-drive...\n";
      else
        std::cout << "impossible to switch auto-drive on without camera...\n";
      break;
  }
}

// set target speed
void set_speed(double kmh) {
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;

  speed = kmh;

  std::cout<<"setting speed to "<< kmh <<" km/h\n";
  
  motorista.setCruisingSpeed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  motorista.setSteeringAngle(wheel_angle);
}

void change_manual_steer_angle(int inc) {
  set_autodrive(false);

  double new_manual_steering = manual_steering + inc;
  if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0) {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering * 0.02);
  }

  if (manual_steering == 0)
    std::cout << "going straight\n";
  else
    std::cout << "turning " << steering_angle << " rad " << (steering_angle < 0 ? "left" : "right") << "\n";
}

void check_keyboard() {
  int key = teclado.getKey();
  switch (key) {
    case webots::Keyboard::UP:
      set_speed(speed + 5.0);
      break;
    case webots::Keyboard::DOWN:
      set_speed(speed - 5.0);
      break;
    case webots::Keyboard::RIGHT:
      change_manual_steer_angle(+1);
      break;
    case webots::Keyboard::LEFT:
      change_manual_steer_angle(-1);
      break;
    case 'A':
      set_autodrive(true);
      break;
  }
}

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++;  // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}

// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
double process_sick_data(const float *sick_data, double *obstacle_dist) {
  const int HALF_AREA = 20;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = sick_data[x];
    if (range < 20.0) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }

  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;

  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

void update_display() {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  
  //wb_display_image_paste(display, speedometer_image, 0, 0, false); ----->
  display->imagePaste(speedometer_image, 0, 0, false);
  
  // draw speedometer needle
  double current_speed = motorista.getCurrentSpeed();
  if (std::isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
 
  //wb_display_draw_line(display, 100, 95, 100 + x, 95 + y); ----->
  display->drawLine(100, 95, 100 + x, 95 + y);
  
  // draw text
  //char txt[64]; ----->
  //sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]); ----->
  std::string txt;
  txt = "GPS coords: " + std::to_string(gps_coords[X]) + " " + std::to_string(gps_coords[Z]);

 //wb_display_draw_text(display, txt, 10, 130); ----->
  display->drawText(txt, 10, 130);

  //sprintf(txt, "GPS speed:  %.1f", gps_speed); ----->
  txt = "GPS speed: " + std::to_string(gps_speed);

  //wb_display_draw_text(display, txt, 10, 140); ----->
   display->drawText(txt, 10, 140);
}

void compute_gps_speed() {
  //const double *coords = wb_gps_get_values(gps); ----->
  const double *coords = gps->getValues();

  //const double speed_ms = wb_gps_get_speed(gps); ----->
  const double speed_ms = gps->getSpeed();

  // store into global variables
  gps_speed = speed_ms * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_need_reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }

  // anti-windup mechanism
  if (std::signbit(yellow_line_angle) != std::signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return KP * yellow_line_angle + KI * integral + KD * diff;
}



int main(int argc, char **argv) {
  //wbu_driver_init(); substituido pelo construtor da classe Driver

  // check if there is a SICK and a display
  
  /*
  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "Sick LMS 291") == 0)
      enable_collision_avoidance = true;
    else if (strcmp(name, "display") == 0)
      enable_display = true;
    else if (strcmp(name, "gps") == 0)
      has_gps = true;
    else if (strcmp(name, "camera") == 0)
      has_camera = true;
  }
  
  ------->
  
*/ 


  for (int j = 0; j < motorista.getNumberOfDevices(); ++j) {
      webots::Device *device = motorista.getDeviceByIndex(j);
      std::string name = device->getName();
      if (name == "Sick LMS 291")
        enable_collision_avoidance = true;
      else if (name == "display")
        enable_display = true;
      else if (name == "gps")
        has_gps = true;
      else if (name == "camera")
        has_camera = true;
    }
  
  
   //camera device
  if (has_camera) {
    //camera = wb_robot_get_device("camera");
    camera = motorista.getCamera("camera");

    //wb_camera_enable(camera, TIME_STEP); ----->
    camera->enable(TIME_STEP);

    //camera_width = wb_camera_get_width(camera); ----->
    camera_width = camera->getWidth();

    //camera_height = wb_camera_get_height(camera); ----->
    camera_height = camera->getHeight();

    //camera_fov = wb_camera_get_fov(camera); ----->
    camera_fov = camera->getFov();
   }

  // SICK sensor
  if (enable_collision_avoidance) {
    //sick = wb_robot_get_device("Sick LMS 291"); ----->
    sick_laser_sensor = motorista.getLidar("Sick LMS 291");

    //wb_lidar_enable(sick, TIME_STEP); ----->
    sick_laser_sensor->enable(TIME_STEP);

    //sick_width = wb_lidar_get_horizontal_resolution(sick); ----->
    sick_width = sick_laser_sensor->getHorizontalResolution();

    //sick_range = wb_lidar_get_max_range(sick); ----->
    sick_range = sick_laser_sensor->getMaxRange();

    //sick_fov = wb_lidar_get_fov(sick); ----->
    sick_fov = sick_laser_sensor->getFov();
  }

  // initialize gps
  if (has_gps) {
    //gps = wb_robot_get_device("gps");
    gps = motorista.getGPS("gps");

    //wb_gps_enable(gps, TIME_STEP); ----->
    gps->enable(TIME_STEP);
  }

  // initialize display (speedometer)
  if (enable_display) {

    //display = wb_robot_get_device("display"); ----->
    display = motorista.getDisplay("display");

    //speedometer_image = wb_display_image_load(display, "speedometer.png");
    speedometer_image = display->imageLoad("speedometer.png");
  }

  // start engine
  if (has_camera)
    set_speed(50.0);  // km/h
  //wbu_driver_set_hazard_flashers(true); ----->
  motorista.setHazardFlashers(true);

  //wbu_driver_set_dipped_beams(true); ----->
  motorista.setDippedBeams(true);

  //wbu_driver_set_antifog_lights(true); ----->
  motorista.setAntifogLights(true);

  //wbu_driver_set_wiper_mode(SLOW); ----->
  motorista.setWiperMode(webots::Driver::WiperMode::SLOW);

  print_help();

  // allow to switch to manual control
  teclado.enable(TIME_STEP);
 
  // main loop
  while (motorista.step() != -1) {
    // get user input
    check_keyboard();
    static int i = 0;
    
    // updates sensors only every TIME_STEP milliseconds
    //if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) { ---->
      // read sensors
      if(i % static_cast<int>(TIME_STEP / motorista.getBasicTimeStep()) == 0) {

        const unsigned char *camera_image = NULL;
        const float *sick_data = NULL;
        if (has_camera)
          //camera_image = wb_camera_get_image(camera);
          camera_image = camera->getImage();

        if (enable_collision_avoidance)      
          //sick_data = wb_lidar_get_range_image(sick);
          sick_data = sick_laser_sensor->getRangeImage();

        if (autodrive && has_camera) {
          double yellow_line_angle = filter_angle(process_camera_image(camera_image));
          double obstacle_dist;
          double obstacle_angle;
          if (enable_collision_avoidance)
            obstacle_angle = process_sick_data(sick_data, &obstacle_dist);

          // avoid obstacles and follow yellow line
          if (enable_collision_avoidance && obstacle_angle != UNKNOWN) {
            // an obstacle has been detected

            //wbu_driver_set_brake_intensity(0.0);
            motorista.setBrakeIntensity(0.0);

            // compute the steering angle required to avoid the obstacle
            double obstacle_steering = steering_angle;
            if (obstacle_angle > 0.0 && obstacle_angle < 0.4)
              obstacle_steering = steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
            else if (obstacle_angle > -0.4)
              obstacle_steering = steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
            double steer = steering_angle;
            // if we see the line we determine the best steering angle to both avoid obstacle and follow the line
            if (yellow_line_angle != UNKNOWN) {
              const double line_following_steering = applyPID(yellow_line_angle);
              if (obstacle_steering > 0 && line_following_steering > 0)
                steer = obstacle_steering > line_following_steering ? obstacle_steering : line_following_steering;
              else if (obstacle_steering < 0 && line_following_steering < 0)
                steer = obstacle_steering < line_following_steering ? obstacle_steering : line_following_steering;
            } else
              PID_need_reset = true;
            // apply the computed required angle
            set_steering_angle(steer);
          } else if (yellow_line_angle != UNKNOWN) {
            // no obstacle has been detected, simply follow the line

            //wbu_driver_set_brake_intensity(0.0);
            motorista.setBrakeIntensity(0.0);

            set_steering_angle(applyPID(yellow_line_angle));
          } else {
            // no obstacle has been detected but we lost the line => we brake and hope to find the line again

            //wbu_driver_set_brake_intensity(0.4);
            motorista.setBrakeIntensity(0.4);

            PID_need_reset = true;
        }
      }

      // update stuff
      if (has_gps)
        compute_gps_speed();
      if (enable_display)
        update_display();
    }

    
    //
    ++i;
  }
  

  return 0;
}
