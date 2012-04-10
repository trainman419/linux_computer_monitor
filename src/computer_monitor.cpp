/*
 * computer_monitor
 *
 * A node to read lm_sensors on Linux and publish sensor values and diagnostics
 *
 * Developed against libsensors4 and libsensors4-dev on Ubuntu 10.04
 *  - this doesn't work against older versions of libsensors
 *
 * large parts of this program are based on lm-sensors prog/sensors/chips.c
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

// libsensors; see libsensors(3)
// and http://www.lm-sensors.org/browser/lm-sensors/trunk/doc/developers/applications
#include <sensors/sensors.h>
#include <sensors/error.h> // for sensors_strerror

#include <string.h>
#include <errno.h>

#include <boost/foreach.hpp>

// helper methods

static double get_value(const sensors_chip_name * name, 
      const sensors_subfeature * sf) {
   double val;
   int err;
   err = sensors_get_value(name, sf->number, &val);
   if(err) {
      // TODO: error-handling here
      val = 0;
   }
   return val;
}

static int get_input_value(const sensors_chip_name * name,
      const sensors_subfeature * sf, double * val) {
   int err;
   err = sensors_get_value(name, sf->number, val);
   if(err) {
      // TODO: error-handling here
   }
   return err;
}


// A superclass to abstract the interface to sensors
class Sensor {
   protected:
      const sensors_chip_name * name;
      const sensors_feature * feature;
      std::string label;

   public:
      Sensor(const sensors_chip_name * n, const sensors_feature * f) : name(n), 
       feature(f) {
         char * l = sensors_get_label(n, f);
         label = l;
         free(l);
      }

      virtual double getValue() = 0;
      std::string getStringValue() {
         return label + ": " + getString(getValue());
      }

      std::string getStringValue(double val) {
         return label + ": " + getString(val);
      }
      int getType() { return feature->type; }

      virtual Sensor * clone() = 0;

      virtual ~Sensor() {}
      
   protected:
      virtual std::string getString(double val) = 0;
};

// Temperature sensor
class SensorTemp : public Sensor {
   private:
   public: 
      SensorTemp(const sensors_chip_name * n, const sensors_feature * f) :
       Sensor(n, f) {
      }

      virtual double getValue() { 
         const sensors_subfeature * sf;
         sf = sensors_get_subfeature(name, feature, 
               SENSORS_SUBFEATURE_TEMP_FAULT);
         if( sf && get_value(name, sf) ) {
            return INFINITY;
         } else {
            double val = 0.0;
            sf = sensors_get_subfeature(name, feature,
                  SENSORS_SUBFEATURE_TEMP_INPUT);
            if(sf && get_input_value(name, sf, &val) == 0 ) {
               return val;
            }
         }
         return INFINITY; 
      }

      virtual std::string getString(double val) { 
         char buf[256];
         snprintf(buf, 256, "%+6.1f", val);
         return std::string(buf);
      }

      virtual Sensor * clone() { return new SensorTemp(name, feature); }

      virtual ~SensorTemp() {}
};

// A dummy sensor implementation
class SensorDummy : public Sensor {
   public:
      SensorDummy(const sensors_chip_name * n, const sensors_feature * f) :
       Sensor(n, f) {
      }

      virtual double getValue() { return 0.0; }
      virtual std::string getString(double val) { return "Dummy Sensor"; } 
      virtual Sensor * clone() { return new SensorDummy(name, feature); }

      virtual ~SensorDummy() {}
};

// A feature is one particular sensor on a chip, with associated subfeatures 
//  for the value, limits and alarms
class Feature {
   Sensor * sensor;

public:

   Feature(const sensors_chip_name *n, const sensors_feature *f)  {
      sensor = 0;

      switch(f->type) {
         case SENSORS_FEATURE_TEMP:
            sensor = new SensorTemp(n, f);
            break;
         default:
            sensor = new SensorDummy(n, f);
            break;
      }
   }

   Feature(const Feature & o) {
      sensor = o.sensor->clone();
   }

   ~Feature() {
      if(sensor) delete sensor;
   }

   // get the value for this feature
   double getValue() { return sensor->getValue(); }

   // get the type of this feature
   int getType() { return sensor->getType(); }

   // get a human-readable representation of this feature.
   //  this probably includes label, value, units and limits
   std::string getStringValue() { return sensor->getStringValue(); }
   std::string getStringValue(double v) { return sensor->getStringValue(v); }
};

// A chip is some piece of monitoring hardware
class Chip {
   const sensors_chip_name * chip;
   static const int NAME_SZ = 256;
   char name[NAME_SZ];
   std::list<Feature> features;

public:

   Chip(const sensors_chip_name * c) : chip(c) {
      // populate name
      if(sensors_snprintf_chip_name(name, NAME_SZ, c) < 0) {
         // error: bad chip name
         snprintf(name, NAME_SZ, "Unknown chip");
      }

      // populate features
      int i = 0;
      const sensors_feature * feature;
      while( (feature = sensors_get_features(chip, &i)) ) {
         features.push_back(Feature(c, feature));
      }
   }

   char * getName() { return name; }
   const sensors_chip_name * getChip() { return chip; }

   std::list<Feature> & getFeatures() { return features; }

   int getNumTempSensors() {
      int res = 0;
      BOOST_FOREACH(Feature f, features) {
         if( f.getType() == SENSORS_FEATURE_TEMP ) ++res;
      }
      return res;
   }
};


int main(int argc, char ** argv) {

   // initialize ROS
   ros::init(argc, argv, "computer_monitor");
   ros::NodeHandle nh;
   ros::Rate loop_rate(1.0); // publish at 1Hz

   ros::Publisher data_pub = nh.advertise<std_msgs::Float64MultiArray>(
         "lm_sensors", 10);

   // initialize libsensors
   //  we can pass NULL for the config file to use the default,
   const char * libsensors_conf_file = NULL;
   FILE * config_file;
   if( libsensors_conf_file ) {
      config_file = fopen(libsensors_conf_file, "r");
      if( !config_file ) {
         ROS_ERROR("Could not open config file %s: %s", libsensors_conf_file,
               strerror(errno));
         return -1;
      }
   } else {
      config_file = NULL;
   }

   int err = sensors_init(config_file);
   if(err) {
      ROS_ERROR("sensors_init: %s", sensors_strerror(err));
      if( config_file ) fclose(config_file);
      return -1;
   }

   std::list<Chip> chips;

   const sensors_chip_name * chip;
   int chip_nr = 0;
   while( (chip = sensors_get_detected_chips(NULL, &chip_nr)) ) {
      chips.push_back(Chip(chip));
   }

   int temp_sensors = 0;

   BOOST_FOREACH(Chip c, chips) {
      temp_sensors = c.getNumTempSensors();
   }

   std_msgs::MultiArrayLayout layout;
   std_msgs::MultiArrayDimension dim;
   dim.label = "temperature";
   dim.size = temp_sensors;
   dim.stride = temp_sensors;
   layout.dim.push_back(dim);

   ROS_INFO("computer_monitor ready with %d temperature sensors", temp_sensors);

   while( ros::ok() ) {
      std_msgs::Float64MultiArray data;
      data.layout = layout;

      BOOST_FOREACH(Chip c, chips) {
         ROS_INFO("%s", c.getName());
         BOOST_FOREACH(Feature f, c.getFeatures()) {
            double val = f.getValue();
            ROS_INFO("%s", f.getStringValue(val).c_str());
            data.data.push_back(val);
         }
      }

      data_pub.publish(data);

      ros::spinOnce();
      loop_rate.sleep();
   }

   sensors_cleanup();

   return 0;
}
