#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>
#include <iostream>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <thread>
#include <PWMUtils.h>

//Fix these
#define PORT_PATH "serial:///dev/ttyTHS1"
#define STARTUP_PARAM "-"
#define STARTUP_PARAM_THRESH 5

using namespace mavsdk;

int main(){
   //Create mavsdk object on stack
   Mavsdk mavsdk;
   
   //Connect to the Pixhawk
   ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);
   
   //Log connection failure
   if (connection_result != ConnectionResult::Success) {
      std::cout << "Adding connection failed: " << connection_result << '\n';
      return 0;
   }

   while (mavsdk.systems().size() == 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
   }

   std::cout << "Connected!" << '\n';

   // Get the list of systems
   auto systems = mavsdk.systems();

   // Check if there is at least one system
   if (systems.empty()) {
      std::cout << "No MAVLink system found!";
      return 0;
   }
    
   std::shared_ptr<mavsdk::System> system = systems[0];

   PWMUtils pwm;

   std::cout << "Waiting for Channel 5 to be switched" << std::endl;

   //Waiting for channel 5 (pin 168) to reach 1200 ElapsedTime
   pwm.waitTillChannel(168, 1200);

   std::cout << "Channel 5 switched!" << std::endl;

   auto mission = Mission{system};

   //Create a vector to store all waypoints
   std::vector<std::shared_ptr<Mission::MissionItem>> mission_items;

   //Waypoint 1
   std::shared_ptr<Mission::MissionItem> waypoint1;
   waypoint1->latitude_deg = 0; //Get these from QGC
   waypoint1->longitude_deg = 0;
   waypoint1->speed_m_s = 0;

   //Waypoint 2
   std::shared_ptr<Mission::MissionItem> waypoint2;
   waypoint2->latitude_deg = 0;
   waypoint2->longitude_deg = 0;
   waypoint2->speed_m_s = 0;

   //Waypoint 3
   std::shared_ptr<Mission::MissionItem> waypoint3;
   waypoint3->latitude_deg = 0;
   waypoint3->longitude_deg = 0;
   waypoint3->speed_m_s = 0;

   //Waypoint 4
   std::shared_ptr<Mission::MissionItem> waypoint4;
   waypoint4->latitude_deg = 0;
   waypoint4->longitude_deg = 0;
   waypoint4->speed_m_s = 0;

   mission_items.push_back(waypoint1); 
   mission_items.push_back(waypoint2); 
   mission_items.push_back(waypoint3); 
   mission_items.push_back(waypoint4); 

   std::cout << "Uploading mission..." << '\n';
   //Mission::MissionPlan mission_plan{};
   //mission_plan.mission_items = mission_items;
}