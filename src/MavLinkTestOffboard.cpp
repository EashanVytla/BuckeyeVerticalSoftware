#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>

#define PORT_PATH "serial:///dev/ttyTHS1"

using namespace mavsdk;
using namespace std;
using std::chrono::seconds;
using std::this_thread::sleep_for;

double targetLatitude = 0;
double targetLongitude = 0;
double currentLatitude = 0;
double currentLongitude = 0;

enum class State {
    INITIAL,
    WAYPOINT1,
    WAYPOINT2,
    WAYPOINT3,
    WAYPOINT4
};

enum class ReverseState {
    WAYPOINT1,
    WAYPOINT2,
    WAYPOINT3,
    WAYPOINT4,
    HOME,
    STOP,
    FINISHED
};

double distance(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude) {
   double distance = abs(sqrt(pow(currLatitude - targetLatitude,2) + pow(currLongitude - targetLongitude,2)));
   return distance; 
}

float yaw(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude) {
    double position;
    if ((targetLongitude - currLongitude) == 0) {
        position = 0;
    }
    position = (targetLatitude - currLatitude) / (targetLongitude - currLongitude);
    return (float) atan(position) * (180/M_PI);
}

void move_to_next_waypoint(Offboard& offboard, Telemetry& telemetry, State& current_state, Offboard::Result& offboard_result, 
double targetLatitude, double targetLongitude, double coordinates[], ofstream& myfile)
{

    switch (current_state) {
        case State::INITIAL: {
            //Waypoint1
            targetLatitude = coordinates[0];
            targetLongitude = coordinates[1];
            const Offboard::PositionGlobalYaw waypoint1{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude), 
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint1);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT1;
                std::cout << "Made it to waypoint1\n";
                myfile << "Made it to waypoint1\n";
            } 
        } break;

        case State::WAYPOINT1: {
            //Waypoint2 (East)
            targetLatitude = coordinates[2];
            targetLongitude = coordinates[3];
            const Offboard::PositionGlobalYaw waypoint2{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint2);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT2;
                std::cout << "Made it to waypoint2\n";
                myfile << "Made it to waypoint2\n";
            }
        } break;

        case State::WAYPOINT2: {
            //Waypoint3
            targetLatitude = coordinates[4];
            targetLongitude = coordinates[5];
            const Offboard::PositionGlobalYaw waypoint3{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint3);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT3;
                std::cout << "Made it to waypoint3\n";
                myfile << "Made it to waypoint3\n";
            }
        } break;

        case State::WAYPOINT3: {
            //Waypoint4
            targetLatitude = coordinates[6];
            targetLongitude = coordinates[7];
            const Offboard::PositionGlobalYaw waypoint4{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint4);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT4;
                std::cout << "Made it to waypoint4\n";
                myfile << "Made it to waypoint4\n";
            }
        } break;

        case State::WAYPOINT4: {
        } break;
}
}

void reverse_move_to_next_waypoint(Offboard& offboard, Telemetry& telemetry, ReverseState& reverse_state, Offboard::Result& offboard_result, 
double originLatitude, double originLongitude, double targetLatitude, double targetLongitude, double coordinates[], ofstream& myfile)
{

    switch (reverse_state) {

        case ReverseState::WAYPOINT4: {
            //Home
            targetLatitude = coordinates[4];
            targetLongitude = coordinates[5];
            const Offboard::PositionGlobalYaw waypoint3{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint3);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::WAYPOINT3;
                std::cout << "Made it to waypoint3\n";
                myfile << "Made it to waypoint3\n";
            }
        } break;

        case ReverseState::WAYPOINT3: {
            //Waypoint4
            targetLatitude = coordinates[2];
            targetLongitude = coordinates[3];
            const Offboard::PositionGlobalYaw waypoint2{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint2);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::WAYPOINT2;
                std::cout << "Made it to waypoint2\n";
                myfile << "Made it to waypoint2\n";
            }
        } break;

        case ReverseState::WAYPOINT2: {
            //Waypoint3
            targetLatitude = coordinates[0];
            targetLongitude = coordinates[1];
            const Offboard::PositionGlobalYaw waypoint1{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint1);
            std::cout << "Going to waypoint3 at 7m relative altitude\n";
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::WAYPOINT1;
                std::cout << "Made it to waypoint1\n";
                myfile << "Made it to waypoint1\n";
            }
        } break;

        case ReverseState::WAYPOINT1: {
            //Home
            targetLatitude = originLatitude;
            targetLongitude = originLongitude;
            const Offboard::PositionGlobalYaw home{
                targetLatitude,
                targetLongitude,
                7.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(home);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::HOME;
                std::cout << "Made it home\n";
                myfile << "Made it home\n";
            }
        } break;

        case ReverseState::HOME: {
            // Stop offboard mode
            offboard_result = offboard.stop();
            if (offboard_result != Offboard::Result::Success) {
            std::cerr << "Offboard stop failed: " << offboard_result << '\n';
            myfile << "Offboard stop failed: " << offboard_result << '\n';
            }
            reverse_state = ReverseState::STOP;
        } break;

        case ReverseState::STOP: {
            reverse_state = ReverseState::FINISHED;
        } break;

        case ReverseState::FINISHED: {
            std::cout << "Finished\n";
            myfile << "Finished\n";
        } break;
    }
}

int main()
{
    std::ofstream myfile("Logs.txt");
    if(!myfile.is_open()){
      cout << "File open failed! Ending program." << endl;
      return 0;
   }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        myfile << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        myfile << "Timed out waiting for system\n";
        return 1;
    }

    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};
    auto param = Param{system.value()};

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        myfile << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";
    myfile << "System is ready\n";

    const auto set_velocity = param.set_param_float("MPC_XY_VEL_MAX", 5.0);
    if (set_velocity != Param::Result::Success) {
        std::cerr << "Velocity not set: " << set_velocity << '\n';
        myfile << "Velocity not set: " << set_velocity << '\n';
        return 1;
    }

    while(telemetry.flight_mode() != Telemetry::FlightMode::Offboard) {
        std::cout << "Waiting for Offboard\n";
        myfile << "Waiting for Offboard\n";
        const Offboard::PositionGlobalYaw currLocation{
                telemetry.position().latitude_deg,
                telemetry.position().longitude_deg,
                7.0f,
                0.0f};
        offboard.set_position_global(currLocation);
        sleep_for(400ms);
    }

    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
        [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                telemetry.unsubscribe_landed_state(handle);
                in_air_promise.set_value();
            }
        });
    
    State current_state = State::INITIAL;
    //State previous_state = State::INITIAL;
    Offboard::Result offboard_result;
    double originLatitude = telemetry.position().latitude_deg;
    double originLongitude = telemetry.position().longitude_deg;
    double coordinates[] = {telemetry.position().latitude_deg + 0.0001, telemetry.position().longitude_deg, telemetry.position().latitude_deg, 
    telemetry.position().longitude_deg + 0.0001,telemetry.position().latitude_deg - 0.0001, telemetry.position().longitude_deg,
    telemetry.position().latitude_deg, telemetry.position().longitude_deg - 0.0001};
    //double coordinates[] = {40.0084244, -83.0175219, 40.0085887, -83.0175251, 40.0085853, -83.0173310, 40.0084330, -83.0173369};

    while (current_state != State::WAYPOINT4) {
        move_to_next_waypoint(offboard, telemetry, current_state, offboard_result, targetLatitude, targetLongitude, coordinates, myfile);
        sleep_for(400ms);
    }

    ReverseState reverse_state = ReverseState::WAYPOINT4;
    while (reverse_state != ReverseState::FINISHED) {
        reverse_move_to_next_waypoint(offboard, telemetry, reverse_state, offboard_result, originLatitude, originLongitude, 
        targetLatitude, targetLongitude, coordinates, myfile);
        sleep_for(400ms);
    }

    const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Landing failed: " << land_result << '\n';
        myfile << "Landing failed: " << land_result << '\n';
        return 1;
    }

    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        myfile << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";
    myfile << "Landed!\n";

    sleep_for(seconds(3));
    std::cout << "Finished...\n";
    myfile << "Finished...\n";

    return 0;
    
}
