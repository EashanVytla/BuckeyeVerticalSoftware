#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <thread>

#define PORT_PATH "udp://localhost:14540"

enum class ProgramState {
    Initializing,
    WaitingForChannel5,
    Channel5Switched,
    Start,
    Waypoint1,
    Waypoint2,
    Waypoint3,
    Waypoint4,
    OffboardModeFailed,
    Exiting,
};

ProgramState program_state = ProgramState::Initializing;

int main() {
    // Create mavsdk object on stack
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::CompanionComputer}};

    // Connect to the Pixhawk
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);

    // Log connection failure
    if (mavsdk::ConnectionResult::connection_result != mavsdk::ConnectionResult::Success) {
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

    auto offboard = mavsdk::Offboard{system};

    std::cout << "Waiting for Channel 5 to be switched" << std::endl;

    // Waiting for channel 5 (pin 168) to reach 1200 ElapsedTime

    std::cout << "Channel 5 switched!" << std::endl;

    offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

    mavsdk::Offboard offboard(system);
    mavsdk::Telemetry telemetry(system);

    // Declare variables outside the switch statement
    mavsdk::Telemetry::GpsGlobalOrigin origin;
    mavsdk::Offboard::Result result;

    while (program_state != ProgramState::Exiting) {
        switch (program_state) {
            case ProgramState::Initializing:
                std::cout << "Reading home position in Global coordinates\n";
                const auto res_and_gps_origin = telemetry.get_gps_global_origin();
                if (res_and_gps_origin.first != mavsdk::Telemetry::Result::Success) {
                    std::cout << "Telemetry failed: " << res_and_gps_origin.first << '\n';
                }
                origin = res_and_gps_origin.second;
                std::cout << "Origin (lat, lon, alt amsl):\n " << origin << '\n';

                std::cout << "Starting Offboard position control in Global coordinates\n";
                result = offboard.start();
                // Initialization
                if (result != mavsdk::Offboard::Result::Success) {
                    std::cout << "Starting Offboard mode failed (" << result << "), exiting." << '\n';
                    program_state = ProgramState::OffboardModeFailed;
                } else {
                    program_state = ProgramState::WaitingForChannel5;
                }
                break;

            case ProgramState::OffboardModeFailed:
                std::cout << "Starting Offboard mode failed, exiting." << std::endl;
                program_state = ProgramState::Exiting;
                break;

            case ProgramState::WaitingForChannel5:
                std::cout << "Waiting for Channel 5 to be switched" << std::endl;
                std::cout << "Channel 5 switched!" << std::endl;
                program_state = ProgramState::Channel5Switched;
                break;

            case ProgramState::Channel5Switched:
                std::cout << "Channel 5 switched!" << std::endl;
                // Add any necessary code for handling Channel 5 switched event
                // Transition to the next state
                program_state = ProgramState::Start;
                break;

            case ProgramState::Start:
                result = offboard.start();
                if (!offboard.is_active()) {
                    std::cout << "Error, Offboard not active" << std::endl;
                }
                std::cout << "Offboard Active" << std::endl;
                program_state = ProgramState::Waypoint1;
                break;

            case ProgramState::Waypoint1:
                // Execute waypoint1
                result = mavsdk::Offboard::set_position_global(40.0084244, -83.0175219, 5.0);
                std::cout << "Moving to waypoint 1" << std::endl;
                program_state = ProgramState::Waypoint2;
                break;

            case ProgramState::Waypoint2:
                // Execute waypoint2
                result = mavsdk::Offboard::set_position_global(40.0085887, -83.0175251, 5.0);
                std::cout << "Moving to waypoint 2" << std::endl;
                program_state = ProgramState::Waypoint3;
                break;

            case ProgramState::Waypoint3:
                // Execute waypoint3
                result = mavsdk::Offboard::set_position_global(40.0085853, -83.0173310, 5.0);
                std::cout << "Moving to waypoint 3" << std::endl;
                program_state = ProgramState::Waypoint4;
                break;

            case ProgramState::Waypoint4:
            // Execute waypoint4
            result = mavsdk::Offboard::set_position_global(40.0084330, -83.0173369, 5.0);
            std::cout << "Moving to waypoint 4" << std::endl;
            program_state = ProgramState::Exiting;
            break;

            case ProgramState::Exiting:
                // Any necessary cleanup code goes here
                result = offboard.stop();
                if (result != Offboard::Result::Success) {
                    std::cout << "Offboard stop failed: " << result << '\n';
                }
                std::cout << "Offboard stopped\n";
                break;
        }
    }

}
            