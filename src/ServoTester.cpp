#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <iostream>
#include <thread>



#define PORT_PATH "serial:///dev/ttyTHS0"

using namespace mavsdk;

int main(){
    int index = 3;
    float value = -1.0;
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};

    ConnectionResult conn_result = mavsdk.add_any_connection(PORT_PATH);
    // Wait for the system to connect via heartbeat
     while (mavsdk.systems().size() == 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "Stopped";
   	}
    // System got discovered.

	auto systems = mavsdk.systems();
	
	if(systems.empty()){
		std::cout << "No Mavlink system found!";	
	}
	
	std::shared_ptr<mavsdk::System> system = systems[0];

	std::cout << "System found";	

    auto action = Action{system};
	
	std::cout << "Action Found";
       
        //for(int i = 1; i < 8; i++){
    	Action::Result servo_result = action.set_actuator(index,  value);
       
	
    if(servo_result !=  Action::Result::Success){
	std::cout << "Action Failed";
	
	}

    //std::cout << i << "'th Result: " << servo_result;
       //}

}


