    ////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                        //
  //           Set up a client to allow manual prompts with the command server              //
 //                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////

#include <CommandInterface.h>                                                                       // Definition of thrift interface
#include <iostream>                                                                                 // std::cerr, std::cout
#include <yarp/os/Network.h>                                                                        // Load configuration files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

int main(int argc, char* argv[])
{
	yarp::os::Network yarp; 
	
	// Connect this client port with the command server       
	yarp::os::Port clientPort; 
	
	if(not clientPort.open("/commandClient"))
	{
		std::cerr << "[ERROR] [COMMAND PROMPT] Could not open port under the name '/commandClient' \n";
		return 1;
	}
	
	if(not yarp.connect("/commandClient", "/commandServer"))
	{
		std::cerr << "[ERROR] [COMMAND PROMPT] Could not connect '/commandClient' with '/commandServer'.\n";
		return 1;
	}
	
	CommandInterface client;
	client.yarp().attachAsClient(clientPort);                                                   // As it says on the label
	
	// Set up an input port here to pass on manual commands
	yarp::os::RpcServer promptPort;                                                             // Create a port for sending / receiving info
	promptPort.open("/commandPrompt");                                                          // Open the port with the name '/command'
	yarp::os::Bottle input;                                                                     // Store information from the user input
	yarp::os::Bottle output;                                                                    // Store information to send to the user
	
	bool active = true;
	
	while(active)
	{
		output.clear();                                                                     // Clear any previous output
		promptPort.read(input,true);                                                        // Read the port for manual commands
		
		std::string command = input.toString();                                             // Convert to string
		
		if(command == "close")
		{
			output.addString("Arrivederci");
			active = false;
		}
		else if(command == "home")
		{
			if(client.move_to_named_configuration(command))
			{
				output.addString("Casa");
			}
			else	output.addString("Problema");
		}
		else if(command == "ready")
		{
			if(client.move_to_named_configuration(command))
			{
				output.addString("Pronto");
			}
			else	output.addString("Problema");
		}	
		else
		{
			output.addString("Cosa");
		}
			
		promptPort.reply(output);
	}
	
	return 0;
}
