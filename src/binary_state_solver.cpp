/*
 * Copyright (c) 2012 Bernhard Firner and Rutgers University
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 * or visit http://www.gnu.org/licenses/gpl-2.0.html
 */

/*******************************************************************************
 * @file binary_state_solver.cpp
 * Offer binary solutions to the world model based upon binary sensors,
 * such as door switches, on/off power switches, etc.
 *
 * @author Bernhard Firner
 ******************************************************************************/

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <time.h>
#include <unistd.h>
#include <utility>
#include <vector>
#include <sstream>

//Handle interrupt signals to exit cleanly.
#include <signal.h>

#include <owl/netbuffer.hpp>
#include <owl/solver_aggregator_connection.hpp>
#include <owl/solver_world_connection.hpp>
#include <owl/sample_data.hpp>
#include <owl/world_model_protocol.hpp>
#include <owl/grail_types.hpp>

#include <owl/client_world_connection.hpp>

using namespace aggregator_solver;

using std::pair;

using world_model::Attribute;
using world_model::grail_time;
using world_model::URI;

//Global variable for the signal handler.
bool interrupted = false;
//Signal handler.
void handler(int signal) {
  psignal( signal, "Received signal ");
  if (interrupted) {
    std::cerr<<"Aborting.\n";
    // This is the second time we've received the interrupt, so just exit.
    exit(-1);
  }
  std::cerr<<"Shutting down...\n";
  interrupted = true;
}

std::u16string toU16(const std::string& str) {
  return std::u16string(str.begin(), str.end());
}

std::string toString(const std::u16string& str) {
  return std::string(str.begin(), str.end());
}

int main(int arg_count, char** arg_vector) {
  if (arg_count == 2 and std::string(arg_vector[1]) == "-?") {
    std::cout<< "name: Switch Solver\n";
    std::cout<< "arguments: worldmodel wm_solver wm_client config_file\n";
    std::cout<< "description: Monitors status of simple on/off switches.\n";
    std::cout<< "requires: 'binary state'\n";
    return 0;
  }

  if (4 > arg_count) {
    std::cerr<<"This program needs 4 arguments:\n";
		std::cerr<<"\t"<<arg_vector[0]<<" <world model ip> <solver port> <client port>\n\n";
		std::cerr<<"This solver uses binary data from objects with attributes named ";
		std::cerr<<"'sensor.door' and 'sensor.water'.\n";
		std::cerr<<"An optional 4th argument may be an integer specifying the number of times a binary value\n";
	  std::cerr<<"must be observed before a state change occurs. Use this to combat packet errors. Try setting\n";
	  std::cerr<<"this to one less than the expected number of receivers that can see a transmitter's packet.\n";
    return 0;
  }

  //Set up a signal handler to catch interrupt signals so we can close gracefully
  signal(SIGINT, handler);  

  //World model IP and ports
  std::string wm_ip(arg_vector[1]);
  int solver_port = std::stoi(std::string((arg_vector[2])));
  int client_port = std::stoi(std::string((arg_vector[3])));
	int transition_threshold = 1;
	if (arg_count == 5) {
		transition_threshold = std::stoi(std::string((arg_vector[4])));
		std::cerr<<"Using a transition threshold of "<<transition_threshold<<'\n';
	}

  //Set up the solver world model connection;
  std::string origin = "binary_state_solver";

  //Remember what names correspond to what solutions and build a
  //query to find all objects of interest.
  //Use the object to solution map to map transmitters to URIs
  std::map<std::u16string, std::u16string> object_to_solution;
	object_to_solution[u"sensor.door"] = u"closed";
	object_to_solution[u"sensor.water"] = u"wet";

	//Map of transmitter URI (with binary data type) to object URIs.
  std::map<URI, URI> tx_to_uri;
  std::map<URI, std::u16string> tx_to_solution;

  //Solution types for the world model.
  std::vector<std::pair<std::u16string, bool>> solution_types{{u"closed", false}, {u"wet", false}};

  std::cerr<<"Trying to connect to world model as a solver.\n";
  SolverWorldModel swm(wm_ip, solver_port, solution_types, toU16(origin));
  if (not swm.connected()) {
    std::cerr<<"Could not connect to the world model as a solver - aborting.\n";
    return 0;
  }

	//Remember switch states so that we only update when something changes
  std::map<URI, bool> switch_state;

  //Search for sensor attributes of any matching IDs
	URI desired_ids = u".*";
  std::vector<URI> attributes{u"sensor.(door|water)"};

	//Update IDs one a second
	world_model::grail_time interval = 1000;

	//Set up parameters for a second request of 'binary state' data.
	//Update as data arrives (interval of 0).
	URI binary_ids = u".*";
	std::vector<URI> binary_attributes{u"binary state"};
	world_model::grail_time binary_interval = 0;

	//We will connect to the world model as a client inside of the processing loop below
	//Whenever we are disconnected we will attempt to reconnect.

  std::cerr<<"Trying to connect to world model as a client.\n";
  ClientWorldConnection cwc(wm_ip, client_port);
	//Send out the requests
	StepResponse sr = cwc.streamRequest(desired_ids, attributes, interval);
	StepResponse binary_response = cwc.streamRequest(binary_ids, binary_attributes, binary_interval);

	std::cerr<<"Starting processing loop...\n";
  while (not interrupted) {
		//Less than operator for two world model attributes
		auto attr_comp = [](const Attribute& a, const Attribute& b) {
			return (a.expiration_date != 0 or a.creation_date < b.creation_date); };
		//Stay connected
    while (not cwc.connected() and not interrupted) {
      std::cerr<<"Waiting 4 seconds before attempting to reconnect client->world model connection\n";
      //Sleep for several seconds after an error before trying to reconnect
      sleep(4);
      cwc.reconnect();
			if (cwc.connected()) {
				//Re-send out the requests
				StepResponse sr = cwc.streamRequest(desired_ids, attributes, interval);
				StepResponse binary_response = cwc.streamRequest(binary_ids, binary_attributes, binary_interval);
			}
    }

		try {

			//Now process the on-demand binary data
			while (binary_response.hasNext() and not interrupted) {
				//Get world model updates
				world_model::WorldState ws = binary_response.next();
				//Check each object for new switch states
				for (const std::pair<URI, std::vector<Attribute>>& I : ws) {
					if (tx_to_uri.end() != tx_to_uri.find(I.first)) {
						URI uri = tx_to_uri[I.first];
						std::u16string soln_name = tx_to_solution[I.first];
						//Get the first byte of the data (will be a one byte binary value)
						bool switch_on = I.second[0].data.at(0);
						//Update if this is new or if the value changed
						if (switch_state.end() == switch_state.find(uri) or switch_state[uri] != switch_on) {
							switch_state[uri] = switch_on;
							//Use the object to solution map to get the solution name.
							SolverWorldModel::AttrUpdate soln{soln_name, world_model::getGRAILTime(), uri, std::vector<uint8_t>()};
							pushBackVal<uint8_t>(switch_on ? 1 : 0, soln.data);
							std::vector<SolverWorldModel::AttrUpdate> solns{soln};
							//Send the data to the world model
							bool retry = true;
							while (retry) {
								try {
									retry = false;
									swm.sendData(solns, false);
								}
								catch (std::runtime_error& err) {
									//Retry if this is just a temporary socket error
									if (err.what() == std::string("Error sending data over socket: Resource temporarily unavailable")) {
										std::cerr<<"Experiencing socket slow down with world model connection. Retrying...\n";
										retry = true;
									}
									//Otherwise keep throwing
									else {
										throw err;
									}
								}
							}
							if (switch_on) {
								std::cout<<toString(uri)<<" is "<<toString(soln_name)<<'\n';
							} else {
								std::cout<<toString(uri)<<" is not "<<toString(soln_name)<<'\n';
							}
						}
					}
				}
			}
			//Check for responses to map sensors to object identifiers
			while (sr.hasNext() and not interrupted) {
				std::cerr<<"Got sensor name data\n";
				//Get world model updates
				world_model::WorldState ws = sr.next();
				//Check each object for switch sensor ID information
				for (const std::pair<URI, std::vector<Attribute>>& I : ws) {
					if (I.second.empty()) {
						std::cerr<<toString(I.first)<<" is an empty object.\n";
					}
					else {
						Attribute newest = *(std::max_element(I.second.begin(), I.second.end(), attr_comp));

						//Transmitters are stored as one byte of physical layer and 16 bytes of ID
						grail_types::transmitter tx_switch = grail_types::readTransmitter(newest.data);
						std::string str = std::to_string(tx_switch.phy) + "." + std::to_string(tx_switch.id.lower);
						std::u16string tx_str = std::u16string(str.begin(), str.end());
						if (newest.expiration_date != 0) {
							//This attribute has been expired so stop updating the
							//status of this ID in the world model
							tx_to_uri.erase(tx_str);
							tx_to_solution.erase(tx_str);
						}
						else {
							//Otherwise, make sure that we have signed up for this sensor's data
							//from the aggregators

							//Map this transmitter to the ID of the object it corresponds to in the world model
							tx_to_uri[tx_str] = I.first;
							//Map this transmitter ID to a solution type from its attribute name
							tx_to_solution[tx_str] = object_to_solution[newest.name];
							std::cerr<<"Adding "<<std::string(I.first.begin(), I.first.end())<<" into object map with transmitter "<<std::string(str)<<"\n";
						}
					}
				}
			}
		}
		catch (std::runtime_error& err) {
			std::cerr<<"Error in client->world model connection: "<<err.what()<<'\n';
		}
  }
}

