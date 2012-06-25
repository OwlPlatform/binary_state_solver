#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <time.h>
#include <utility>
#include <vector>
#include <sstream>

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

struct Debug {
  bool on;
};

template<typename T>
Debug& operator<<(Debug& dbg, T arg) {
  if (dbg.on) {
    std::cout<<arg;
  }
  return dbg;
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
    std::cout<< "arguments: aggregator agg_solver worldmodel wm_solver wm_client config_file\n";
    std::cout<< "description: Monitors status of simple on/off switches.\n";
    std::cout<< "requires: sensor\\.switch\n";
    std::cout<< "config_file: tuple 1 Region name.\n";
    std::cout<< "config_file: tuple* 2 An item class (eg. door) and the name for its state when the switch is on (eg. closed).\n";
    return 0;
  }

  if (7 > arg_count or (arg_count % 2) == 0) {
    std::cerr<<"This program needs 6 or more arguments:\n";
    std::cerr<<"\t"<<arg_vector[0]<<" [<aggregator ip> <aggregator port>]+ <world model ip> <solver port> <client port> <config file>\n\n";
    std::cerr<<"Any number of server ip/port pairs may be provided to connect to multiple servers.\n";
    std::cerr<<"Lines in the config file consist of the object class to track and the name of its switch status.\n";
    std::cerr<<"For instance: \"doors\" \"closed\"\n";
    return 0;
  }

  //Grab the ip and ports for the aggregators and world model
  std::vector<SolverAggregator::NetTarget> servers;
  for (int s_num = 1; s_num < arg_count - 4; s_num += 2) {
    std::string server_ip(arg_vector[s_num]);
    uint16_t server_port = std::stoi(std::string(arg_vector[s_num + 1]));
    servers.push_back(SolverAggregator::NetTarget{server_ip, server_port});
  }

  //World model IP and ports
  std::string wm_ip(arg_vector[arg_count - 4]);
  int solver_port = std::stoi(std::string((arg_vector[arg_count - 3])));
  int client_port = std::stoi(std::string((arg_vector[arg_count - 2])));

  //Set up the solver world model connection;
  std::string origin = "grail/switch_solver\nversion 1.0";

  //Read in type information from the config file
  //Types for the GRAIL world model will be read from the file
  std::vector<std::pair<std::u16string, bool>> type_pairs;

  //Remember what names correspond to what solutions and build a
  //query to find all objects of interest.
  //Use the object to solution map to map transmitters to URIs
  std::map<std::u16string, std::u16string> object_to_solution;
  std::map<pair<uint8_t, uint128_t>, URI> tx_to_uri;
  std::mutex tx_to_uri_mutex;

  std::ifstream config(arg_vector[arg_count-1]);
  if (not config.is_open()) {
    std::cerr<<"Error opening configuration file \""<<arg_vector[arg_count-1]<<"\"\n";
    return 1;
  }
  std::string line;
  //Read in each type and its corresponding name
  while (std::getline(config, line)) {
    std::istringstream is(line);
    std::string obj_class, solution;
    if (is >> obj_class >> solution) {
      //Each switch value is a single byte for on or off but the
      //solution name for each object class is different.
      //Safe this as a non-transient solution type
      //Change underlines into spaces.
      std::transform(obj_class.begin(), obj_class.end(), obj_class.begin(),
          [&](char c) { return c == '_' ? ' ' : c;});
      type_pairs.push_back(std::make_pair(toU16(solution), false));
      std::cerr<<"Class \""<<obj_class<<"\" has solution name \""<<solution<<'\n';
      object_to_solution[toU16(obj_class)] = toU16(solution);
    }
    else {
      std::cerr<<"Couldn't make sense of line: \""<<line<<"\"\n";
    }
  }
  config.close();

  if (object_to_solution.empty()) {
    std::cerr<<"There are no types in the config file - aborting.\n";
    return 1;
  }

  SolverWorldModel swm(wm_ip, solver_port, type_pairs, toU16(origin));
  if (not swm.connected()) {
    std::cerr<<"Could not connect to the world model as a solver - aborting.\n";
    return 0;
  }

  ClientWorldConnection cwc(wm_ip, client_port);

  //Search for any IDs with names <anything>.obj_class.<anything>
  URI desired_ids = u".*\\.";
  if (object_to_solution.size() == 1) {
    desired_ids = u".*\\." + object_to_solution.begin()->first + u"\\..*";
  }
  else {
    desired_ids += u"(";
    for (auto I = object_to_solution.begin(); I != object_to_solution.end(); ++I) {
      //Insert a | (OR in regex) into the regex if this isn't the first object
      if (I != object_to_solution.begin()) {
        desired_ids += u"|";
      }
      desired_ids += I->first;
    }
    desired_ids += u")";
  }
  desired_ids += u"\\..*";
  //Search for any matching IDs with switch sensors
  std::vector<URI> attributes{u"sensor.switch.*"};
  //Update at most once a second
  world_model::grail_time interval = 1000;
  StepResponse sr = cwc.streamRequest(desired_ids, attributes, interval);

  //Subscription rules for the GRAIL aggregator
  //The transmitters to request will be discovered from the world model
  //We need to remember what transmitters we've already requested over here.
  std::map<uint8_t, Rule> phy_to_rule;
  std::map<URI, bool> switch_state;

  //Connect to the aggregator and update it with new rules as the world model
  //provided transmitters of interest
  auto packet_callback = [&](SampleData& s) {
    if (s.valid and 1 == s.sense_data.size()) {
      int switch_value = readPrimitive<uint8_t>(s.sense_data, 0);
      if (0 == switch_value or 255 == switch_value) {
        URI uri;
        {
          std::unique_lock<std::mutex> lck(tx_to_uri_mutex);
          uri = tx_to_uri[std::make_pair(s.physical_layer, s.tx_id)];
        }
        bool switch_on = 255 == switch_value;
        if (switch_state.end() == switch_state.find(uri) or switch_state[uri] != switch_on) {
          switch_state[uri] = switch_on;
          //Use the object to solution map to get the solution name.
          for (auto obj_soln = object_to_solution.begin(); obj_soln != object_to_solution.end(); ++obj_soln) {
            if (uri.find(u"." + obj_soln->first + u".") != std::u16string::npos) {
              SolverWorldModel::AttrUpdate soln{obj_soln->second, world_model::getGRAILTime(), uri, std::vector<uint8_t>()};
              pushBackVal<uint8_t>(switch_on ? 1 : 0, soln.data);
              std::vector<SolverWorldModel::AttrUpdate> solns{soln};
              //Send the data to the world model
              swm.sendData(solns, false);
              if (switch_on) {
                std::cout<<toString(uri)<<" is "<<toString(obj_soln->second)<<'\n';
              } else {
                std::cout<<toString(uri)<<" is not "<<toString(obj_soln->second)<<'\n';
              }
            }
          }
        }
      }
    }
  };
  SolverAggregator aggregator(servers, packet_callback);

  //Less than operator for two world model attributes
  auto attr_comp = [](const Attribute& a, const Attribute& b) {
    return (a.expiration_date != 0 or a.creation_date < b.creation_date); };
  std::cerr<<"Starting loop...\n";
  while (sr.hasNext()) {
    //Remember if we need to ask the aggregator for more information.
    bool new_transmitters = false;
    //Get world model updates
    world_model::WorldState ws = sr.next();
    //Check each object for switch sensor ID information
    for (auto I : ws) {
      Attribute newest = *(std::max_element(I.second.begin(), I.second.end(), attr_comp));
      if (newest.expiration_date != 0) {
        //TODO FIXME This attribute has been expired so stop updating the
        //status of this ID in the world model
      }
      //Otherwise, make sure that we have signed up for this sensor's data
      //from the aggregators

      //Transmitters are stored as one byte of physical layer and 16 bytes of ID
      grail_types::transmitter tx_switch = grail_types::readTransmitter(newest.data);
      if (phy_to_rule.find(tx_switch.phy) == phy_to_rule.end()) {
        phy_to_rule[tx_switch.phy].physical_layer  = tx_switch.phy;
        phy_to_rule[tx_switch.phy].update_interval = 1000;
      }
      //See if this is a new transmitter
      auto tx_present = [&](const Transmitter& tx) {return tx.base_id == tx_switch.id;};

      std::vector<Transmitter>& phy_txers = phy_to_rule[tx_switch.phy].txers;
      if (phy_txers.end() == std::find_if(phy_txers.begin(), phy_txers.end(), tx_present)) {
        //Mark that we are making a change to the aggregator rules.
        new_transmitters = true;
        //Only accept data from sensors that we care about
        Transmitter sensor_id;
        sensor_id.base_id = tx_switch.id;
        sensor_id.mask.upper = 0xFFFFFFFFFFFFFFFF;
        sensor_id.mask.lower = 0xFFFFFFFFFFFFFFFF;
        phy_txers.push_back(sensor_id);
        //Also map this transmitter's ID to the object it corresponds to in the world model
        {
          std::unique_lock<std::mutex> lck(tx_to_uri_mutex);
          tx_to_uri[std::make_pair(tx_switch.phy, tx_switch.id)] = I.first;
        }
      }
    }
    //Make new subscriptions to the aggregator if there are new transmitters
    if (new_transmitters) {
      Subscription sub;
      for (auto phy_rule : phy_to_rule) {
        sub.push_back(phy_rule.second);
      }
      aggregator.addRules(sub);
    }
  }
}

