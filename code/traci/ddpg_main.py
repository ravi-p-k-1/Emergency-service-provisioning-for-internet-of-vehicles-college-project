from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
from ddpg import *
import re
from pprint import pprint
import json
import pickle

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import sumolib


with open('.\\susang\\traci\\env.json','r') as file:
    json_obj = json.load(file)

initial_state=EmergencyVehicleState([],[],[])
action_space=EmergencyVehicleAction()
agent=DQNAgent(initial_state, action_space, epsilon=json_obj['epsilon'], epsilon_decay=json_obj['epsilon_decay'], epsilon_min=json_obj['epsilon_min'], gamma=json_obj['gamma'], batch_size=json_obj['batch_size'], episode=json_obj['episode'], update_target_freq=json_obj['update_target_freq'])
total_reward=0
buffer=[]
experience=[]

json_obj['episode']+=1
json_obj['epsilon']=json_obj['epsilon']*json_obj['epsilon_decay']

if json_obj['epsilon']<json_obj['epsilon_min']:
    json_obj['epsilon']=json_obj['epsilon_min']

with open('.\\susang\\traci\\env.json','w') as file:
    json.dump(json_obj, file, indent=2)

net=sumolib.net.readNet("C:\\Users\\ravip\\OneDrive\\Documents\\SEM-7\\MP\\susang\\final\\final.net.xml")

def run():
    step = 0
    flag=False
    while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime()<360:

        traci.simulationStep()
        current_vehicles=set(traci.vehicle.getIDList())

        for vehID in current_vehicles:
            type=traci.vehicle.getTypeID(vehID)
            if re.search("^emer",type) and vehID=='71':
                emergency_vehicle_id=vehID
                search_radius=75

                emer_speed_data=traci.vehicle.getSpeed(emergency_vehicle_id)
                emer_acc_data=traci.vehicle.getAcceleration(emergency_vehicle_id)
                emer_lane_data=traci.vehicle.getLaneIndex(emergency_vehicle_id)
                new_state=agent.state
                new_state.vehicle_data=[emergency_vehicle_id, emer_speed_data, emer_acc_data, emer_lane_data]
                new_state.neighbour_data=[]

                for vehicle_id in traci.vehicle.getIDList():

                    x1, y1 = traci.vehicle.getPosition(emergency_vehicle_id)
                    x2, y2 = traci.vehicle.getPosition(vehicle_id)
                    distance = traci.simulation.getDistance2D(x1, y1, x2, y2)
                    emer_road_id = traci.vehicle.getRoadID(emergency_vehicle_id)
                    veh_road_id = traci.vehicle.getRoadID(vehicle_id)

                    if distance<=search_radius and vehicle_id!=emergency_vehicle_id and emer_road_id==veh_road_id:
                        veh_speed_data=traci.vehicle.getSpeed(vehicle_id)
                        veh_acc_data=traci.vehicle.getAcceleration(vehicle_id)
                        veh_lane_data=traci.vehicle.getLaneIndex(vehicle_id)
                        new_state.neighbour_data.append([vehicle_id, veh_speed_data, veh_acc_data, veh_lane_data, distance])

                closest_traffic_light_distance=10000000000000
                closest_traffic_light_ID=""

                for traffic_light_id in traci.trafficlight.getIDList():
                    x1, y1 = traci.vehicle.getPosition(emergency_vehicle_id)
                    x2, y2 = net.getNode(traffic_light_id).getCoord()
                    distance = traci.simulation.getDistance2D(x1, y1, x2, y2)
                    if(closest_traffic_light_distance > distance):
                        closest_traffic_light_distance=distance
                        closest_traffic_light_ID=traffic_light_id
                        new_state.traffic_light_data=[closest_traffic_light_ID, closest_traffic_light_distance]

                if closest_traffic_light_distance<50 :

                    vehicle_angle=traci.vehicle.getAngle(emergency_vehicle_id)
                    direction=""

                    # Adjusting the angle interpretation (0 degrees for west)
                    adjusted_angle = (vehicle_angle + 90) % 360

                    # Determine the direction based on the adjusted angle
                    if 45 <= adjusted_angle < 135:
                        direction = "south"
                    elif 135 <= adjusted_angle < 225:
                        direction = "east"
                    elif 225 <= adjusted_angle < 315:
                        direction = "north"
                    else:
                        direction = "west"

                    complete_definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(closest_traffic_light_ID)

                    logic = complete_definition[0]

                    # print('logic object --',vehicle_angle, direction)

                    phase_index=0

                    for i in logic.phases:

                        if direction=='east' and ( i.name=='ew' or i.name=='e' ):
                            traci.trafficlight.setPhase(closest_traffic_light_ID, phase_index)
                            print('phase_changed')
                            break
                        
                        if direction=='west' and ( i.name=='ew' or i.name=='w' ):
                            traci.trafficlight.setPhase(closest_traffic_light_ID, phase_index)
                            print('phase changed')
                            break
                        
                        if direction=='north' and ( i.name=='ns' or i.name=='n' ):
                            traci.trafficlight.setPhase(closest_traffic_light_ID, phase_index)
                            print('phase changed')
                            break
                        
                        if direction=='south' and ( i.name=='ns' or i.name=='s' ):
                            traci.trafficlight.setPhase(closest_traffic_light_ID, phase_index)
                            print('phase changed')
                            break

                        phase_index+=1
                        
                
                agent.update_state(new_state)
                action = agent.select_action()
                # print(action)
                global experience
                experience=agent.reward(action)
                if experience:
                    if len(experience[0].neighbour_data)>=agent.req_neighbour:
                        buffer.append(experience)
                # print(experience)

                if experience:
                    if experience[1]=='L' and action!=-1:
                        traci.vehicle.changeLane(emergency_vehicle_id, agent.state.vehicle_data[3], 5)
                
                if experience:
                    if experience[1]=='R' and action!=-1:
                        traci.vehicle.changeLane(emergency_vehicle_id, agent.state.vehicle_data[3], 5)
                
        if '71' in traci.vehicle.getIDList():
            flag=True
        if flag and not '71' in traci.vehicle.getIDList():
            flag=False
            # print(buffer)
            buffer[len(buffer)-1][4]=True
            agent.save_experience_buffer(buffer, 'buffer.pkl')

        if experience and '71' in traci.vehicle.getIDList():
            if len(experience)>0:
                agent.train(experience)
        


        step += 1

    weights_data={
        'q_network_weights': agent.q_network.get_weights(),
        'target_network_weights': agent.target_network.get_weights()
    }
        
    with open('.\\susang\\traci\\weights.pkl','wb') as file:
        pickle.dump(weights_data, file)

    print('reward ',agent.total_reward)
    
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":

    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    traci.start([sumoBinary, "-c", "C:\\Users\\ravip\\OneDrive\\Documents\\SEM-7\\MP\\susang\\final\\final.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])
    run()

    # print(buffer)




