#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2009-2023 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import re
import traci.constants as tc

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

buffer=[]


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    p0 = 1. / 10
    p1 = 1. / 7
    p2 = 1. / 5
    p3 = 1. / 7
    p4 = 1. / 10
    emer = 1. / 50
    with open("C:\\Users\\ravip\\OneDrive\\Documents\\SEM-7\\MP\\susang\\final\\trips.trips.xml", 'w+') as routes:
        print("""<routes>
        <vType id="normal" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="8.67" guiShape="passenger"/>
        <vType id="emergency" accel="1.0" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="16" guiShape="bus" color="1,0,0"/>
        <route id="R0" edges="E4" lane="E4_2" />
        """, file=routes)

        # <route id="left" edges="52o 2i 1o 51i" />
        # <route id="down" edges="54o 4i 3o 53i" />
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < p0:
                print('    <vehicle id="p0_%i" type="normal" depart="%i" departLane="0" route="R0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p1:
                print('    <vehicle id="p1_%i" type="normal" depart="%i" departLane="1" route="R0"/>' % (
                    vehNr, i), file=routes)
                # print('    changeLane("p1_%i" , 5 , 3600)' % (
                # vehNr), file=routes)
                # traci.vehicle.changeLane("p1_%i".format(vehNr),5 , 3600)
                vehNr += 1
            if random.uniform(0, 1) < p2:
                print('    <vehicle id="p2_%i" type="normal" depart="%i" departLane="2" route="R0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p3:
                print('    <vehicle id="p3_%i" type="normal" depart="%i" departLane="3" route="R0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p4:
                print('    <vehicle id="p4_%i" type="normal" depart="%i" departLane="4" route="R0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < emer:
                print('    <vehicle id="emer_%i" type="emergency" depart="%i" departLane="2" route="R0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1

        print("</routes>", file=routes)



def run():
    """execute the TraCI control loop"""
    step = 0
    flag=False
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    previous_vehicles = set()
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        current_vehicles = set(traci.vehicle.getIDList()) #RAVI COMM why did you make it a set when it is already a list?
        new_vehicles = current_vehicles - previous_vehicles

        # To search for emergency vehicle
        for vehID in traci.vehicle.getIDList():
            type = traci.vehicle.getTypeID(vehID)
            if re.search("^emer", type):
                # print(vehID)
                buffer.append([1,False])
                reference_vehicle_id = vehID
                # If we get one then search for vehicles in range search_radius
                lane = -1
                search_radius = 75
                # traci.vehicle.getNeighbors()
                nearby_vehicles = traci.vehicle.getNeighbors(reference_vehicle_id, 2)
                # print(nearby_vehicles)
                for vehicle_id in traci.vehicle.getIDList():
                    # if vehicle_id != reference_vehicle_id:
                    # print(vehicle_id)
                    x1, y1 = traci.vehicle.getPosition(reference_vehicle_id)
                    x2, y2 = traci.vehicle.getPosition(vehicle_id)
                    distance = traci.simulation.getDistance2D(x1, y1, x2, y2)

                    if distance <= search_radius and vehicle_id != reference_vehicle_id:

                        # If same lane change the lane
                        lane_ref = traci.vehicle.getLaneID(reference_vehicle_id)
                        lane_eco = traci.vehicle.getLaneID(vehicle_id)
                        if lane_ref == lane_eco:

                            # Check if we have any vehicle on side
                            left = True
                            right = True
                            # nearby_vehicles = traci.vehicle.getNeighbors(reference_vehicle_id, search_radius)
                            for target_vehicle_id in traci.vehicle.getIDList():
                                target_lane_id = traci.vehicle.getLaneID(target_vehicle_id)
                                reference_lane_id = traci.vehicle.getLaneID(vehicle_id)

                                # If too close
                                tolerance = 30

                                x1, y1 = traci.vehicle.getPosition(target_vehicle_id)
                                x2, y2 = traci.vehicle.getPosition(reference_vehicle_id)
                                distance = traci.simulation.getDistance2D(x1, y1, x2, y2)
                                if target_lane_id != reference_lane_id and distance <= tolerance and target_vehicle_id != vehID:
                                    # print(vehicle_id)
                                    if traci.vehicle.getLaneIndex(target_vehicle_id) < traci.vehicle.getLaneIndex(vehicle_id):
                                        left = False
                                        traci.vehicle.setSpeed(target_vehicle_id, max(traci.vehicle.getSpeed(target_vehicle_id) - 0.5, 5))
                                        traci.vehicle.setSpeed(vehicle_id, min(traci.vehicle.getSpeed(vehicle_id) + 0.5, 14))
                                    else:
                                        right = False
                                        traci.vehicle.setSpeed(target_vehicle_id, max(traci.vehicle.getSpeed(target_vehicle_id) - 0.5, 5))
                                        traci.vehicle.setSpeed(vehicle_id, min(traci.vehicle.getSpeed(vehicle_id) + 0.5, 14))

                            index = traci.vehicle.getLaneIndex(vehicle_id)
                            current_edge = traci.vehicle.getRoadID(vehicle_id)
                            num_lanes = traci.edge.getLaneNumber(current_edge)
                            if left and index - 1 >= 0:
                                traci.vehicle.changeLane(vehicle_id, index-1, 3600)
                            elif right and index + 1 <= num_lanes - 1:
                                traci.vehicle.changeLane(vehicle_id, index + 1, 3600)
        if '71' in traci.vehicle.getIDList():
            flag=True
        if flag and not '71' in traci.vehicle.getIDList():
            flag=False
            buffer[len(buffer)-1][1]=True
            # print(buffer)

        # Get updates for all induction loops
        loop_data = traci.inductionloop.getIDList()
        # print(loop_data)
        # Loop through the data for each induction loop
        for loop_id in loop_data:
            last_vehicles = traci.inductionloop.getLastStepVehicleIDs(loop_id)
            for last_vehicle in last_vehicles:
                last_vehicle_type = traci.vehicle.getTypeID(last_vehicle)
                if last_vehicle_type == "emergency":
                    if loop_id in ("e1_30", "e1_31", "e1_32", "e1_21", "e1_22", "e1_23", "e1_0", "e1_1", "e1_2", "e1_65", "e1_66", "e1_67", "e1_49", "e1_50", "e1_51", "e1_39", "e1_40", "e1_41"):
                        for light in traci.trafficlight.getIDList(): #RAVI COMM what is this?
                            traci.trafficlight.setPhase(light, 1)
                    elif loop_id in ("e1_36", "e1_37", "e1_38", "e1_27", "e1_28", "e1_29", "e1_19", "e1_18", "e1_20", "e1_59", "e1_60", "e1_61", "e1_64", "e1_63", "e1_62", "e1_3", "e1_4", "e1_5"):
                        for light in traci.trafficlight.getIDList():
                            traci.trafficlight.setPhase(light, 3)
                    elif loop_id in ("e1_10", "e1_9", "e1_11", "e1_12", "e1_13", "e1_14", "e1_33", "e1_34", "e1_35", "e1_42", "e1_43", "e1_44", "e1_52", "e1_54", "e1_55", "e1_68", "e1_70", "e1_71"):
                        for light in traci.trafficlight.getIDList():
                            traci.trafficlight.setPhase(light, 5)
                    else:
                        for light in traci.trafficlight.getIDList():
                            traci.trafficlight.setPhase(light, 1)
        # vehicles_in_lane = traci.lane.getLastStepVehicleIDs(lane_id)



        step += 1
        previous_vehicles = current_vehicles
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

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    # generate_routefile()
    traci.start([sumoBinary, "-c", "C:\\Users\\ravip\\OneDrive\\Documents\\SEM-7\\MP\\susang\\final\\final.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])
    # traci.start([sumoBinary, "-c", "E:/nirma/sem-7/minor_project/final/final.sumocfg",
                

    run()
    # subprocess and then the python script connects and runs
    # this is the normal way of using traci. sumo is started as a

