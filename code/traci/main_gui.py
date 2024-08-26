

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    p0 = 1. / 10
    p1 = 1. / 7
    p2 = 1. / 5
    p3 = 1. / 7
    p4 = 1. / 10
    emer = 1. / 50
    with open("E:/nirma/sem-7/minor_project/practice/data/simple.rou.xml", 'w+') as routes:
        print("""<routes>
        <vType id="normal" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="8.67" guiShape="passenger"/>
        <vType id="emergency" accel="1.0" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="16" guiShape="bus" color="1,0,0"/>
        <route id="R0" edges="E4" lane="E4_2" />
        """, file=routes)

        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < p0:
                print('    <vehicle id="p0_%i" type="normal" depart="%i" departLane="0" route="R0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p1:
                print('    <vehicle id="p1_%i" type="normal" depart="%i" departLane="1" route="R0"/>' % (
                    vehNr, i), file=routes)
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
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
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

    generate_routefile()
    traci.start([sumoBinary, "-c", "E:/nirma/sem-7/minor_project/practice/data/simple.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])
    run()
