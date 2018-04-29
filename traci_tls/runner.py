#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import math

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci

TIMEOUT = 60
R = 100

def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pWE = 1. / 10
    pEW = 1. / 11
    pNS = 1. / 30
    pSN = 1. / 30
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="up" edges="53o 3i 4o 54i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pSN:
                print('    <vehicle id="up_%i" type="typeNS" route="up" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


    
def calc_dist(active_list, direction = None):
   
    distances = []
    for car in active_list:
        pos = traci.vehicle.getPosition(car)
        dist = math.sqrt((pos[0] - 510)**2 + (pos[1] - 510)**2) 
            ##print(car,pos,dist)
        try:
            
            distances.append((car, dist))
            #pos = traci.simulation.getPosition(car)
            #distances2.append((car, (pos[0], pos[1])))
        except Exception:
            print("exception happened")
            continue

    #print(distances2)    
    distances = sorted(distances,key=lambda x:x[1])
    print("__________________")

    if direction == None:
        return distances[0]
    elif direction == 'UD':
        #return distances
        return next(x for x in distances if "left" in x[0] or "right" in x[0])
    elif direction == 'LR':
        #return distances
        return next(x for x in distances if "up" in x[0] or "down" in x[0])

    
    
def elect_leader(active_list):
    return calc_dist(active_list)[0]
    
def run():
    """execute the TraCI control loop"""
    step = 0

    active_list = set()

    leader_id = None

    bootstrap = True
    timeout = TIMEOUT
    
    # we start with phase 2 where EW has green
    traci.trafficlight.setPhase("0", 1)

    #print(traci.simulation.getPosition("0"))
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        ##active list generation##
        active_list = active_list.union(set(traci.simulation.getDepartedIDList()))
        arrived_list = set(traci.simulation.getArrivedIDList())
        for car in active_list.intersection(arrived_list):
            active_list.remove(car)

        if calc_dist(active_list)[1] < R and bootstrap:
            leader_id = calc_dist(active_list)[0]
            bootstrap = False
            #######
            #Uncomment this to change colour (I think)
            #######
            #traci.vehicle.setColor(leader_id, (250, 0, 0))
            print(leader_id)
            
        if not bootstrap:
            ## calculate perpendicular closest car distance
            perp = 0
            if "up" in leader_id or "down" in leader_id:
                ##find smallest dist for left/right
                perp = calc_dist(active_list, "UD")[1]
            else:
                perp = calc_dist(active_list, "LR")[1]
                ##find smallest dist for up/down    

            ## check if new leader time
            if perp > R or timeout <= 0:
                if "up" in leader_id or "down" in leader_id:
                    leader_id = calc_dist(active_list,"UD")[0]
                    traci.trafficlight.setPhase("0", 0)
                    #######
                    #Uncomment this to change colour (I think)
                    #######
                    #traci.vehicle.setColor(leader_id, (250, 0, 0))
                else:
                    leader_id = calc_dist(active_list,"LR")[0]
                    traci.trafficlight.setPhase("0", 1)
                    #######
                    #Uncomment this to change colour (I think)
                    #######
                    #traci.vehicle.setColor(leader_id, (250, 0, 0))

                timeout = TIMEOUT
                print(leader_id)
        '''
        if len(active_list) > 0 and leader == None:
            leader_id = elect_leader(active_list)
        '''
            
        '''
        if traci.trafficlight.getPhase("0") == 1:
            # we are not already switching
            if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
                # there is a vehicle from the north, switch
                traci.trafficlight.setPhase("0", 0)
            else:
                # otherwise try to keep green for EW
                traci.trafficlight.setPhase("0", 1)
        '''
        step += 1
        timeout -= 1
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
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
