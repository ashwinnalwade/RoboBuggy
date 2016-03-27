package com.roboclub.robobuggy.robots;

import java.io.IOException;
import java.util.ArrayList;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.localizers.KfLocalizer;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.simulation.SimulatedBuggy;
import com.roboclub.robobuggy.simulation.SimulatedGPSNode;
import com.roboclub.robobuggy.simulation.SimulatedRBSMNode;
import com.roboclub.robobuggy.simulation.SimulationPlayer;

/**
 * A robot file for a simulated robot that can be used for internal testing of nodes along simulated paths 
 * @author Trevor Decker
 *
 */
public final class SimRobot extends AbstractRobot{
    private static SimRobot instance;
	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance =  new SimRobot();
		}
		return instance;
	}
	
	private SimRobot(){
		super();
	
		nodeList.add(new HighTrustGPSLocalizer());
		nodeList.add(new SimulatedGPSNode());
		nodeList.add(new SimulatedRBSMNode());
		ArrayList<GpsMeasurement> wayPoints = new ArrayList<GpsMeasurement>();
		for(int i = 0;i<100;i++){
			wayPoints.add(new GpsMeasurement(i,0));
		}
		nodeList.add(new WayPointFollowerPlanner(wayPoints));
		SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();
		simBuggy.setDx(.1);
		//simBuggy.setDth(1);
		//simBuggy.setDth(1.0);

	}
}