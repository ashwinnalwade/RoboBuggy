package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;
import java.util.Date;

/**
 * Plans a path based on a set of waypoints
 */
public class WayPointFollowerPlanner extends PathPlannerNode {
    private ArrayList<GpsMeasurement> wayPoints;
    private GPSPoseMessage pose; //TODO change this to a reasonable type
    private int lastClosestIndex = 0;
    private GpsMeasurement target;

    // we only want to look at the next 10 waypoints as possible candidates
    private static final int WAY_POINT_LOOKAHEAD_MAX = 50;

    /**
     * @param wayPoints the list of waypoints to follow
     */
    public WayPointFollowerPlanner(ArrayList<GpsMeasurement> wayPoints) {
        super(NodeChannel.PATH_PLANNER);
        this.wayPoints = wayPoints;
        target = wayPoints.get(0);
        pose = new GPSPoseMessage(new Date(0), 0, 0, 0);// temp measurment
    }

    @Override
    public void updatePositionEstimate(GPSPoseMessage m) {
        pose = m;

    }

    @Override
    protected GpsMeasurement getTargetWaypoint() {
        return target;
    }

    //find the closest way point
    private int getClosestIndex(ArrayList<GpsMeasurement> wayPoints, GPSPoseMessage currentLocation) {
        double min = Double.MAX_VALUE; //note that the brakes will definitely deploy at this

        int closestIndex = lastClosestIndex;
        for (int i = lastClosestIndex; i < (lastClosestIndex + WAY_POINT_LOOKAHEAD_MAX) && i < wayPoints.size(); i++) {
            GPSPoseMessage gpsPoseMessage = wayPoints.get(i).toGpsPoseMessage(0);
            double d = GPSPoseMessage.getDistance(currentLocation, gpsPoseMessage);
            if (d < min) {
                min = d;
                closestIndex = i;
            }
            // eventually cut off search somehow
        }
        lastClosestIndex = closestIndex;
        return closestIndex;
    }

    @Override
    public double getCommandedSteeringAngle() {
        // determines the angle at which to move every 50 milliseconds
        //PD control of DC steering motor handled by low level
        
        double commandedAngle;
        commandedAngle = purePursuitController();

        return commandedAngle;
    }

    private double purePursuitController() {
        // https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
        // section 2.2

        int closestIndex = getClosestIndex(wayPoints, pose);

        double k = 2.5;
        double velocity = pose.getVelocity();
        double lookaheadLowerBound = 5.0;
        double lookaheadUpperBound = 25.0;
        double lookahead = (k * velocity)/2;
        if(lookahead < lookaheadLowerBound) {
            lookahead = lookaheadLowerBound;
        }
        else if(lookahead > lookaheadUpperBound) {
            lookahead = lookaheadUpperBound;
        }

        //pick the first point that is at least lookahead away, then point buggy toward it
        int lookaheadIndex = 0;
        for(lookaheadIndex = closestIndex; lookaheadIndex < wayPoints.size(); lookaheadIndex++) {
            if(GPSPoseMessage.getDistance(pose, wayPoints.get(lookaheadIndex).toGpsPoseMessage(0)) > lookahead) {
                break;
            }
        }

        //if we are out of points then just go straight
        if (lookaheadIndex >= wayPoints.size()) {
            new RobobuggyLogicNotification("HELP out of points", RobobuggyMessageLevel.EXCEPTION);
            return 0;
        }

        //find a path from our current location to that point
        GpsMeasurement target = wayPoints.get(lookaheadIndex);
        double dx = LocalizerUtil.convertLonToMeters(target.getLongitude()) - LocalizerUtil.convertLonToMeters(pose.getLongitude());
        double dy = LocalizerUtil.convertLatToMeters(target.getLatitude()) - LocalizerUtil.convertLatToMeters(pose.getLatitude());
        double deltaHeading = Math.atan2(dy, dx) - pose.getHeading();

        //Pure Pursuit steering controller
        double commandedAngle = Math.atan2(2 * RobobuggyKFLocalizer.WHEELBASE_IN_METERS * Math.sin(deltaHeading), lookahead);
        commandedAngle = Util.normalizeAngleRad(commandedAngle);
        return commandedAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        // need to brake when 15 m off course
        return false;
    }


}
