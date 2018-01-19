package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.simulation.FullSimRunner;
import com.roboclub.robobuggy.ui.*;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;

import static com.roboclub.robobuggy.main.RobobuggyConfigFile.INITIAL_POS_LAT;
import static com.roboclub.robobuggy.main.RobobuggyConfigFile.INITIAL_POS_LON;


/**
 * A robot file for a simulated robot that can be used for internal testing of nodes along simulated paths
 *
 * @author Trevor Decker
 */
public final class XmitSimRobot extends AbstractRobot {
    private static XmitSimRobot instance;

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     * If no instance exists, a new one is created.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    public static AbstractRobot getInstance() {
        if (instance == null) {
            instance = new XmitSimRobot();
        }
        return instance;
    }

    private XmitSimRobot() {
        super();

        nodeList.add(new FullSimRunner("Full Sim Toolbox", new LocTuple(INITIAL_POS_LAT, INITIAL_POS_LON)));

        try {
            Socket s = new Socket("192.168.1.202", 8080);
            PrintWriter out = new PrintWriter(s.getOutputStream());

            new Subscriber("xmit", NodeChannel.POSE.getMsgPath(), (topicName, m) -> {
                GPSPoseMessage msg = ((GPSPoseMessage) m);
                String xmit = msg.getLatitude() + "," + msg.getLongitude();
                out.write(xmit);
                out.flush();
            });
        } catch (IOException e) {
            e.printStackTrace();
        }

        //setup the gui
        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        tabs.addTab(new MainGuiWindow(), "Home");
        tabs.add(new PathPanel(), "Path Visualizer");
        tabs.addTab(new ConfigurationPanel(), "Configuration");

    }
}
