package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.simulation.FullSimRunner;
import com.roboclub.robobuggy.ui.*;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Date;

import static com.roboclub.robobuggy.main.RobobuggyConfigFile.INITIAL_POS_LAT;
import static com.roboclub.robobuggy.main.RobobuggyConfigFile.INITIAL_POS_LON;


/**
 * A robot file for a simulated robot that can be used for internal testing of nodes along simulated paths
 *
 * @author Trevor Decker
 */
public final class RecvSimRobot extends AbstractRobot {
    private static RecvSimRobot instance;

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     * If no instance exists, a new one is created.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    public static AbstractRobot getInstance() {
        if (instance == null) {
            instance = new RecvSimRobot();
        }
        return instance;
    }

    private RecvSimRobot() {
        super();

        try {
            ServerSocket serverSocket = new ServerSocket(8080);
            Socket client = serverSocket.accept();
            BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
            Publisher p = new Publisher(NodeChannel.POSE.getMsgPath());

            String input;
            while (true) {
                input = in.readLine();
                if (input == null) continue;
                String[] coords = input.split(",");

                p.publish(new GPSPoseMessage(new Date(), Double.parseDouble(coords[0]), Double.parseDouble(coords[1]), 0));
            }

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
