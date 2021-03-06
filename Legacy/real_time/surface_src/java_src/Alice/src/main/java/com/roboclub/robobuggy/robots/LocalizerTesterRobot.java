package com.roboclub.robobuggy.robots;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.simulation.LocalizerTester;
import com.roboclub.robobuggy.ui.ConfigurationPanel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.MainGuiWindow;
import com.roboclub.robobuggy.ui.PathPanel;
import com.roboclub.robobuggy.ui.RobobuggyGUITabs;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

import java.io.FileNotFoundException;
import java.util.ArrayList;

/**
 * Created by vivaanbahl on 1/27/17.
 */
public class LocalizerTesterRobot extends AbstractRobot {

    private static LocalizerTesterRobot instance;

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    protected LocalizerTesterRobot() {
        super();

        ArrayList<GpsMeasurement> waypoints = null;
        try {
            waypoints = WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile());
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        nodeList.add(new LocalizerTester("localizer tester", waypoints));
        nodeList.add(new RobobuggyKFLocalizer(10, "localizer", new LocTuple(40.441670, -79.9416362)));

        //setup the gui
        RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow", 1.0, 1.0);
        Gui.getInstance().addWindow(mainWindow);
        RobobuggyGUITabs tabs = new RobobuggyGUITabs();
        mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
        tabs.addTab(new MainGuiWindow(), "Home");
        tabs.add(new PathPanel(), "Path Visualizer");
        tabs.addTab(new ConfigurationPanel(), "Configuration");
    }

    /**
     * Returns a reference to the one instance of the {@link Robot} object.
     *
     * @return a reference to the one instance of the {@link Robot} object
     */
    public static LocalizerTesterRobot getInstance() {
        if (instance == null) {
            instance = new LocalizerTesterRobot();
        }
        return instance;
    }
}
