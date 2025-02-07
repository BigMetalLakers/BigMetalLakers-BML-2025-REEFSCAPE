package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class Paths {
    // Make all paths
    private static final PathPlannerAuto path3 = new PathPlannerAuto("Path1.3");
    private static final PathPlannerAuto nullPath = new PathPlannerAuto("null");

    public static final PathPlannerAuto getPath(int pathNum, int colour) {
        // make pathNum dif based on current colour
        switch (pathNum) {
            case 3: 
                return path3;
        }
        return nullPath;
        
    }
}
