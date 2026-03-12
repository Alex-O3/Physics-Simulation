import PhysicsSim.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Main {
    public static boolean gameRunning = true;
    public static long timeStep = (long) (1000.0 / 300.0);
    public static double dt = (double) (timeStep) / 1000.0;
    public static void main(String[] args) throws Exception {
        Simulation sim = new Simulation(0, 0 ,500, 500,1.0, true, true);
        Simulation.showCreationInfo = true;
        sim.setDebugBounds(false,false);

        sim.setupDemo(11);

        long time = 0;
        System.out.println("Size: " + sim.size() + " objects/hitboxes");
        while (gameRunning) {
            try {
                if (time < timeStep) Thread.sleep(timeStep);
                time = sim.stepToNextFrame(dt, 5);
            }
            catch (Exception e) {
                System.out.println(e);
            }

        }
    }
}