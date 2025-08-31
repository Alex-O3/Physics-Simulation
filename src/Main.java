import PhysicsSim.*;

import java.awt.*;
import java.util.Arrays;

public class Main {
    public static boolean gameRunning = true;
    public static long timeStep = (long) (1000.0 / 60.0);
    public static double dt = (double) (timeStep) / 1000.0;
    public static void main(String[] args) throws Exception {
        Simulation sim = new Simulation(0, 0 ,500, 500,1.0, true);
        sim.setupDemo(8);

        long time = 0;
        System.out.println(sim.size());
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