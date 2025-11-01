import PhysicsSim.*;

import java.awt.*;
import javax.swing.*;
import java.util.Arrays;

public class Main {
    public static boolean gameRunning = true;
    public static long timeStep = (long) (1000.0 / 60.0);
    public static double dt = (double) (timeStep) / 1000.0;
    //prompt the user to choose a demo
    public static final Prompt prompt = new Prompt(new String[]{"Demo1", "Demo2", "Demo3", "Demo4", "Demo5", "Demo6", "Demo7", "Demo8", "Demo9", "Demo10"});
    public static boolean start = false;
    public static void main(String[] args) throws Exception {
        while (!start) {
            Thread.sleep(0);
        }
        Simulation sim = new Simulation(0, 0, 500, 500, 1.0, true);
        int demoID = Integer.parseInt(String.valueOf(prompt.input.substring(4)));
        sim.setupDemo(demoID);

        long time = 0;
        while (gameRunning && start) {
            try {
                if (time < timeStep) Thread.sleep(timeStep);
                time = sim.stepToNextFrame(dt, 5);
            }
            catch (Exception e) {
                System.out.println(e);
            }

        }
    }

    public static void beginRunning() {
        start = true;
    }
}