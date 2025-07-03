import PhysicsSim.Simulation;
import java.awt.*;

public class Main {
    public static boolean gameRunning = true;
    public static long timeStep = (long) (1000.0 / 120.0);
    public static double dt = (double) (timeStep) / 1000.0;
    public static void main(String[] args) {
        Simulation sim = new Simulation(500, 500);
        sim.setupDemo1();

        while (gameRunning) {
            try {
                Thread.sleep(timeStep);
                sim.step(dt, true);
            }
            catch (Exception e) {
                System.out.println(e);
            }

        }
    }
}