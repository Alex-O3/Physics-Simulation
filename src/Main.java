import PhysicsSim.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Main {
    public static boolean gameRunning = true;
    public static double dt = 1.0 / 300.0;
    public static long timeStep = (long) (1000.0 * dt);
    public static void main(String[] args) throws Exception {
        Simulation sim = new Simulation(0, 0 ,500, 500,1.0, true, true);
        Simulation.showCreationInfo = true;
        sim.setupDemo(1);

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

    public static void setupPIExperiment(Simulation sim) {
        sim.bounds = false;
        sim.airResistance = false;
        sim.COEFFICIENT_OF_FRICTION = 0.0;
        sim.COEFFICIENT_OF_RESTITUTION = 1.0;

        sim.addObstaclePolygon(new double[]{-1000.0, 525.0, 525.0, -1000.0}, new double[]{500.0, 500.0, 525.0, 525.0},
                1.0, Color.green);
        sim.addRigidbodyPolygon(4, 10.0 * Math.sqrt(2.0), Math.PI / 4.0, new double[]{250.0, 490.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 0.01, Color.blue);
        sim.addRigidbodyPolygon(4, 20.0 * Math.sqrt(2.0), Math.PI / 4.0, new double[]{100.0, 480.0, 50.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.gray);
        sim.addObstaclePolygon(new double[]{500.0, 500.0, 525.0, 525.0}, new double[]{500.0, 450.0, 450.0, 500.0}, 1.0, Color.magenta);

        sim.getObject(1).makeAdoptOtherSurfaceOnly(true);
        sim.getObject(1).lockRotation(true);
        sim.getObject(2).lockRotation(true);
        sim.getObject(2).makeAdoptOtherSurfaceOnly(true);
        sim.getObject(0).setSurfaceCoefficient("RESTITUTION", 0.0);
        final int[] collisionCount = {0};
        sim.getObject(1).addEventListener(new EventListener() {
            @Override
            public void collidedPre(CollisionEvent e) {
                if (e.collision() == sim.getObject(2) || e.collision() == sim.getObject(3)) {
                    collisionCount[0]++;
                    System.out.println(collisionCount[0]);
                }
            }

            @Override
            public void collidedPost(CollisionEvent e) {

            }

            @Override
            public void intersected(HitboxIntersectionEvent e) {

            }

            @Override
            public void intersected(PhysicsObjectIntersectionEvent e) {

            }
        });
    }
}