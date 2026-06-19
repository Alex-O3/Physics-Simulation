import PhysicsSim.*;

import java.awt.*;

public class Main {
    public static boolean gameRunning = true;
    public static double dt = 1.0 / 120.0;
    public static long timeStep = (long) Math.floor(1000.0 * dt);
    public static void main(String[] args) throws Exception {
        Simulation sim = new Simulation(0, 0 ,500, 500,1.0, true, true);

        sim.setupDemo(1);

        long time = 0;
        System.out.println("Size: " + sim.awakeSize() + " objects/hitboxes");
        while (gameRunning) {
            try {
                if (time < timeStep) Thread.sleep(timeStep - time);
                time = sim.stepToNextFrame(dt, 12);
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

    public static void createTrackDemo(Simulation sim) {
        sim.bounds = false;
        sim.showCircleRotationDebug(true);
        sim.COEFFICIENT_OF_FRICTION = 0.5;
        sim.airResistance = true;
        Simulation.createMaterial("tire", 1000.0, 0.5, 0.5);
        Simulation.createMaterial("carBody", 2000.0, 0.2, 0.2);
        Simulation.createMaterial("obstacle", 50.0, 0.2, 0.2);
        double trackLength = 2000.0;
        double gravity = 90.0;
        double maxSpeed = Math.PI * 4.0;
        sim.addObstaclePolygon(new double[]{0.0, trackLength, trackLength, 0.0}, new double[]{500.0, 500.0, 525.0, 525.0}, 1.0, Color.green);
        sim.addRigidbodyCircle(new double[]{50.0, 450.0, 0.0, 0.0, 0.0, gravity, 0.0, 0.0}, 20.0, 100.0, Color.gray);
        sim.addRigidbodyCircle(new double[]{150.0, 450.0, 0.0, 0.0, 0.0, gravity, 0.0, 0.0}, 20.0, 100.0, Color.gray);
        sim.addRigidbodyPolygon(new double[]{20.0, 180.0, 180.0, 20.0}, new double[]{420.0, 420.0, 400.0, 400.0}, new double[]{100.0, 415.0, 0.0, 0.0, 0.0, gravity, 0.0, 0.0}, 300.0, Color.blue);
        sim.addRigidbodyPolygon(new double[]{0.0, 25.0, 0.0}, new double[]{-10.0, 20.0, 20.0}, new double[]{180.0 + 25.0 / 3.0, 410.0, 0.0, 0.0, 0.0, gravity, 0.0, 0.0}, 100.0, Color.cyan);

        sim.getObject(1).pinAttach(sim.getObject(3), new double[]{0.0, 0.0}, new double[]{-50.0, 5.0});
        sim.getObject(2).pinAttach(sim.getObject(3), new double[]{0.0, 0.0}, new double[]{50.0, 5.0});
        sim.getObject(4).weldAttach(sim.getObject(3), new double[]{-25.0 / 3.0, -10.0}, new double[]{80.0, 0.0});
        sim.addRigidbodyPolygon(4, 15.0, Math.PI / 4.0, new double[]{500.0, 475.0, 0.0, 0.0, 0.0, gravity, 0.0, 0.0}, 50.0, Color.red);

        sim.getObject(1).setMaterial("tire");
        sim.getObject(2).setMaterial("tire");
        sim.getObject(3).setMaterial("carBody");
        sim.getObject(4).setMaterial("carBody");



        PhysicsSim.Script tireMovementScript = new Script() {
            @Override
            public void runBefore(double dt) {
                try {
                    for (PhysicsObject tire : sim.getObjectWithMaterialName("tire")) tire.setAngularVelocity(tire.getAngularMovement()[0] + (maxSpeed - tire.getAngularMovement()[0]) * dt);
                }
                catch (Exception _) {

                }
            }

            @Override
            public void runAfter(double dt) {

            }
        };
        sim.getScripts().add(tireMovementScript);

        sim.describeAllObjects();
    }
}