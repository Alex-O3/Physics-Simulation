package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;

public class Simulation {
    //physical constants told to Softbody, Point, and Rigidbody
    public static final double COEFFICIENT_OF_RESTITUTION = 0.75;
    public static final double COEFFICIENT_OF_FRICTION_DYNAMIC = 0.35;
    public static final double COEFFICIENT_OF_FRICTION_STATIC = 0.5;
    public static final double GRAVITATIONAL_CONSTANT = 10.0;
    public static final double AIR_DENSITY = 0.01204;
    public static final double DRAG_COEFFICIENT = 0.95;
    public static final double CLAMP_LIMIT = 0.0;
    public static final double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public static final double HOLD_DAMPING = 1.0;
    public static final double MTV_EPSILON = 0.00165;
    public static double POINT_MIN_RADIUS = 5.0;
    public static double STIFFNESS = 405.0;
    public static double DAMPING_COEFFICIENT_RELATOR = 0.6;
    public static double SPRING_MAX_DIST_MULTIPLIER = 5.0;
    public static double SPRING_MIN_DIST_MULTIPLIER = 0.0;
    public static double REPULSION_STRENGTH = 0.0;

    public static final double worldTopBound = 0.0;
    public static final double worldBottomBound = 460.0;
    public static final double worldLeftBound = 0.0;
    public static final double worldRightBound = 485.0;

    public static final boolean universalGravity = false;
    public static final boolean airResistance = false;
    public static final boolean buoyancy = false;
    public static final boolean bounds = true;

    private final Display display;
    public Simulation(int screenWidth, int screenHeight) {
        display = new Display(screenWidth, screenHeight);
        synchronizeSettings();
    }

    public void synchronizeSettings() {
        Rigidbody.COEFFICIENT_OF_RESTITUTION = COEFFICIENT_OF_RESTITUTION;
        Rigidbody.COEFFICIENT_OF_FRICTION_DYNAMIC = COEFFICIENT_OF_FRICTION_DYNAMIC;
        Rigidbody.COEFFICIENT_OF_FRICTION_STATIC = COEFFICIENT_OF_FRICTION_STATIC;
        Rigidbody.GRAVITATIONAL_CONSTANT = GRAVITATIONAL_CONSTANT;
        Rigidbody.AIR_DENSITY = AIR_DENSITY;
        Rigidbody.DRAG_COEFFICIENT = DRAG_COEFFICIENT;
        Rigidbody.CLAMP_LIMIT = CLAMP_LIMIT;
        Rigidbody.CONTACT_POINTS_MERGE_DISTANCE = CONTACT_POINTS_MERGE_DISTANCE;
        Rigidbody.HOLD_DAMPING = HOLD_DAMPING;
        Rigidbody.MTV_EPSILON = MTV_EPSILON;
        Rigidbody.worldTopBound = worldTopBound;
        Rigidbody.worldBottomBound = worldBottomBound;
        Rigidbody.worldLeftBound = worldLeftBound;
        Rigidbody.worldRightBound = worldRightBound;
        Rigidbody.universalGravity = universalGravity;
        Rigidbody.airResistance = airResistance;
        Rigidbody.buoyancy = buoyancy;
        Rigidbody.bounds = bounds;

        Point.COEFFICIENT_OF_RESTITUTION = COEFFICIENT_OF_RESTITUTION;
        Point.COEFFICIENT_OF_FRICTION_DYNAMIC = COEFFICIENT_OF_FRICTION_DYNAMIC;
        Point.COEFFICIENT_OF_FRICTION_STATIC = COEFFICIENT_OF_FRICTION_STATIC;
        Point.GRAVITATIONAL_CONSTANT = GRAVITATIONAL_CONSTANT;
        Point.AIR_DENSITY = AIR_DENSITY;
        Point.DRAG_COEFFICIENT = DRAG_COEFFICIENT;
        Point.CLAMP_LIMIT = CLAMP_LIMIT;
        Point.CONTACT_POINTS_MERGE_DISTANCE = CONTACT_POINTS_MERGE_DISTANCE;
        Point.MTV_EPSILON = MTV_EPSILON;
        Point.worldTopBound = worldTopBound;
        Point.worldBottomBound = worldBottomBound;
        Point.worldLeftBound = worldLeftBound;
        Point.worldRightBound = worldRightBound;
        Point.universalGravity = universalGravity;
        Point.airResistance = airResistance;
        Point.buoyancy = buoyancy;
        Point.bounds = bounds;
        Point.POINT_MIN_RADIUS = POINT_MIN_RADIUS;
        Point.REPULSION_STRENGTH = REPULSION_STRENGTH;
        Softbody.DAMP_COEFFICIENT = DAMPING_COEFFICIENT_RELATOR;
    }
    public void step(double dt, boolean mouse) {
        if (mouse) display.mouseDragRigidbodies(dt);
        Rigidbody.step(dt);
        Point.step(dt);
        Softbody.step(dt);
        display.refresh();
    }
    public void setupDemo1() {
        if (validDemo()) {
            addRigidbody(new double[]{-14.1, 7.3, 18.8, -10.9, 0.0, -26.6}, new double[]{-16.2, -20.7, 7.0, 23.7, -7.8, -0.8}, new double[]{100.0, 200.0, 0.0, 00.0, 0.0, 90.0, 0.0, 0.0}, 5.0, Color.blue);
            addRigidbody(new double[]{-21.3, 48.5, 53.0, 5.0, -32.0}, new double[]{32.4, 20.7, -21.0, -21.0, -1.6}, new double[]{450.0, 100.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 40.0, Color.yellow);
            addObstacle(new double[]{0.0, 300.0, 300.0, 0.0}, new double[]{250.0, 480.0, 500.0, 500.0}, 0.0, Color.green);
            addBall(new double[]{100.0, 125.0, 0.0, 0.0, 0.0, 90.0}, 15.0, 20.0, Color.black);
        }
    }
    public void setupDemo2() {
        if (validDemo()) {
            addSpringSoftbody(new double[]{200.0, 475.0, 400.0, 100.0}, new double[]{400.0, 400.0, 325.0, 335.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 500.0, Color.blue, 405.0, 10.0, 5.0 / Math.sqrt(2));
            addObstacle(new double[]{0.0, 500.0, 500.0, 0.0}, new double[]{250.0, 250.0, 300.0, 300.0}, 0.0, Color.green);
            addPressureSoftbody(new double[]{100.0, 25.0, 25.0, 100.0}, new double[]{50.0, 50.0, 125.0, 125.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 186.0, Color.black, 500000.0);
            addRigidbody(3, 20.0, new double[]{30.0, 450.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 10.0, Color.gray);
        }
    }
    public void setupDemo3() {
        if (validDemo()) {
            addRope(new double[]{25.0, 75.0, 125.0, 175.0, 225.0, 275.0, 325.0, 375.0, 425.0, 475.0}, new double[]{250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 10.0, Color.ORANGE, 200.0, 0.5, 5.0, new boolean[]{true, false, false, true, false, false, true, false, false, true});
            addRope(new double[]{250.0, 350.0, 450.0}, new double[]{25.0, 25.0, 25.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 25.0, Color.black, 100.0, 0.1, 5.0, new boolean[]{true, false, false});
        }
    }
    private boolean validDemo() {
        if (Point.num == 0 && Rigidbody.num == 0 && Softbody.num == 0) return(true);
        else {
            System.out.println("Demo setup failed. Physics object(s) already exist.");
            return(false);
        }
    }

    public void addRigidbody(double[] x, double[] y, double[] motion, double mass, Color color) {
        new Rigidbody(x, y, motion, mass, color);
    }
    public void addRigidbody(int numSides, double radius, double[] motion, double mass, Color color) {
        double[] x = new double[numSides];
        double[] y = new double[numSides];
        double angleDivision = (2.0 * Math.PI) / numSides;
        for (int i = 0; i < numSides; i = i + 1) {
            x[i] = radius * Math.cos(angleDivision * i);
            y[i] = radius * Math.sin(angleDivision * i);
        }
        new Rigidbody(x, y, motion, mass, color);
    }
    public void addObstacle(double[] x, double[] y, double mass, Color color) {
        new Obstacle(x, y, mass, color);
    }
    public void addBall(double[] motion, double radius, double mass, Color color) {
        new Point(motion, radius, mass, color, true);
        System.out.println("Area: " + (Math.PI * radius * radius) + ", Moment of Inertia: " + (0.5 * mass * radius * radius) + ", Mass: " + mass + ", Difference against air density: " + ((mass / (Math.PI * radius * radius)) - AIR_DENSITY));
    }
    public void addSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, boolean isPressureModel, double pointRadius, double targetDensity, double stiffness, double initialPressure, double max_dist_multiplier, double min_dist_multiplier) {
        new Softbody(isPressureModel, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, initialPressure, mass, color, max_dist_multiplier, min_dist_multiplier);
    }
    public void addPressureSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double initialPressure) {
        new Softbody(true, definingX, definingY, movingMotion, POINT_MIN_RADIUS, 5.0, STIFFNESS, initialPressure, mass, color, 20.0, 0.1);
        System.out.println("Pressure softbodies often oscillate and, like their spring counterpart, behave erratically.");
        System.out.println("Be wary of how softbodies will collide with moving rigidbodies and interlock with other softbodies");
    }
    public void addSpringSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double pointRadius) {
        new Softbody(false, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, 0.0, mass, color, 1.5, 0.0);
        System.out.println("Spring softbodies can behave erratically or explosively if their internal structure is severely damaged and will otherwise deform under duress as an effect of nodes catching on one another.");
        System.out.println("Be wary of how softbodies will collide with moving rigidbodies and interlock with other softbodies");
    }
    public void addRope(double[] x, double[] y, double[] movingMotion, double massPerPoint, Color color, double stiffness, double allowance_multiplier, double pointRadius, boolean[] lockedInPlace) {
        int startIndex = Point.num;
        double HOOKE_CONSTANT = stiffness * massPerPoint;
        double SPRING_DAMPING = DAMPING_COEFFICIENT_RELATOR * Math.sqrt(HOOKE_CONSTANT * massPerPoint);
        double minDist = 1.0 - allowance_multiplier;
        double maxDist = 1.0 + allowance_multiplier;
        for (int i = 0; i < x.length; i = i + 1) {
            Point ropePoint = new Point(new double[]{x[i], y[i], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3]}, pointRadius, massPerPoint, color, true);
            if (i != 0) {
                ropePoint.attach(Point.get(startIndex + i - 1));
            }
            if (lockedInPlace[i]) ropePoint.setIsMovable(false);
            ropePoint.SPRING_MIN_DIST_MULTIPLIER = minDist;
            ropePoint.SPRING_MAX_DIST_MULTIPLIER = maxDist;
            ropePoint.HOOKE_SPRING_CONSTANT = HOOKE_CONSTANT;
            ropePoint.SPRING_DAMPING_COEFFICIENT = SPRING_DAMPING;
        }

        System.out.println("Ropes must have some non-zero stiffness value or else drifting may occur.");
    }
}
