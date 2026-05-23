package PhysicsSim;
import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * This is the center facade class for the package PhysicsSim, developed by Alex Ottenlips. The physics simulation takes place in a 2D environment, single-threaded, on a simplistic
 * JFrame display, natively, though the information for a custom display may be pulled with methods like getPositionsX() on a PhysicsObject or Hitbox. Accessible through the facade are those two
 * aforementioned classes, which allow the user to interface with objects in the simulation. This class includes methods like addRigidbodyPolygon() to create PhysicsObjects or Hitboxes, methods like getObject()
 * to retrieve the bodies, and a variety of other methods for purposes such as adding joints, testing for collisions, or sleeping objects.
 * To move from one frame of the simulation, call
 * stepToNextFrame() with the number of simulation steps per frame and the time, in seconds, per frame. To experiment with this class, try setupDemo() with a valid demoID to run preset demos.
 * Though not a comprehensive list, this is a brief list of the primary features of this project:
 * <ul>
 *     <li>Dynamic and static rigidbody and softbody collision, considering repulsion effects with softbodies to prevent softbody "tangles"
 *     and solid joint/spring collisions. Friction and restitution taken into account.</li>
 *     <li>One-sided face objects that bodies can only pass through from one side.</li>
 *     <li>Hitbox intersection tests, implemented via removing the collision response portion of collisions with a particular body.</li>
 *     <li>Highly simplified models of air resistance and gravity in accordance with Newton's Law of Universal Gravity.</li>
 *     <li>Controller methods to allow the control of a body via keyboard inputs.</li>
 *     <li>Varied list of options of joints, including spring, distance-contraint, pin, revolute, weld, and translational.</li>
 * </ul>
 */
public class Simulation {
    /*Porting the project to some other language or utilizing the GPU, perhaps through OpenCL, would allow quicker computation through parallelization.

    Future feature ideas include having more
        unique and/or custom spring structure options for softbodies, muscle joints that can pull with tension,
        adding a controller for moving alongside normals, improved visual scheme (though not the focus of the project),
        more controller options, multi-threading, universal gravity optimizations through multipole expansions, or more. Many of these
        are out of my reach for a long time.
    The simulation lacks continuous collision detection for between time steps, and higher time steps quickly lead to instability, particularly with softbodies.*/
    //as an important note, the IDs of rigidbodies are often stored more globally as positive integers (including 0),
    // and softbodies as the negative integers and -1 is the wall. For collidingIDs, solid joints are stored as -ID-2 of the parent rigidbodies and softbodies are not stored.


    public final int ID;
    private static int num = 0;
    public static boolean showCreationInfo = false;
    private final ArrayList<Script> scripts = new ArrayList<>();
    private static final ArrayList<Simulation> simulations = new ArrayList<>();
    final ArrayList<SAPCell> sapCells = new ArrayList<>();
    public final ArrayList<BVHTreeRoot> BVHtrees = new ArrayList<>();
    final ArrayList<Integer> rigidbodyObjectsIDToGlobalID = new ArrayList<>();
    final HashMap<String, Integer> rigidbodyHitboxesNamesToObjectID = new HashMap<>();
    final ArrayList<Integer> softbodyObjectsIDToGlobalID = new ArrayList<>();
    final ArrayList<PhysicsObject> physicsObjects = new ArrayList<>();
    final ArrayList<Hitbox> hitboxes = new ArrayList<>();
    final static Material defaultMaterial = new Material("Default", 10.0, 0.75, 0.15);
    final static HashMap<String, Material> materials = new HashMap<>();

    //physical constants told to Softbody and Rigidbody
    public double COEFFICIENT_OF_RESTITUTION = 0.75;
    public double COEFFICIENT_OF_FRICTION = 0.0;
    public double GRAVITATIONAL_CONSTANT = 10.0;
    public double AIR_DENSITY = 0.001204;
    public double DRAG_COEFFICIENT = 0.95;
    public final double[] WIND_SPEED = new double[]{0.0, 0.0};
    public double CLAMP_LIMIT = 0.0;
    public double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public double MOUSE_SPEED_LIMIT = 1000.0;
    public double MTV_EPSILON = 0.00165;
    public double PARALLEL_EDGE_TOLERANCE = 0.001;
    public double DAMPING_COEFFICIENT_RELATOR = 0.6;
    public double SHAPE_DAMPING_COEFFICIENT_RELATOR = 1.0;
    public double REPULSION_STRENGTH = 150.0;
    public double REPULSE_RADIUS_MULTIPLIER = 5.0;
    public double solidJointCollisionLineThickness = 2.0;

    public double worldTopBound = 0.0;
    public double worldBottomBound = 460.0;
    public double worldLeftBound = 0.0;
    public double worldRightBound = 485.0;

    public boolean universalGravity = false;
    public boolean airResistance = true;
    public boolean bounds = true;
    public boolean mouse = false;
    public boolean printKeys = false;
    public boolean drawing = true;
    public boolean displayCentersOfMasses = false;
    private int fpsCountingBuffer = 60;
    private double fps = 60.0;
    private double[] frameTimes = new double[fpsCountingBuffer];
    private int frameCountLoop = 0;
    public boolean isSAPvsBVH = true;
    public boolean hollowSoftbodiesSelfCollide = false;

    final Display display;
    public final double keysCacheRemovalBufferTime = 0.0;

    /**
     * Constructs a physics simulation using JFrame display to render single-color shapes. Objects are stored as
     * instances of PhysicsObject or Hitbox, whose simID must match this simulation ID to be altered by this Simulation
     * instance. IDs increment in order of creation.
     * @param x the x pixel coordinate of the top-left corner of the native JFrame display
     * @param y the y pixel coordinate of the top-left corner of the native JFrame display
     * @param screenWidth in pixels
     * @param screenHeight in pixels
     * @param resolutionScaling in pixels, the ratio between world space distance and display distance.
     * @param mouse whether the mouse can interact with objects.
     * @param draw whether to use the native JFrame display for this simulation.
     */
    public Simulation(int x, int y, int screenWidth, int screenHeight, double resolutionScaling, boolean mouse, boolean draw) {
        ID = num;
        num = num + 1;
        display = new Display(x, y, screenWidth, screenHeight, ID, mouse);
        if (!draw) {
            display.frame.setVisible(false);
            drawing = false;
        }
        this.mouse = mouse;
        worldTopBound = 0.0;
        worldBottomBound = screenHeight - 40.0;
        worldLeftBound = 0.0;
        worldRightBound = screenWidth - 15.0;
        display.resolutionScaling = resolutionScaling;
        simulations.add(this);
        new SAPCell(ID, 0, 0);
        new BVHTreeRoot(ID, 8);
    }

    /**
     *
     * @return ArrayList&lt;Script&gt; of all the scripts running before and after each simulation step.
     */
    public ArrayList<Script> getScripts() {
        return scripts;
    }

    private int counterUntilTreeConstruction = 0;
    private boolean firstFrame = true;
    private long lastRebalanceTime = 0;

    /**
     * Steps the simulation from one frame to the next at a specified time step. Larger time steps may result in tunnelling effects and unintended or inaccurate results.
     * Each simulation steps, axis-aligned-bounding-boxes (AABBS) are used in the broad phase in one of two currently implemented algorithms: Sweep-and-Prune (SAP) and Bounding-Volumne-Hierarchy (BVH).
     * These algorithms reduce the time complexity of collision detection to make simulating thousands of objects feasible on a single-thread. Once the broad phase is conducted,
     *  each pair of intersecting AABBs is tested in the narrow phase by Separating-Axis-Theorem (SAT) based algorithms depending on the set of geometries involved in the collision.
     *  Presently, only circles and polygons are used, with polygonal clipping allowed. Other special geometries are further allowed, such as those of solid distance-constraint and spring joints and
     *  one-sided faces, in addition to world bounds for the simulation. Following these procedures each step, the collision information for each rigidbody, in the form of a minimum translation vector (MTV) and point of contact, is
     *  trimmed to match sets of criteria, such as the merging of collision points too close to one another or the exclusion of hitboxes from physicsObject collision information.
     *  Afterwards, each rigidbody steps itself, calculating impulses to satisfy the constraint established by the coefficients of restitution and friction in a collision. Accelerations are then calculated according to universal gravity, obeying Newton's Law of Universal Gravitation despite
     *  being in 2D with non-symmetrical bodies, and other factors like air resistance, spring joints, and repulsion between softbodies. All impulses are applied to enforce constraints, but do so assuming they are the independent scenario, meaning the simulation gradually
     *  steps towards satisfying all constraints rather than satisfying all of them all at once, which would be to the detriment of algorithmic efficiency.
     *  Softbodies follow this by calculating properties like shape match and pressure forces before all objects and hitboxes update their information from the previous simulation step's state to the next. The mouse is considered once every simulation step, and may appear slower if FPS is lower.
     * @param dt the time assigned to one frame. Each internal simulation step is equal to dt / stepsPerFrame.
     * @param stepsPerFrame the number of internal simulation steps between each frame.
     * @return the length of time, in milliseconds, this step from one frame to another took.
     */
    public long stepToNextFrame(double dt, int stepsPerFrame) {
        if (firstFrame && !isSAPvsBVH) {
            BVHtrees.get(0).createTree();
            firstFrame = false;
        }
        if (counterUntilTreeConstruction == 0 && !isSAPvsBVH) {
            long time = System.currentTimeMillis();
            BVHtrees.get(0).rebalanceTree();
            lastRebalanceTime = System.currentTimeMillis() - time;
            if (Simulation.showCreationInfo) System.out.println("Time to Rebalance: " + lastRebalanceTime);
            firstFrame = false;
        }
        counterUntilTreeConstruction += 1;
        if (counterUntilTreeConstruction % 60 == 0) counterUntilTreeConstruction = 0;

        long beforeTime = System.currentTimeMillis();
        dt = dt / (double) stepsPerFrame;
        for (int frameCount = 0; frameCount < stepsPerFrame; frameCount = frameCount + 1) {
            for (Script script : scripts) script.runBefore();

            if (mouse) display.mouseDrag(dt, stepsPerFrame);
            Rigidbody.clearCollisionInformation(ID);
            if (isSAPvsBVH) for (SAPCell sap : sapCells) {
                sap.updateSort();
                for (int i = 0; i < sap.pairs.size() / 2; i = i + 1) {
                    sap.aabbs.get(sap.pairs.get(2 * i)).findCollisions(sap.aabbs.get(sap.pairs.get(2 * i + 1)));
                }
            }
            else for (BVHTreeRoot BVH : BVHtrees) {
                BVH.updateBounds();
                BVH.findPairs();

                for (int i = 0; i < BVH.pairs.size() / 2; i = i + 1) {
                    BVH.aabbs.get(BVH.pairs.get(2 * i)).findCollisions(BVH.aabbs.get(BVH.pairs.get(2 * i + 1)));
                }
            }
            Rigidbody.finalizeCollisionInformation(ID);
            Rigidbody.step(dt, ID);
            Softbody.step(ID);
            Rigidbody.updateMotion(dt, ID);

            for (Script script : scripts) script.runAfter();
        }

        if (!display.keyReleased) {
            display.firstPress = false;
        }
        else if (display.keyReleasedFirstTime) {
            if (!display.keysCache.isEmpty()) display.keysCache.remove((Character)display.keyPressed);
            if (!display.keysCache.isEmpty()) display.keyPressed = display.keysCache.getLast();
            else display.keyPressed = '\u0000';
            display.keyReleasedFirstTime = false;
        }
        if (keysCacheRemovalBufferTime > 0.0 && System.currentTimeMillis() - display.lastKeyTime > keysCacheRemovalBufferTime) {
            display.keyPressed = '\u0000';
            display.keysCache.clear();
            display.keyReleasedFirstTime = false;
        }
        if (printKeys && !display.keysCache.isEmpty()) System.out.println(display.keysCache);
        if (drawing) display.refresh();
        else {
            mouse = false;
            SwingUtilities.getWindowAncestor(display).setVisible(false);
        }

        long time = System.currentTimeMillis() - beforeTime;
        frameCountLoop = frameCountLoop % fpsCountingBuffer;
        frameTimes[frameCountLoop] = (double) time / 1000.0;
        double sum = 0.0;
        for (int i = 0; i < frameTimes.length; i = i + 1) {
            sum += frameTimes[i];
        }
        fps = (double)fpsCountingBuffer / sum;
        frameCountLoop += 1;
        display.fps.setText("FPS: " + String.format("%.2f", fps));
        return(time);
    }

    /**
     * Sets up this simulation as a demo of a given ID. If there are already objects or hitboxes in this simulation, a demo cannot be created. The following are valid demoIDs:
     * <br>(1): Basic dynamic rigidbodies with an immovable obstacle, bounds, and no extra features like air resistance or friction. The simplest the simulation gets.
     * <br>(2): Rigidbodies sliding one top of one another to demonstrate friction effects and the assignment of unique surface coefficients.
     * <br>(3): Demo 1, but with air resistance active and amplified for clarity of demonstration.
     * <br>(4): universal gravity as per Newton's Law of Universal Gravitation between the center of masses of rigidbodies.
     * <br>(5): one-sided faces that allow objects to pass through only from one side.
     * <br>(6): ropes with variable immovable or movable points and solid joint collisions.
     * <br>(7): a spring-pressure softbody on the top and a spring-mass softbody on the bottom.
     * <br>(8): a spring-shape-match hollow softbody on the left and a spring-shape-match solid softbody on the right.
     * <br>(9): a standard controller setup. Includes a death hitbox below the central platform.
     * <br>(10): a stress-test utilizing Sweep-and-Prune broad phase optimization.
     * <br>(11): a set of all currently implemented constraint joints. From left to right: distance-constraint, pin, revolute, weld, translational.
     * @param demoID the ID of the demo to set up.
     */
    public void setupDemo(int demoID) {
        if (validDemo()) {
            COEFFICIENT_OF_RESTITUTION = 0.5;
            COEFFICIENT_OF_FRICTION = 0.1;
            airResistance = false;
            worldBottomBound = 460.0;
            worldTopBound = 0.0;
            worldLeftBound = 0.0;
            worldRightBound = 485.0;
            switch(demoID) {
                case(3): {
                    AIR_DENSITY = 0.1;
                    airResistance = true;
                }
                case(1): {
                    if (demoID != 3) {
                        AIR_DENSITY = 0.0;
                        COEFFICIENT_OF_FRICTION = 0.0;
                    }
                    addRigidbodyPolygon(new double[]{-14.1, 7.3, 18.8, -10.9, 0.0, -26.6}, new double[]{-16.2, -20.7, 7.0, 23.7, -7.8, -0.8}, new double[]{200.0, 200.0, 0.0, 0.0, 0.0, 90.0, 3.0, 0.0}, 50.0, Color.blue);
                    addRigidbodyPolygon(new double[]{-21.3, 48.5, 53.0, 5.0, -32.0}, new double[]{32.4, 20.7, -21.0, -21.0, -1.6}, new double[]{400.0, 100.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 40.0, Color.yellow);

                    addObstaclePolygon(new double[]{0.0, 300.0, 300.0, 0.0}, new double[]{250.0, 480.0, 500.0, 500.0}, 1.0, Color.green);
                    addRigidbodyCircle(new double[]{100.0, 125.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 15.0, 10.0, Color.black);
                    if (demoID == 1) System.out.println("Demo 1: shows basic rigidbodies physically interacting. Includes an immovable object. No air resistance or friction in action.");
                    if (demoID == 3) System.out.println("Demo 3: Same as demo 1, but with friction and basic air resistance turned on. This simplified model assumes all \"air\" is moving at a uniform speed with a uniform density, itself unaffected by body. CFD is not considered.");
                    break;
                }
                case(2): {
                    COEFFICIENT_OF_FRICTION = 0.0;
                    COEFFICIENT_OF_RESTITUTION = 0.25;
                    addRigidbodyPolygon(new double[]{-50.0, 25.0, 25.0, -50.0}, new double[]{-25.0, -25.0, 25.0, 25.0}, new double[]{50.0, 440.0, 50.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.black);
                    //addObstaclePolygon(new double[]{0.0, 75.0, 75.0, 0.0}, new double[]{415.0, 415.0, 465.0, 465.0}, new double[]{50.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.black);
                    addRigidbodyPolygon(new double[]{-12.5, 12.5, 12.5, -12.5}, new double[]{-12.5, -12.5, 12.5, 12.5}, new double[]{100.0, 360.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 1.0, Color.blue);
                    createMaterial("TopSquare", 10.0, 0.25, 0.0);
                    createMaterial("BottomSquare", 100.0, 0.25, 5.0);
                    physicsObjects.get(0).setMaterial("BottomSquare");
                    physicsObjects.get(0).makeAdoptOtherSurfaceOnly(true);
                    physicsObjects.get(1).setMaterial("TopSquare");
                    physicsObjects.get(1).makeAdoptOtherSurfaceOnly(true);
                    System.out.println("Demo 2: the top square has no starting velocity and is carried by friction between it and the bottom one with velocity. Friction and restitution unique to each surface-surface interaction.");
                    break;
                }
                case(4): {
                    addRigidbodyCircle(new double[]{250.0, 250.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 25.0, 100000.0, Color.yellow);
                    addRigidbodyCircle(new double[]{250.0, 100.0, 82.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 10.0, 10000.0, Color.blue);
                    addRigidbodyCircle(new double[]{50.0, 250.0, 0.0, 71.0, 0.0, 0.0, 0.0, 0.0}, 5.0, 1000.0, Color.red);
                    universalGravity = true;
                    airResistance = false;
                    bounds = false;
                    System.out.println("Demo 4: shows a simplified model of gravitational forces between the centers of mass of objects.");
                    break;
                }
                case(5): {
                    addRigidbodyPolygon(3, 20.0, 0.0, new double[]{30.0, 450.0, 0.0, -400.0, 0.0, 90.0, 0.0, 0.0}, 10.0, Color.gray);
                    addRigidbodyPolygon(4, 20.0, 0.0, new double[]{400.0, 400.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 100.0, Color.blue);
                    addRigidbodyCircle(new double[]{250.0, 420.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 100.0, Color.blue);
                    addFace(new double[]{250.0, 250.0}, new double[]{500.0, 250.0}, 100.0, Color.green);
                    addFace(new double[]{250.0, 250.0}, new double[]{0.0, 250.0}, 100.0, Color.green);
                    addFace(new double[]{300.0, 100.0}, new double[]{400.0, 100.0}, 0.0, Color.green);
                    addFace(new double[]{400.0, 100.0}, new double[]{400.0, 200.0}, 0.0, Color.green);
                    addFace(new double[]{400.0, 200.0}, new double[]{300.0, 200.0}, 0.0, Color.green);
                    addFace(new double[]{300.0, 200.0}, new double[]{300.0, 100.0}, 0.0, Color.green);
                    System.out.println("Demo 5: shows one-sided face objects.");
                    break;
                }
                case(6): {
                    addRope(new double[]{25.0, 75.0, 125.0, 175.0, 225.0, 275.0, 325.0, 375.0, 425.0, 475.0}, new double[]{250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0},
                            new double[]{0.0, 0.0, 0.0, 90.0}, 10.0, Color.ORANGE, 200.0, 0.5,
                            5.0, new boolean[]{true, false, false, false, false, false, false, false, false, true});
                    addRope(new double[]{250.0, 350.0, 450.0}, new double[]{25.0, 25.0, 25.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 25.0, Color.black, 100.0, 0.1, 5.0, new boolean[]{true, false, false});

                    addRigidbodyPolygon(5, 15.0, 0.0, new double[]{250.0, 150.0, 0.0, 150.0, 0.0, 90.0, 0.0, 0.0}, 1.0, Color.blue);
                    addRigidbodyCircle(new double[]{50.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 100.0, Color.yellow);
                    addRigidbodyCircle(new double[]{450.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 5.0, 1.0, Color.yellow);
                    System.out.println("Demo 6: shows linked springs between points, forming ropes.");
                    break;
                }
                case(7): {
                    addSpringSoftbody(new double[]{200.0, 475.0, 400.0, 100.0}, new double[]{400.0, 400.0, 325.0, 335.0}, new double[]{0.0, 0.0, 0.0, 90.0},
                            500.0, Color.blue, 5000.0, 15.0, 5.0 / Math.sqrt(2));
                    addObstaclePolygon(new double[]{0.0, 500.0, 500.0, 0.0}, new double[]{250.0, 250.0, 300.0, 300.0}, 0.0, Color.green);
                    addPressureSoftbody(new double[]{100.0, 25.0, 25.0, 100.0}, new double[]{50.0, 50.0, 125.0, 125.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 500.0,
                            Color.black, 100.0, 30.0, 200.0, 2.5);
                    addRigidbodyCircle(new double[]{400.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 15.0, 100.0, Color.yellow);
                    addRigidbodyPolygon(3, 40.0, 0.0, new double[]{30.0, 450.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.gray);
                    System.out.println("Demo 7: shows pressure-mass and spring-mass softbody models.");
                    break;
                }
                case(8): {
                    addShapedSoftbody(false, new double[]{100.0, 200.0, 200.0, 100.0}, new double[]{100.0, 100.0, 300.0, 300.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 500.0, Color.blue, 100.0, 15.0, 3.0, 200.0);
                    addRigidbodyPolygon(new double[]{-50.0, 50.0, 50.0, -50.0}, new double[]{-50.0, -50.0, 50.0, 50.0}, new double[]{300.0, 250.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 200.0, Color.blue);
                    addShapedSoftbody(true, new double[]{350.0, 450.0, 450.0, 350.0}, new double[]{100.0, 100.0, 150.0, 150.0}, new double[]{-150.0, 0.0, 0.0, 90.0}, 500.0, Color.magenta, 100.0, 20.0, 3.0, 200.0);
                    System.out.println("Demo 8: shows shaped (2) softbody model.");
                    break;
                }
                case(9): {
                    bounds = false;
                    airResistance = true;
                    createMaterial("Ground1", 10.0, 0.1, 0.5);
                    createMaterial("Ground2", 100.0, 0.5, 0.3);
                    createMaterial("pillPart", 100.0, 1.0, 0.1);
                    addRigidbodyPolygon(new double[]{-15, 15, 15, -15}, new double[]{-32, -32, 32, 32}, new double[]{250.0, 220.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100, Color.blue);
                    addRigidbodyCircle(new double[]{250.0, 188.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 16.0, 100, Color.blue);
                    addRigidbodyCircle(new double[]{250.0, 252.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 16.0, 100, Color.blue);
                    getObject(0).setMaterial("pillPart");
                    getObject(1).setMaterial("pillPart");
                    getObject(2).addStandardControllerBundle();
                    getObject(0).weldAttach(getObject(1), new double[]{0.0, -32.0}, new double[]{0.0, 0.0});
                    getObject(0).weldAttach(getObject(2), new double[]{0.0, 32.0}, new double[]{0.0, 0.0});

                    //assign materials
                    addObstaclePolygon(new double[]{-500.0, 1000.0, 1000.0, -500.0}, new double[]{400.0, 400.0, 500.0, 500.0}, 10.0, Color.green);
                    getObject(3).setMaterial("Ground2");
                    showVelocityVector(1.0);
                    addObstaclePolygon(new double[]{375.0, 125.0, 125.0, 375.0}, new double[]{280.0, 280.0, 290.0, 290.0}, new double[]{0.0, 0.0, 0.0, 0.0, 0.1, 0.0}, 10.0, Color.green);
                    getObject(4).setMaterial("Ground2");
                    addObstaclePolygon(new double[]{0.0, 275.0, 275.0, 0.0}, new double[]{250.0, 250.0, 260.0, 260.0}, new double[]{100.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0, Color.yellow);
                    getObject(5).setMaterial("Ground1");

                    //add hitboxes
                    addHitboxPolygon(new double[]{-2000.0, 2000.0, 2000.0, -2000.0}, new double[]{800.0, 800.0, 1000.0, 1000.0}, Color.red, "Death_Zone");
                    getHitbox("Death_Zone").addEventListener(new EventListener() {
                        @Override
                        public void collidedPre(CollisionEvent e) {

                        }

                        @Override
                        public void collidedPost(CollisionEvent e) {

                        }

                        @Override
                        public void intersected(HitboxIntersectionEvent e) {

                        }

                        @Override
                        public void intersected(PhysicsObjectIntersectionEvent e) {
                            try {
                                if (e.physicsObject().getMaterialName().equals("Player"))
                                    e.physicsObject().setPosition(new double[]{250.0, 0.0});
                            } catch (Exception _) {

                            }
                        }
                    });

                    if (Simulation.showCreationInfo) {
                        listAllMaterials();
                        System.out.println();
                    }

                    System.out.println("Demo 9: shows the implementation of a player controller. 'Ground' materials to jump off must have 'Ground' in their name and player lock comes with 'Player' materials.");
                    System.out.println("Implementing the following code will detect when the player collides with the \"Death Zone\" hitbox: sim.getHitbox(0).getObjectCollisions().contains(sim.getObject(2));");
                    break;
                }
                case(10): {
                    isSAPvsBVH = true;
                    COEFFICIENT_OF_RESTITUTION = 1.0;
                    COEFFICIENT_OF_FRICTION = 0.0;
                    for (int i = 0; i < 2000; i = i + 1) {
                        double[] motion = new double[]{Math.random() * 500.0, Math.random() * 500.0, Math.random() * 50.0 * Math.signum(Math.random() - 0.5), Math.random() * 50.0 * Math.signum(Math.random() - 0.5), 0.0, 90.0, 0.0, 0.0};
                        addRigidbodyCircle(motion, 2.5, 10.0, Color.blue);
                    }
                    addRigidbodyCircle(new double[]{250.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 1000.0, Color.yellow);
                    System.out.println("Demo 10: shows optimization using Sweep and Prune algorithm.");
                    break;
                }
                case(11): {
                    //shows distance, pin, revolute, and weld separated by partitions
                    worldRightBound += 500.0;
                    double[] parentOffset = new double[]{25.0, -25.0};
                    double[] otherOffset = new double[]{-10.0, 10.0 * Math.sqrt(3.0)};
                    for (int i = 1; i <= 5; i++) {
                        addRigidbodyPolygon(new double[]{-25.0, 25.0, 25.0, -25.0}, new double[]{-25.0, -25.0, 25.0, 25.0}, new double[]{27.0 + 200.0 * (i - 1), 350.0, 0.0, 0.0, 0.0, 90.0, 0.5, 0.0}, 10.0, Color.blue);
                        addRigidbodyPolygon(new double[]{20.0, -10.0, -10.0}, new double[]{0.0, 10.0 * Math.sqrt(3.0), -10.0 * Math.sqrt(3.0)}, new double[]{27.0 + 200.0 * (i - 1), 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 20.0, Color.yellow);
                        PhysicsObject base = getObject(2 * i - 2);
                        PhysicsObject top = getObject(2 * i - 1);
                        switch (i) {
                            //distance
                            case 1: {
                                base.distanceSpringAttach(top, 100.0, 2.0, parentOffset, otherOffset, false);
                                break;
                            }
                            case 2: {
                                base.pinAttach(top, parentOffset, otherOffset);
                                break;
                            }
                            case 3: {
                                base.revoluteAttach(top, parentOffset, otherOffset, -0.25 * Math.PI, 0.25 * Math.PI);
                                break;
                            }
                            case 4: {
                                base.weldAttach(top, parentOffset, otherOffset);
                                break;
                            }
                            case 5: {
                                base.translationalAttach(top, parentOffset, otherOffset, new double[]{-1.0, 0.0}, new double[]{0.0, 50.0});
                                break;
                            }
                        }
                    }
                    for (int i = 1; i <= 4; i++) {
                        addObstaclePolygon(new double[]{132.0 + 180.0 * (i - 1), 132.0 + 180.0 * (i - 1), 132.0 + 180.0 * (i - 1) + 5.0, 132.0 + 180.0 * (i - 1) + 5.0}, new double[]{0.0, 460.0, 460.0, 0.0},1.0, Color.green);
                    }
                    System.out.println("Demo 11: shows, in order, distance-constrain joints, pin joints, revolute joints, and weld joints.");
                    break;
                }
                default: {
                    System.out.println("No demo exists for that demo ID.");
                    break;
                }
            }
            if (showCreationInfo) {
                describeAllObjects();
                describeAllHitboxes();
            }
        }
    }
    private boolean validDemo() {
        if (physicsObjects.isEmpty() && hitboxes.isEmpty()) return(true);
        else {
            System.out.println("Demo setup failed. Physics object(s) or Hitbox(s) already exist.");
            return(false);
        }
    }

    protected SAPCell getSAPCell(int x, int y) {
        //placeholder. SAPCells, if later fully implemented, will be implemented as a spatial hash table
        for (SAPCell sap : sapCells) {
            if (sap.x == x && sap.y == y) {
                return(sap);
            }
        }
        return(null);
    }

    /**
     * Sets the number of frames over which the fps is averaged.
     * @param frameBuffer
     */
    public void setFrameCountingBuffer(int frameBuffer) {
        fpsCountingBuffer = frameBuffer;
        frameTimes = new double[fpsCountingBuffer];
    }


    /**
     * Creates a rigidbody with a polygon geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param x the x-coordinates of this polygon in an imagined plane of creation
     * @param y the y-coordinates of this polygon in an imagined plane of creation
     * @param motion double[8]{posX, posY, vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass. All points will be moved such that their center of mass in the
     * imagined plane of creation matches the real world once the position of the center of mass is set.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display.
     */
    public void addRigidbodyPolygon(double[] x, double[] y, double[] motion, double mass, Color color) {
        physicsObjects.add(new PhysicsObject(new Rigidbody(new Polygon(x, y, MTV_EPSILON), motion, mass, color, ID)));
        rigidbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }
    /**
     * Creates a rigidbody with a regular polygon geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param numSides the number of side this polygon will be created with.
     * @param radius the distance from those sides to the origin.
     * @param phase the angle offset from the offset vector (1,0) that the first point is set to.
     * @param motion double[8]{posX, posY, vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass. All points will be moved such that their center of mass in the
     * imagined plane of creation matches the real world once the position of the center of mass is set.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display.
     */
    public void addRigidbodyPolygon(int numSides, double radius, double phase, double[] motion, double mass, Color color) {
        double[] x = new double[numSides];
        double[] y = new double[numSides];
        double angleDivision = (2.0 * Math.PI) / numSides;
        for (int i = 0; i < numSides; i = i + 1) {
            x[i] = radius * Math.cos(angleDivision * i + phase);
            y[i] = radius * Math.sin(angleDivision * i + phase);
        }
        addRigidbodyPolygon(x, y, motion, mass, color);
    }

    /**
     * Creates a rigidbody with a polygon geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param motion double[8]{posX, posY, vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass. All point will be moved such that their center of mass in the
     * imagined plane of creation matches the real world once the position of the center of mass is set.
     * @param radius the radius of the circle rigidbody to be created.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display.
     */
    public void addRigidbodyCircle(double[] motion, double radius, double mass, Color color) {
        physicsObjects.add(new PhysicsObject(new Rigidbody(new Circle(radius), motion, mass, color, ID)));
        rigidbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }

    /**
     * Creates an immovable obstacle rigidbody with a polygon geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param x the x-coordinates of this polygon in world space.
     * @param y the y-coordinates of this polygon in world space.
     * @param movingMotion double[6]{vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     */
    public void addObstaclePolygon(double[] x, double[] y, double[] movingMotion, double mass, Color color) {
        double[] motion = new double[]{0.0, 0.0, movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        Rigidbody obstacle = new Rigidbody(new Polygon(x, y, MTV_EPSILON), motion, mass, color, ID);
        obstacle.setPosX(x[0] - ((Polygon) obstacle.geometry).getXPoints(0));
        obstacle.setPosY(y[0] - ((Polygon) obstacle.geometry).getYPoints(0));
        obstacle.setIsMovable(false);
        physicsObjects.add(new PhysicsObject(obstacle));
        rigidbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }
    /**
     * Creates an immovable obstacle rigidbody with a polygon geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param x the x-coordinates of this polygon in world space.
     * @param y the y-coordinates of this polygon in world space.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     */
    public void addObstaclePolygon(double[] x, double[] y, double mass, Color color) {
        addObstaclePolygon(x, y, new double[]{0.0,0.0,0.0,0.0,0.0,0.0}, mass, color);
    }
    /**
     * Creates an immovable obstacle rigidbody with a circle geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param pos double[2]{posX, posY} of the circle's center of mass.
     * @param radius the radius of this obstacle circle.
     * @param movingMotion double[6]{vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     */
    public void addObstacleCircle(double[] pos, double radius, double[] movingMotion, double mass, Color color) {
        double[] motion = new double[]{pos[0], pos[1], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        Rigidbody obstacle = new Rigidbody(new Circle(radius), motion, mass, color, ID);
        obstacle.setIsMovable(false);
        physicsObjects.add(new PhysicsObject(obstacle));
        rigidbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }
    /**
     * Creates an immovable obstacle rigidbody with a circle geometry as a physics object. By default, it will be drawn to the native JFrame display and will not belong to a parent softbody, nor will it be part of connected or compound body.
     * @param pos double[2]{posX, posY} of the circle's center of mass.
     * @param radius the radius of this obstacle circle.
     * @param mass the mass of the rigidbody. The density is always assumed uniform, but may be changed.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     */
    public void addObstacleCircle(double[] pos, double radius, double mass, Color color) {
       addObstacleCircle(pos, radius, new double[]{0.0, 0.0, 0.0, 0.0}, mass, color);
    }
    /**
     * Creates a hitbox with a polygon geometry. By default, it will not be drawn to the native JFrame display.
     * @param x the x-coordinates of this polygon in world space.
     * @param y the y-coordinates of this polygon in world space.
     * @param movingMotion double[6]{vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     * @param name String assigned to this hitbox by which it can be referred to singly. If another hitbox occupies this name exactly, references by name will direct to this hitbox instead.
     */
    public void addHitboxPolygon(double[] x, double[] y, double[] movingMotion, Color color, String name) {
        double[] motion = new double[]{0.0, 0.0, movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        Rigidbody obstacle = new Rigidbody(new Polygon(x, y, MTV_EPSILON), motion, 1.0, color, ID);
        obstacle.setPosX(x[0] - ((Polygon) obstacle.geometry).getXPoints(0));
        obstacle.setPosY(y[0] - ((Polygon) obstacle.geometry).getYPoints(0));
        obstacle.setIsMovable(false);
        obstacle.isHitbox = true;
        obstacle.draw = false;
        hitboxes.add(new Hitbox(obstacle, name));
        rigidbodyObjectsIDToGlobalID.add(hitboxes.size() - 1);
        rigidbodyHitboxesNamesToObjectID.put(name, hitboxes.size() - 1);
    }
    /**
     * Creates a hitbox with a polygon geometry. By default, it will not be drawn to the native JFrame display.
     * @param x the x-coordinates of this polygon in world space.
     * @param y the y-coordinates of this polygon in world space.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     * @param name String assigned to this hitbox by which it can be referred to singly. If another hitbox occupies this name exactly, references by name will direct to this hitbox instead.
     */
    public void addHitboxPolygon(double[] x, double[] y, Color color, String name) {
        addHitboxPolygon(x, y, new double[]{0.0,0.0,0.0,0.0,0.0,0.0}, color, name);
    }
    /**
     * Creates a hitbox with a circle geometry. By default, it will not be drawn to the native JFrame display.
     * @param pos double[2]{posX, posY} of the circle's center of mass.
     * @param radius the radius of this hitbox circle.
     * @param movingMotion double[6]{vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     * @param name String assigned to this hitbox by which it can be referred to singly. If another hitbox occupies this name exactly, references by name will direct to this hitbox instead.
     */
    public void addHitboxCircle(double[] pos, double radius, double[] movingMotion, Color color, String name) {
        double[] motion = new double[]{pos[0], pos[1], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        Rigidbody obstacle = new Rigidbody(new Circle(radius), motion, 1.0, color, ID);
        obstacle.setIsMovable(false);
        obstacle.isHitbox = true;
        obstacle.draw = false;
        hitboxes.add(new Hitbox(obstacle, name));
        rigidbodyObjectsIDToGlobalID.add(hitboxes.size() - 1);
        rigidbodyHitboxesNamesToObjectID.put(name, hitboxes.size() - 1);
    }
    /**
     * Creates a hitbox with a circle geometry. By default, it will not be drawn to the native JFrame display.
     * @param pos double[2]{posX, posY} of the circle's center of mass.
     * @param radius the radius of this hitbox circle.
     * @param color the color this rigidbody will be drawn to the native JFrame display
     * @param name String assigned to this hitbox by which it can be referred to singly. If another hitbox occupies this name exactly, references by name will direct to this hitbox instead.
     */
    public void addHitboxCircle(double[] pos, double radius, Color color, String name) {
        addHitboxCircle(pos, radius, new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, color, name);
    }

    /**
     * Creates a one-sided rigidbody obstacle physics object. Objects may pass through from one-end, but not the other. The normal allowed is defined as the
     * vector from pos1 to pos2 rotated 90 degrees counterclockwise.
     * @param pos1 double[2]{posX, posY}
     * @param pos2 double[2]{posX, posY}
     * @param movingMotion double[6]{vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass.
     * @param width of the face rigidbody. A width too small may result in collision errors for larger time steps.
     * @param color java.awt Color instance of this object.
     */
    public void addFace(double[] pos1, double[] pos2, double[] movingMotion, double width, double mass, Color color) {
        double[] motion = new double[]{0.0, 0.0, movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        double nX = -(pos2[1] - pos1[1]);
        double nY = pos2[0] - pos1[0];
        double magnitude = Math.sqrt(nX * nX + nY * nY);
        nX /= magnitude;
        nY /= magnitude;
        double[] x = new double[]{pos1[0], pos2[0], -nX * width + pos2[0], -nX * width + pos1[0]};
        double[] y = new double[]{pos1[1], pos2[1], -nY * width + pos2[1], -nY * width + pos1[1]};
        Rigidbody face = new Rigidbody(new Polygon(x, y, MTV_EPSILON), motion, mass, color, ID);
        Polygon polygon = (Polygon) face.geometry;
        face.setPosX(x[0] - polygon.getXPoints(0));
        face.setPosY(y[0] - polygon.getYPoints(0));
        polygon.setFace(0);
        face.setIsMovable(false);
        physicsObjects.add(new PhysicsObject(face));
        rigidbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }
    /**
     * Creates a one-sided rigidbody obstacle physics object. Objects may pass through from one-end, but not the other. The normal allowed is defined as the
     * vector from pos1 to pos2 rotated 90 degrees counterclockwise. The width is set to 1 unit by default.
     * @param pos1 double[2]{posX, posY}
     * @param pos2 double[2]{posX, posY}
     * @param movingMotion double[6]{vX, vY, initialAX, initialAY, angularV, initialAngularA} in reference to the rigidbody's center of mass.
     * @param color java.awt Color instance of this object.
     */
    public void addFace(double[] pos1, double[] pos2, double[] movingMotion, double mass, Color color) {
        addFace(pos1, pos2, movingMotion,1.0, mass, color);
    }
    /**
     * Creates a one-sided rigidbody obstacle physics object. Objects may pass through from one-end, but not the other. The normal allowed is defined as the
     * vector from pos1 to pos2 rotated 90 degrees counterclockwise. The width is set to 1 unit by default and movingMotion is set to all 0 by default.
     * @param pos1 double[2]{posX, posY}
     * @param pos2 double[2]{posX, posY}
     * @param color java.awt Color instance of this object.
     */
    public void addFace(double[] pos1, double[] pos2, double mass, Color color) {
        addFace(pos1, pos2, new double[]{0.0,0.0,0.0,0.0,0.0,0.0},1.0, mass, color);
    }

    /**
     * Creates a pressure-spring softbody (hollow) along a defined border with a simplified model of pressure due to gas on the inside
     * of the softbody. Contains a solid joint boundary. Softbody members cannot collide with solid joints belonging to members of the same softbody, as enforced in broad phase collision, but pressure softbodies are the exception to this rule.
     * @param definingX the x boundary points along which the softbody is contained.
     * @param definingY the y boundary points along which the softbody is contained.
     * @param movingMotion double[4]{vX, vY, initialAX, initialAY} applied to each member body at the moment of creation. Acceleration values are converted to initial values that persist.
     * @param mass the mass of the entirety of all member bodies of the softbody.
     * @param color java.awt Color instance of each member body.
     * @param stiffness the Hooke's constant applied to each spring connection is this value * the mass per body while the damping coefficient is the
     *                  corresponding simulation's DAMPING_COEFFICIENT_RELATOR * sqrt(massPer * Hooke's constant).
     * @param targetDensity the target density lattice construction aims to achieve in members per 2500 unit^2 In this case, the target distance between points would create a lattice structure of the target density if used in a lattice rather than a hollow shape.
     * @param initialPressure the initial given pressure of the shape at construction. Pressure is evaluated every step as proportional by the initial pressure to the relative change in area compared to the softbody's area at construction.
     *                        For any boundary edge, the force applied is calculated as pressure * length.
     * @param pointRadius the radius of all member bodies.
     */
    public void addPressureSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double initialPressure, double pointRadius) {
        movingMotion = new double[]{movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], 0.0, 0.0};
        physicsObjects.add(new PhysicsObject(new Softbody(SoftbodyType.PressureSpring, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, initialPressure, mass, color, -1, -1, ID)));
        softbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }

    /**
     * Creates a lattice structure spring-mass softbody and contains a solid joint boundary. Note that these softbodies, not paired with shape-match or pressure methods, can
     * very easily deform upon contact such that the boundary is pushed inside the shape. Softbody members cannot collide with solid joints belonging to members of the same softbody, as enforced in broad phase collision.
     * @param definingX the x boundary points along which the softbody is contained.
     * @param definingY the y boundary points along which the softbody is contained.
     * @param movingMotion double[4]{vX, vY, initialAX, initialAY} applied to each member body at the moment of creation. Acceleration values are converted to initial values that persist.
     * @param mass the mass of the entirety of all member bodies of the softbody.
     * @param color java.awt Color instance of each member body.
     * @param stiffness the Hooke's constant applied to each spring connection is this value * the mass per body while the damping coefficient is the
     *                  corresponding simulation's DAMPING_COEFFICIENT_RELATOR * sqrt(massPer * Hooke's constant).
     * @param targetDensity the target density lattice construction aims to achieve in members per 2500 unit^2.
     * @param pointRadius the radius of all member bodies.
     */
    public void addSpringSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double pointRadius) {
        movingMotion = new double[]{movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], 0.0, 0.0};
        physicsObjects.add(new PhysicsObject(new Softbody(SoftbodyType.SpringMass, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, 0.0, mass, color, 1.5, 0.0, ID)));
        softbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
    }

    /**
     * Creates a softbody with spring connections between members. This type of softbody calculates the "ideal" shape of the softbody by
     * finding an angle offset from its original position that minimizes, by Least Squares, the distance between ideal points and real points. Spring-like
     * effects are then applied to every member between its position and its ideal position, using damping to prevent oscillatory explosions
     * dependent on the member's velocity relative to the softbody's center of mass. As opposed to solely using spring lattices, shape-match methods
     * prevent issues regarding deformation that render the lattice structure collapsing into unfavorable situations wherein the boundary is within the hull.
     * Contains a solid joint boundary.  Softbody members cannot collide with solid joints belonging to members of the same softbody, as enforced in broad phase collision.
     * @param isSolid whether the softbody is constructed via a lattice (true) or along the defined boundary (false).
     * @param definingX the x boundary points along which the softbody is contained.
     * @param definingY the y boundary points along which the softbody is contained.
     * @param movingMotion double[4]{vX, vY, initialAX, initialAY} applied to each member body at the moment of creation. Acceleration values are converted to initial values that persist.
     * @param mass the mass of the entirety of all member bodies of the softbody.
     * @param color java.awt Color instance of each member body.
     * @param stiffness the Hooke's constant applied to each spring connection is this value * the mass per body while the damping coefficient is the
     *                  corresponding simulation's DAMPING_COEFFICIENT_RELATOR * sqrt(massPer * Hooke's constant).
     * @param targetDensity the target density lattice construction aims to achieve in members per 2500 unit^2. The distance calculated for this lattice is used to determine the separation between boundary
     *                      members in the case of the softbody being hollow.
     * @param pointRadius the radius of all member bodies.
     * @param shape_match_strength multiplied by the mass of a member body, this equals Hooke's Constant in the spring relation between the current position
     *                             and the desired position. The corresponding damping coefficient is equal to its Simulation's DAMPING_COEFFICIENT_RELATOR * sqrt(match_strength * massPer).
     */
    public void addShapedSoftbody(boolean isSolid, double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double pointRadius, double shape_match_strength) {
        movingMotion = new double[]{movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], 0.0, 0.0};
        SoftbodyType type = SoftbodyType.ShapeMatchHollow;
        if (isSolid) type = SoftbodyType.ShapeMatchSolid;
        physicsObjects.add(new PhysicsObject(new Softbody(type, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, 0.0, mass, color, 2.0, 0.1, ID)));
        softbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
        double massPer = (mass / Softbody.get(Softbody.num - 1).size());
        Softbody.get(Softbody.num - 1).SHAPE_MATCH_STRENGTH = shape_match_strength;
        Softbody.get(Softbody.num - 1).SHAPE_MATCH_DAMPING = SHAPE_DAMPING_COEFFICIENT_RELATOR * Math.sqrt(massPer * shape_match_strength);
    }

    /**
     * Creates a rope with a member of Circle geometry at each coordinate given.
     * @param x double[n] of x-coordinates of points.
     * @param y double[n] of y-coordinates of points.
     * @param movingMotion double[4]{vX, vY, initialAX, initialAY} given to every member at construction. Note that immovable points will not be affected
     *                     by the initial acceleration values.
     * @param massPerPoint the mass given to each point in the rope, where none is concentrated in the solid joints.
     * @param color java.awt Color instance of the rope.
     * @param stiffness the Hooke's constant applied to each spring connection is this value * the mass per body while the damping coefficient is the
     *                  corresponding simulation's DAMPING_COEFFICIENT_RELATOR * sqrt(massPer * Hooke's constant).
     * @param allowance_multiplier proportional to the distance between members at construction, the leeway given in constructed distance-constraint joints.
     * @param pointRadius the radius of all member bodies.
     * @param lockedInPlace boolean[n] where true means to make the member immovable and false means to keep it movable. If this parameter is not sufficiently long, it is assumed
     *                      that the rest of its contents are false.
     */
    public void addRope(double[] x, double[] y, double[] movingMotion, double massPerPoint, Color color, double stiffness, double allowance_multiplier, double pointRadius, boolean[] lockedInPlace) {
        int startIndex = Rigidbody.num;
        int beforeSize = physicsObjects.size();
        double HOOKE_CONSTANT = stiffness * massPerPoint;
        double SPRING_DAMPING = DAMPING_COEFFICIENT_RELATOR * Math.sqrt(HOOKE_CONSTANT * massPerPoint);
        double minDist = 1.0 - allowance_multiplier;
        double maxDist = 1.0 + allowance_multiplier;
        ArrayList<Rigidbody> ropePoints = new ArrayList<>();
        for (int i = 0; i < x.length; i = i + 1) {
            Rigidbody ropePoint = new Rigidbody(new Circle(pointRadius), new double[]{x[i], y[i], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], 0.0, 0.0},
                    massPerPoint, color, ID);
            ropePoint.lockRotation(true);
            physicsObjects.add(new PhysicsObject(ropePoint));
            rigidbodyObjectsIDToGlobalID.add(physicsObjects.size() - 1);
            ropePoints.add(ropePoint);
            if (i != 0) {
                ropePoint.springAttach(Rigidbody.get(startIndex + i - 1), 1.0, 1.0);
            }
            if (i < lockedInPlace.length && lockedInPlace[i]) ropePoint.setIsMovable(false);
        }
        for (Rigidbody ropePoint : ropePoints) {
            ropePoint.setAllSpringJoints(HOOKE_CONSTANT, SPRING_DAMPING, minDist, maxDist);
            ropePoint.attachments.get(0).makeSolid();
        }

        if (showCreationInfo) {
            System.out.println("Rope " + beforeSize + " to " + (physicsObjects.size() - 1) + ": Dynamic Rope 'Rigidbody' " + startIndex + " to " + (Rigidbody.num - 1));
            System.out.println("Mass per node: " + massPerPoint + ", Stiffness: " + stiffness + ", Allowance: " + allowance_multiplier + ", Node size: " + pointRadius + ", Rope Node Count: " + x.length);
            System.out.println();
        }
    }

    /**
     * Get a Simulation with the given simID.
     * @param ID
     * @return Simulation
     */
    public static Simulation get(int ID) {
        return(simulations.get(ID));
    }

    /**
     * Get a PhysicsObject with the given object ID.
     * @param ID the ID of the object in question, which does not always match its localID. IDs increment in order of creation.
     * @return PhysicsObject
     * @throws Exception if that ID does not exist
     */
    public PhysicsObject getObject(int ID) {
        return physicsObjects.get(ID);
    }

    /**
     * Get a PhysicsObject of the given ID in its local type ("Rigidbody" or "Softbody").
     * @param type either "Rigidbody" or "Softbody"
     * @param IDinType the ID of the rigidbody or softbody, which is not always the same as its object ID. LocalIDs increment in order of creation and do not concern themselves with whether they are a hitbox or object.
     * @return PhysicsObject
     * @throws Exception if this ID does not exist.
     */
    public PhysicsObject getObject(String type, int IDinType) throws Exception {
        switch (type) {
            case "Rigidbody": {
                PhysicsObject physicsObject = physicsObjects.get(rigidbodyObjectsIDToGlobalID.get(IDinType));
                if (!physicsObject.rigidbody.isHitbox) return physicsObject;
            }
            case "Softbody": return physicsObjects.get(softbodyObjectsIDToGlobalID.get(IDinType));
            default: throw new Exception("getObject(String type, int localID) --> either 'type' does not exist or 'IDinType' is out of bounds.");
        }
    }
    /**
     * Get a Hitbox with the given hitbox ID.
     * @param ID the ID of the hitbox in question, which does not always match its localID. IDs increment in order of creation.
     * @return Hitbox
     * @throws Exception if that ID does not exist
     */
    public Hitbox getHitbox(int ID) {
        return(hitboxes.get(ID));
    }
    /**
     * Get a Hitbox of the given ID in its local type ("Rigidbody" or "Softbody").
     * @param type either "Rigidbody" or "Softbody"
     * @param IDinType the ID of the rigidbody or softbody, which is not always the same as its object ID. LocalIDs increment in order of creation and do not concern themselves with whether they are a hitbox or object.
     * @return Hitbox
     * @throws Exception if this ID does not exist.
     */
    public Hitbox getHitbox(String type, int IDinType) throws Exception {
        if (type.equals("Rigidbody")) {
            Hitbox hitbox = hitboxes.get(rigidbodyObjectsIDToGlobalID.get(IDinType));
            if (hitbox.rigidbody.isHitbox) return hitbox;
        }
        throw new Exception("getHitbox(String type, int localID) --> either 'type' does not exist or 'IDinType' is out of bounds.");
    }

    /**
     * Get the hitbox that was assigned this name.
     * @param name the name of the hitbox in question.
     * @return Hitbox
     */
    public Hitbox getHitbox(String name) {
        Integer id = rigidbodyHitboxesNamesToObjectID.get(name);
        if (id == null) return null;
        else return hitboxes.get(id);
    }

    /**
     * Create a material to hold common values of density, coefficients of restitution, and coefficients of friction.
     * Attempting to create a new material of a name that already exists results in nothing occurring.
     * Note that the list of materials is shared between all simulations, and that materials cannot be removed.
     * @param name the name assigned to this material to refer to later when assigning this material to objects.
     * @param density mass per 2500 unit^2 area.
     * @param restitution the coefficient of restitutions for collision resolution.
     * @param dynamic_friction the coefficient of friction for collision resolution. Bodies switch to static friction if the dynamic friction exceeds constraints.
     */
    public static void createMaterial(String name, double density, double restitution, double dynamic_friction) {
        if (!materials.containsValue(defaultMaterial)) materials.put("Default", defaultMaterial);
        if (!materials.containsKey(name)) {
            materials.put(name, new Material(name, density, restitution, dynamic_friction));
        }

    }

    /**
     * List all materials and their properties to System.out. Note that the list of materials is shared between all simulations.
     */
    public static void listAllMaterials() {
        if (!materials.containsValue(defaultMaterial)) materials.put("Default", defaultMaterial);
        for (Material material : materials.values()) {
            material.print();
        }
    }

    /**
     * List all objects and describe their properties to System.out, unless belonging to a parent softbody. Joint specific information is not provided, and information does not reflect rope instantiation.
     */
    public void describeAllObjects() {
        if (!physicsObjects.isEmpty()) System.out.println("\nNote: 'describeAllObjects()' calls do not describe ropes, but rather their constituent parts.\n");
        int i = 0;
        for (PhysicsObject physicsObject : physicsObjects) {
            if (!physicsObject.getType().equals("Rigidbody") || (physicsObject.getType().equals("Rigidbody") && physicsObject.rigidbody.parentSoftbody == -1)) {
                System.out.print("Physics Object " + i + ": ");
                System.out.println(physicsObject);
                System.out.println();
            }
            i += 1;
        }
        System.out.println("Size: " + physicsObjects.size() + " objects.");
    }

    /**
     * List all hitboxes and describe their properties to System.out.
     */
    public void describeAllHitboxes() {
        int i = 0;
        for (Hitbox hitbox : hitboxes) {
            System.out.print("Hitbox " + i + ": ");
            System.out.println(hitbox);
            System.out.println();
            i += 1;
        }
        System.out.println("\nSize: " + hitboxes.size() + " hitboxes.");
    }

    /**
     * Set the size of the window in which the simulation is displayed.
     * @param x pixel coordinate from top-left.
     * @param y pixel coordinate from top-left.
     * @param width pixel width of screen.
     * @param height pixel height of screen.
     */
    public void setScreenSize(int x, int y, int width, int height) {
        display.frame.setBounds(x, y, width, height);
    }

    /**
     * Set the resolution scaling the simulation displays in. Higher values widen the observable range of the simulation.
     * @param resolution
     */
    public void setResolution(double resolution) {
        display.resolutionScaling = resolution;
    }

    /**
     * Sets the top left corner x and y pixel coordinates on the screen.
     * @param x
     * @param y
     */
    public void setDisplayTopLeftCorner(int x, int y) {
        display.resolutionCenterX = x + display.getWidth() / display.resolutionScaling;
        display.pixelShiftX = Math.round(-(((x - display.resolutionCenterX) / display.resolutionScaling) + display.resolutionCenterX));
        display.resolutionCenterY = y + display.getHeight() / display.resolutionScaling;
        display.pixelShiftY = Math.round(-(((y - display.resolutionCenterY) / display.resolutionScaling) + display.resolutionCenterY));
    }

    /**
     * Shows the direction and magnitude of the velocity vectors at the centers of mass of all objects to the scaling given.
     * @param scaling
     */
    public void showVelocityVector(double scaling) {
        display.debugVector = 1;
        display.debugVectorScaling = scaling;
    }
    /**
     * Shows the direction and magnitude of the acceleration vectors at the centers of mass of all objects to the scaling given.
     * @param scaling
     */
    public void showAccelerationVector(double scaling) {
        display.debugVector = 2;
        display.debugVectorScaling = scaling;
    }

    /**
     * Disables the debug velocity and acceleration vectors.
     */
    public void hideDebugVectors() {
        display.debugVector = 0;
    }

    /**
     * Shows the rotation of rigidbodies with a circle geometry to display their angular movement.
     * @param showCircleRotation
     */
    public void showCircleRotationDebug(boolean showCircleRotation){
        display.showCircleRotation = showCircleRotation;
    }

    /**
     * Enables or disables the debug visual of the bounds of all AABBs in the simulation, as well as circles centered at every object with the largest radius of that object.
     * Circles depicted in light blue for softbodies represent the repulse radius for calculating effects of repulsion between members of different softbodies.
     * @param debugBounds
     * @param debugRadius
     */
    public void setDebugBounds(boolean debugBounds, boolean debugRadius) {
        display.debugBounds = debugBounds;
        display.debugRadius = debugRadius;
    }

    /**
     * Sets the amount of time that the mouse is allowed continue sliding the display as a multiple of time since the last mouse input was read.
     * @param sliding_multiplier
     */
    public void setMouse_Sliding_Multiplier(double sliding_multiplier) {
        if (Double.isFinite(sliding_multiplier)) display.MOUSE_SLIDING_MULTIPLIER = sliding_multiplier;
    }

    /**
     * Sets the simID of all object to -1 to effectively sleep the simulation.
     */
    public void sleepAllObjects() {
        for (PhysicsObject physicsObject : physicsObjects) {
            physicsObject.sleep();
        }
    }

    /**
     * Sets all objects to their original simIDs to effectively wake the simulation.
     */
    public void wakeAllObjects() {
        for (PhysicsObject physicsObject : physicsObjects) {
            physicsObject.wake();
        }
    }

    /**
     *
     * @return the count of awake objects and hitboxes in this simulation.
     */
    public int size() {
        return objectSize() + hitboxSize();
    }

    /**
     *
     * @return the count of awake physicsObjects.
     */
    public int objectSize() {
        int count = 0;
        for (PhysicsObject object : physicsObjects) {
            switch (object.getType()) {
                case "Rigidbody": {
                    if (object.rigidbody.simID >= 0) count += 1;
                    break;
                }
                case "Softbody": {
                    if (object.softbody.simID >= 0) count += 1;
                }
            }

        }
        return count;
    }

    /**
     *
     * @return the count of awake hitboxes
     */
    public int hitboxSize() {
        int count = 0;
        for (Hitbox object : hitboxes) {
            if (object.rigidbody.simID >= 0) count += 1;
        }
        return count;
    }

    /**
     * Keyboard press Controller input if the native keystroke detection functionality is not used.
     * @param input char input
     */
    public void pressKey(char input) {
        display.pressKey(input);
    }
    /**
     * Keyboard release Controller input if the native keystroke detection functionality is not used.
     * @param input char input
     */
    public void releaseKey(char input) {
        display.releaseKey(input);
    }
}
