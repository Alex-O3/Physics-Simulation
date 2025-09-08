package PhysicsSim;
import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

public class Simulation {
    /*Porting the project to some other language or utilizing the GPU, perhaps through OpenCL, would allow quicker computation through parallelization.
    Switching between SAP and BVH optimizations (BVH still being improved), you may notice that they produce different outcomes.
    This is because they find the AABB pairs in different orders and therefore the impulses in each set of collisions are calculated
    and applied in different orders, producing different outcomes. This is most apparent in Demo 10. Fixing this issue is a later project.

    Future feature ideas include making springs attachable to a given location within a ball OR rigidbody and having more
        unique and/or custom spring structure options for softbodies, adding weld joints that bind
        bodies together at a specified point (with a specified movement allowance with specific settings), hinge joints that bind a point of a body (not edge)
        to the edge of a another, translational joints that bind edges together, muscle joints that can pull with tension,
        merging two contacting points on the same edge into one in-between contact point
        to give more convincing resting, adding a controller for moving alongside normals, improved visual scheme (though not the focus of the project),
        more controller options, multi-threading, universal gravity optimizations through multipole expansions, or more. Many of these
        are out of my reach for a long time.
    The simulation lacks continuous collision detection for between time steps, and higher time steps quickly lead to instability, particularly with softbodies.*/
    //as an important note, the IDs of rigidbodies are often stored more globally as positive integers (including 0),
    //points as negative even integers, and softbodies or softbody edges (depending on the use case) as the negative odd integers (in the latter case,
    //-1 is the wall).


    public final int ID;
    private static int num = 0;
    public static boolean showCreationWarnings = false;
    public static boolean showCreationInfo = false;
    private static final ArrayList<Simulation> simulations = new ArrayList<>();
    final ArrayList<SAPCell> sapCells = new ArrayList<>();
    public final ArrayList<BVHTreeRoot> BVHtrees = new ArrayList<>();
    final ArrayList<PhysicsObject> physicsObjects = new ArrayList<>();
    final ArrayList<Hitbox> hitboxes = new ArrayList<>();
    final static Material defaultMaterial = new Material("Default", 10.0, 0.75, 0.35);
    final static ArrayList<Material> materials = new ArrayList<>();

    //physical constants told to Softbody, Point, and Rigidbody
    public double COEFFICIENT_OF_RESTITUTION = 0.75;
    public double COEFFICIENT_OF_FRICTION = 0.35;
    public double GRAVITATIONAL_CONSTANT = 10.0;
    public double AIR_DENSITY = 0.01204;
    public double DRAG_COEFFICIENT = 0.95;
    public double CLAMP_LIMIT = 0.0;
    public double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public double MOUSE_SPEED_LIMIT = 1000.0;
    public double MTV_EPSILON = 0.00165;
    public double POINT_MIN_RADIUS = 5.0;
    public double DAMPING_COEFFICIENT_RELATOR = 0.6;
    public double SHAPE_DAMPING_COEFFICIENT_RELATOR = 0.6;
    public double REPULSION_STRENGTH = 150.0;
    public double REPULSE_RADIUS_MULTIPLIER = 2.0;

    public double worldTopBound = 0.0;
    public double worldBottomBound = 460.0;
    public double worldLeftBound = 0.0;
    public double worldRightBound = 485.0;

    public boolean universalGravity = false;
    public boolean airResistance = true;
    public boolean buoyancy = false;
    public boolean bounds = true;
    public boolean mouse = false;
    public boolean printKeys = false;
    public boolean drawing = true;
    private int fpsCountingBuffer = 60;
    private double fps = 60.0;
    private double[] frameTimes = new double[fpsCountingBuffer];
    private int frameCountLoop = 0;
    public boolean isSAPvsBVH = true;

    final Display display;
    public final double keysCacheRemovalBufferTime = 0.0;
    public Simulation(int x, int y, int screenWidth, int screenHeight, double resolutionScaling, boolean mouse) {
        ID = num;
        num = num + 1;
        display = new Display(x, y, screenWidth, screenHeight, ID, mouse);
        this.mouse = mouse;
        worldTopBound = 0.0;
        worldBottomBound = screenHeight - 40.0;
        worldLeftBound = 0.0;
        worldRightBound = screenWidth - 15.0;
        display.resolutionScaling = resolutionScaling;
        synchronizeSettings();
        simulations.add(this);
        new SAPCell(ID, 0, 0);
        BVHTreeRoot tree = new BVHTreeRoot(ID, 8);
    }

    public void synchronizeSettings() {
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (Rigidbody.get(i).simID != ID) continue;
            Rigidbody.get(i).GRAVITATIONAL_CONSTANT = GRAVITATIONAL_CONSTANT;
            Rigidbody.get(i).AIR_DENSITY = AIR_DENSITY;
            Rigidbody.get(i).DRAG_COEFFICIENT = DRAG_COEFFICIENT;
            Rigidbody.get(i).CLAMP_LIMIT = CLAMP_LIMIT;
            Rigidbody.get(i).CONTACT_POINTS_MERGE_DISTANCE = CONTACT_POINTS_MERGE_DISTANCE;
            Rigidbody.get(i).MOUSE_SPEED_LIMIT = MOUSE_SPEED_LIMIT;
            Rigidbody.get(i).MTV_EPSILON = MTV_EPSILON;
            Rigidbody.get(i).worldTopBound = worldTopBound;
            Rigidbody.get(i).worldBottomBound = worldBottomBound;
            Rigidbody.get(i).worldLeftBound = worldLeftBound;
            Rigidbody.get(i).worldRightBound = worldRightBound;
            Rigidbody.get(i).universalGravity = universalGravity;
            Rigidbody.get(i).airResistance = airResistance;
            Rigidbody.get(i).buoyancy = buoyancy;
            Rigidbody.get(i).bounds = bounds;
        }

        for (int i = 0; i < Point.num; i = i + 1) {
            if (Point.get(i).simID != ID) continue;
            Point.get(i).GRAVITATIONAL_CONSTANT = GRAVITATIONAL_CONSTANT;
            Point.get(i).AIR_DENSITY = AIR_DENSITY;
            Point.get(i).DRAG_COEFFICIENT = DRAG_COEFFICIENT;
            Point.get(i).CLAMP_LIMIT = CLAMP_LIMIT;
            Point.get(i).CONTACT_POINTS_MERGE_DISTANCE = CONTACT_POINTS_MERGE_DISTANCE;
            Point.get(i).MOUSE_SPEED_LIMIT = MOUSE_SPEED_LIMIT;
            Point.get(i).MTV_EPSILON = MTV_EPSILON;
            Point.get(i).worldTopBound = worldTopBound;
            Point.get(i).worldBottomBound = worldBottomBound;
            Point.get(i).worldLeftBound = worldLeftBound;
            Point.get(i).worldRightBound = worldRightBound;
            Point.get(i).universalGravity = universalGravity;
            Point.get(i).airResistance = airResistance;
            Point.get(i).buoyancy = buoyancy;
            Point.get(i).bounds = bounds;
            Point.get(i).POINT_MIN_RADIUS = POINT_MIN_RADIUS;
            Point.get(i).REPULSION_STRENGTH = REPULSION_STRENGTH;
            Point.get(i).REPULSE_RADIUS_MULTIPLIER = REPULSE_RADIUS_MULTIPLIER;
        }

        for (int i = 0; i < Softbody.num; i = i + 1) {
            if (Softbody.get(i).simID != ID) continue;
            Softbody.get(i).DAMP_COEFFICIENT = DAMPING_COEFFICIENT_RELATOR;
            Softbody.get(i).MTV_EPSILON = MTV_EPSILON;
        }
    }
    private int counterUntilTreeConstruction = 0;
    boolean tempBoolean = true;
    long lastRebalanceTime = 0;
    public long stepToNextFrame(double dt, int stepsPerFrame) {
        if (tempBoolean && !isSAPvsBVH) {
            BVHtrees.get(0).createTree();
            tempBoolean = false;
        }
        if (counterUntilTreeConstruction == 0 && !isSAPvsBVH) {
            long time = System.currentTimeMillis();
            BVHtrees.get(0).rebalanceTree();
            lastRebalanceTime = System.currentTimeMillis() - time;
            System.out.println("Time to Rebalance: " + lastRebalanceTime);
            tempBoolean = false;
        }
        counterUntilTreeConstruction += 1;
        if (counterUntilTreeConstruction % 60 == 0) counterUntilTreeConstruction = 0;
        long beforeTime = System.currentTimeMillis();
        dt = dt / (double) stepsPerFrame;
        for (int frameCount = 0; frameCount < stepsPerFrame; frameCount = frameCount + 1) {
            if (mouse) display.mouseDrag(dt, stepsPerFrame);
            Rigidbody.clearCollisionInformation(ID);
            Point.clearCollisionInformation(ID);
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
            Point.finalizeCollisionInformation(ID);
            Rigidbody.step(dt, ID);
            Point.step(dt, ID);
            Softbody.step(ID);
            Rigidbody.updateMotion(dt, ID);
            Point.updateMotion(dt, ID);
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
    public void setupDemo(int demoID) {
        if (validDemo()) switch(demoID) {
            case(3): {
                buoyancy = true;
                System.out.println("Demo 3: Same as demo 1, but with buoyancy and air resistance turned on.");
            }
            case(1): {
                if (demoID != 3) AIR_DENSITY = AIR_DENSITY / 10.0;
                addRigidbody(new double[]{-14.1, 7.3, 18.8, -10.9, 0.0, -26.6}, new double[]{-16.2, -20.7, 7.0, 23.7, -7.8, -0.8}, new double[]{200.0, 200.0, 0.0, 00.0, 0.0, 90.0, 0.0, 0.0}, 50.0, Color.blue);
                addRigidbody(new double[]{-21.3, 48.5, 53.0, 5.0, -32.0}, new double[]{32.4, 20.7, -21.0, -21.0, -1.6}, new double[]{400.0, 100.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 40.0, Color.yellow);
                //addRigidbody(new double[]{-500.0, 1000.0, 1000.0, -500.0}, new double[]{500.0, 500.0, 600.0, 600.0}, new double[]{250.0, 550.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 10000000.0, Color.green);
                //addObstacle(new double[]{-500.0, 1000.0, 1000.0, -500.0}, new double[]{500.0, 500.0, 600.0, 600.0}, 1.0, Color.green);
                addObstacle(new double[]{0.0, 300.0, 300.0, 0.0}, new double[]{250.0, 480.0, 500.0, 500.0}, 1.0, Color.green);
                addBall(new double[]{100.0, 125.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 15.0, 20.0, Color.black);
                if (demoID == 1) System.out.println("Demo 1: shows basic rigidbodies physically interacting. Includes an immovable object.");
                synchronizeSettings();
                break;
            }
            case(2): {
                COEFFICIENT_OF_FRICTION = 0.0;
                COEFFICIENT_OF_RESTITUTION = 0.25;
                addRigidbody(new double[]{-50.0, 25.0, 25.0, -50.0}, new double[]{-25.0, -25.0, 25.0, 25.0}, new double[]{50.0, 460.0, 50.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.black);
                addRigidbody(new double[]{-12.5, 12.5, 12.5, -12.5}, new double[]{-12.5, -12.5, 12.5, 12.5}, new double[]{80.0, 397.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 1.0, Color.blue);
                airResistance = false;
                synchronizeSettings();
                Rigidbody.get(1).COEFFICIENT_OF_FRICTION = 5.0;
                System.out.println("Demo 2: the top square has no starting velocity and is carried by friction between it and the bottom one with velocity. Friction and restitution unique to each surface-surface interaction.");
                break;
            }
            case(4): {
                addBall(new double[]{250.0, 250.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 25.0, 100000.0, Color.yellow);
                addBall(new double[]{250.0, 100.0, 82.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 10.0, 10000.0, Color.blue);
                addBall(new double[]{50.0, 250.0, 0.0, 71.0, 0.0, 0.0, 0.0, 0.0}, 5.0, 1000.0, Color.red);
                universalGravity = true;
                airResistance = false;
                bounds = false;
                synchronizeSettings();
                System.out.println("Demo 4: shows gravitational forces and orbits between the centers of mass of objects.");
                break;
            }
            case(5): {
                addRigidbody(3, 20.0, new double[]{30.0, 450.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 10.0, Color.gray);
                addRigidbody(4, 20.0, new double[]{400.0, 400.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 100.0, Color.blue);
                addBall(new double[]{250.0, 420.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 100.0, Color.blue);
                addFace(new double[]{250.0, 250.0}, new double[]{500.0, 250.0}, 100.0, Color.green);
                addFace(new double[]{250.0, 250.0}, new double[]{0.0, 250.0}, 100.0, Color.green);
                addFace(new double[]{300.0, 100.0}, new double[]{400.0, 100.0}, 0.0, Color.green);
                addFace(new double[]{400.0, 100.0}, new double[]{400.0, 200.0}, 0.0, Color.green);
                addFace(new double[]{400.0, 200.0}, new double[]{300.0, 200.0}, 0.0, Color.green);
                addFace(new double[]{300.0, 200.0}, new double[]{300.0, 100.0}, 0.0, Color.green);
                synchronizeSettings();
                System.out.println("Demo 5: shows one-sided face objects.");
                break;
            }
            case(6): {
                addRope(new double[]{25.0, 75.0, 125.0, 175.0, 225.0, 275.0, 325.0, 375.0, 425.0, 475.0}, new double[]{250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0, 250.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 10.0, Color.ORANGE, 200.0, 0.5, 5.0, new boolean[]{true, false, false, true, false, false, true, false, false, true});
                addRope(new double[]{250.0, 350.0, 450.0}, new double[]{25.0, 25.0, 25.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 25.0, Color.black, 100.0, 0.1, 5.0, new boolean[]{true, false, false});
                synchronizeSettings();
                System.out.println("Demo 6: shows linked springs between points, forming ropes.");
                break;
            }
            case(7): {
                addSpringSoftbody(new double[]{200.0, 475.0, 400.0, 100.0}, new double[]{400.0, 400.0, 325.0, 335.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 500.0, Color.blue, 405.0, 15.0, 5.0 / Math.sqrt(2));
                addObstacle(new double[]{0.0, 500.0, 500.0, 0.0}, new double[]{250.0, 250.0, 300.0, 300.0}, 0.0, Color.green);
                addPressureSoftbody(new double[]{100.0, 25.0, 25.0, 100.0}, new double[]{50.0, 50.0, 125.0, 125.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 500.0, Color.black, 500.0, 50.0, 5000.0, 2.5);
                addBall(new double[]{400.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 100.0, Color.yellow);
                //addRigidbody(6, 25.0, new double[]{400.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.yellow);
                addRigidbody(3, 40.0, new double[]{30.0, 450.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.gray);
                synchronizeSettings();
                System.out.println("Demo 7: shows pressure-mass (1) and spring-mass (0) softbody models.");
                break;
            }
            case(8): {
                addShapedSoftbody(true, new double[]{100.0, 200.0, 200.0, 100.0}, new double[]{100.0, 100.0, 300.0, 300.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 500.0, Color.blue, 100.0, 15.0, 3.0, 100.0);
                addRigidbody(new double[]{-50.0, 50.0, 50.0, -50.0}, new double[]{-50.0, -50.0, 50.0, 50.0}, new double[]{250.0, 250.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 1000.0, Color.blue);
                addShapedSoftbody(false, new double[]{350.0, 450.0, 450.0, 350.0}, new double[]{100.0, 100.0, 150.0, 150.0}, new double[]{-150.0, 0.0, 0.0, 90.0}, 500.0, Color.blue, 100.0, 20.0, 3.0, 100.0);
                airResistance = false;
                synchronizeSettings();
                System.out.println("Demo 8: shows shaped (2) softbody model.");
                break;
            }
            case(9): {
                bounds = false;
                AIR_DENSITY = AIR_DENSITY / 10.0;
                createMaterial("Ground1", 10.0, 0.1, 0.6);
                createMaterial("Ground2", 100.0, 0.5, 0.5);
                addBall(new double[]{250.0, 220.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 100.0, Color.blue);
                //addRigidbody(4, 25.0, new double[]{250.0, 250.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 100.0, Color.blue);
                addObstacle(new double[]{-500.0, 1000.0, 1000.0, -500.0}, new double[]{400.0, 400.0, 500.0, 500.0}, 10.0, Color.green);
                physicsObjects.get(0).addStandardControllerBundle();
                physicsObjects.get(1).setMaterial("Ground1");
                showVelocityVector(1.0);
                addObstacle(new double[]{375.0, 125.0, 125.0, 375.0}, new double[]{310.0, 310.0, 320.0, 320.0}, new double[]{0.0, 0.0, 0.0, 0.0, 0.1, 0.0}, 10.0, Color.green);
                physicsObjects.get(2).setMaterial("Ground2");
                addObstacle(new double[]{0.0, 275.0, 275.0, 0.0}, new double[]{250.0, 250.0, 260.0, 260.0}, new double[]{100.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0, Color.yellow);
                getObject(3).setMaterial("Ground1");
                synchronizeSettings();
                if(Simulation.showCreationInfo) {
                    listAllMaterials();
                    System.out.println();
                }
                System.out.println("Demo 9: shows the implementation of a player controller. 'Ground' materials to jump off must have 'Ground' in their name and player lock comes with 'Player' materials.");
                break;
            }
            case(10): {
                COEFFICIENT_OF_RESTITUTION = 0.99;
                AIR_DENSITY /= 10.0;
                COEFFICIENT_OF_FRICTION = 0.0;
                for (int i = 0; i < 2000; i = i + 1) {
                    addBall(new double[]{Math.random() * 500.0, Math.random() * 500.0, Math.random() * 50.0 * Math.signum(Math.random() - 0.5), Math.random() * 50.0 * Math.signum(Math.random() - 0.5), 0.0, 90.0, 0.0, 0.0}, 2.5, 10.0, Color.blue);
                    physicsObjects.get(physicsObjects.size() - 1).lockRotation(true);
                }
                //addShapedSoftbody(false, new double[]{200.0, 300.0, 300.0, 200.0}, new double[]{200.0, 200.0, 300.0, 300.0}, new double[]{0.0, 0.0, 0.0, 90.0}, 1000.0, Color.black, 100.0, 10.0, 3.0, 300.0);
                addBall(new double[]{250.0, 50.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0}, 25.0, 1000.0, Color.yellow);
                synchronizeSettings();
                System.out.println("Demo 10: shows optimization using Sweep and Prune algorithm.");
                break;
            }
            default: {
                System.out.println("No demo exists for that demo ID.");
                break;
            }
        }
    }
    private boolean validDemo() {
        if (physicsObjects.isEmpty()) return(true);
        else {
            System.out.println("Demo setup failed. Physics object(s) already exist.");
            return(false);
        }
    }

    public SAPCell getSAPCell(int x, int y) {
        //placeholder. SAPCells, if later fully implemented, will be implemented as a spatial hash table
        for (SAPCell sap : sapCells) {
            if (sap.x == x && sap.y == y) {
                return(sap);
            }
        }
        return(null);
    }
    public void setFrameCountingBuffer(int frameBuffer) {
        fpsCountingBuffer = frameBuffer;
        frameTimes = new double[fpsCountingBuffer];
    }

    public void addRigidbody(double[] x, double[] y, double[] motion, double mass, Color color) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Dynamic 'Rigidbody' " + Rigidbody.num);
        physicsObjects.add(new PhysicsObject(new Rigidbody(x, y, motion, mass, color, ID)));
        if (showCreationInfo) System.out.println();
    }
    public void addRigidbody(int numSides, double radius, double[] motion, double mass, Color color) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Dynamic 'Rigidbody' " + Rigidbody.num);
        double[] x = new double[numSides];
        double[] y = new double[numSides];
        double angleDivision = (2.0 * Math.PI) / numSides;
        for (int i = 0; i < numSides; i = i + 1) {
            x[i] = radius * Math.cos(angleDivision * i);
            y[i] = radius * Math.sin(angleDivision * i);
        }
        physicsObjects.add(new PhysicsObject(new Rigidbody(x, y, motion, mass, color, ID)));
        if (showCreationInfo) System.out.println();
    }
    public void addObstacle(double[] x, double[] y, double[] movingMotion, double mass, Color color) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Obstacle 'Rigidbody' " + Rigidbody.num);
        double[] motion = new double[]{0.0, 0.0, movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        Rigidbody obstacle = new Rigidbody(x, y, motion, mass, color, ID);
        obstacle.setPosX(obstacle.getPosX() + obstacle.getCX());
        obstacle.setPosY(obstacle.getPosY() + obstacle.getCY());
        obstacle.setIsMovable(false);
        physicsObjects.add(new PhysicsObject(obstacle));
        if (showCreationInfo) System.out.println();
    }
    public void addObstacle(double[] x, double[] y, double mass, Color color) {
        addObstacle(x, y, new double[]{0.0,0.0,0.0,0.0,0.0,0.0}, mass, color);
    }
    public void addObstacle(double[] pos, double radius, double[] movingMotion, double mass, Color color) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Obstacle 'Point' " + Point.num);
        double[] motion = new double[]{pos[0], pos[1], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3]};
        Point obstacle = new Point(motion, radius, mass, color, true, ID);
        obstacle.setIsMovable(false);
        physicsObjects.add(new PhysicsObject(obstacle));
        if (showCreationInfo) System.out.println();
    }
    public void addObstacle(double[] pos, double radius, double mass, Color color) {
       addObstacle(pos, radius, new double[]{0.0, 0.0, 0.0, 0.0}, mass, color);
    }
    public void addHitbox(double[] x, double[] y, double[] movingMotion, double mass, Color color, String name) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Hitbox 'Rigidbody' " + Rigidbody.num);
        double[] motion = new double[]{0.0, 0.0, movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        Rigidbody obstacle = new Rigidbody(x, y, motion, mass, color, ID);
        obstacle.setPosX(obstacle.getPosX() + obstacle.getCX());
        obstacle.setPosY(obstacle.getPosY() + obstacle.getCY());
        obstacle.setIsMovable(false);
        obstacle.isHitbox = true;
        obstacle.draw = false;
        hitboxes.add(new Hitbox(obstacle, name));
        if (showCreationInfo) System.out.println();
    }
    public void addHitbox(double[] x, double[] y, double mass, Color color, String name) {
        addHitbox(x, y, new double[]{0.0,0.0,0.0,0.0,0.0,0.0}, mass, color, name);
    }
    public void addHitbox(double[] pos, double radius, double[] movingMotion, double mass, Color color, String name) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Hitbox 'Point' " + Point.num);
        double[] motion = new double[]{pos[0], pos[1], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3]};
        Point obstacle = new Point(motion, radius, mass, color, true, ID);
        obstacle.setIsMovable(false);
        obstacle.isHitbox = true;
        obstacle.draw = false;
        hitboxes.add(new Hitbox(obstacle, name));
        if (showCreationInfo) System.out.println();
    }
    public void addHitbox(double[] pos, double radius, double mass, Color color, String name) {
        addHitbox(pos, radius, new double[]{0.0, 0.0, 0.0, 0.0}, mass, color, name);
    }
    public void addFace(double[] pos1, double[] pos2, double[] movingMotion, double width, double mass, Color color) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Face 'Rigidbody' " + Rigidbody.num);
        double[] motion = new double[]{0.0, 0.0, movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], movingMotion[4], movingMotion[5]};
        double nX = -(pos2[1] - pos1[1]);
        double nY = pos2[0] - pos1[0];
        double magnitude = Math.sqrt(nX * nX + nY * nY);
        nX /= magnitude;
        nY /= magnitude;
        double[] x = new double[]{pos1[0], pos2[0], -nX * width + pos2[0], -nX * width + pos1[0]};
        double[] y = new double[]{pos1[1], pos2[1], -nY * width + pos2[1], -nY * width + pos1[1]};
        Rigidbody face = new Rigidbody(x, y, motion, mass, color, ID);
        face.setPosX(face.getPosX() + face.getCX());
        face.setPosY(face.getPosY() + face.getCY());
        face.getTriangles().get(0).partOfFace = true;
        face.getTriangles().get(0).enabled[0] = false;
        face.getTriangles().get(0).enabled[2] = false;
        face.getTriangles().get(1).partOfFace = true;
        face.getTriangles().get(1).enabled[0] = false;
        face.getTriangles().get(1).enabled[1] = false;
        face.getTriangles().get(1).enabled[2] = false;
        face.setIsMovable(false);
        physicsObjects.add(new PhysicsObject(face));
        if (showCreationInfo) System.out.println();
    }
    public void addFace(double[] pos1, double[] pos2, double[] movingMotion, double mass, Color color) {
        addFace(pos1, pos2, movingMotion,1.0, mass, color);
    }
    public void addFace(double[] pos1, double[] pos2, double mass, Color color) {
        addFace(pos1, pos2, new double[]{0.0,0.0,0.0,0.0,0.0,0.0},1.0, mass, color);
    }
    public void addBall(double[] motion, double radius, double mass, Color color) {
        if (showCreationInfo) System.out.println("Physics Object " + physicsObjects.size() + ": Dynamic 'Point' " + Point.num);
        physicsObjects.add(new PhysicsObject(new Point(motion, radius, mass, color, true, ID)));
        if (showCreationInfo) {
            System.out.println("Area: " + (Math.PI * radius * radius) + ", Moment of Inertia: " + (0.5 * mass * radius * radius) + ", Mass: " + mass + ", Difference against air density: " + ((mass / (Math.PI * radius * radius)) - AIR_DENSITY));
            System.out.println();
        }
    }
    public void addPressureSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double initialPressure, double pointRadius) {
        physicsObjects.add(new PhysicsObject(new Softbody(1, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, initialPressure, mass, color, 2.0, 0.1, ID)));
        if (showCreationWarnings) {
            System.out.println("Pressure softbodies often oscillate and, like their spring counterpart, behave erratically.");
            System.out.println("Be wary of how softbodies will collide with moving rigidbodies and interlock with other softbodies");
        }
        if (showCreationInfo) {
            System.out.println("Physics Object " + (physicsObjects.size() - 1) + ": Dyanmic 'Softbody' " + (Softbody.num - 1));
            System.out.println("Size: " + Softbody.get(Softbody.num - 1).size() + " member Points and " + Softbody.get(Softbody.num - 1).boundarySize() + " boundary member Points with " + (mass / Softbody.get(Softbody.num - 1).size()) + " mass per member.");
            System.out.println("Initial Pressure: " + initialPressure + ", Mass: " + mass + " | Type: Pressure-Spring-Mass (1).");
            System.out.println();
        }
    }
    public void addSpringSoftbody(double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double pointRadius) {
        physicsObjects.add(new PhysicsObject(new Softbody(0, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, 0.0, mass, color, 1.5, 0.0, ID)));
        if (showCreationWarnings) {
            System.out.println("Spring softbodies can behave erratically or explosively if their internal structure is severely damaged and will otherwise deform under duress as an effect of nodes catching on one another.");
            System.out.println("Be wary of how softbodies will collide with moving rigidbodies and interlock with other softbodies");
        }
        if (showCreationInfo) {
            System.out.println("Physics Object " + (physicsObjects.size() - 1) + ": Dyanmic 'Softbody' " + (Softbody.num - 1));
            double massPer = mass / Softbody.get(Softbody.num - 1).size();
            System.out.println("Size: " + Softbody.get(Softbody.num - 1).size() + " member Points and " + Softbody.get(Softbody.num - 1).boundarySize() + " boundary member Points with " + massPer + " mass per member.");
            System.out.println("Stiffness: " + stiffness + ", Mass: " + mass + " | Type: Spring-Mass (0).");
            System.out.println();
        }
    }
    public void addShapedSoftbody(boolean isSolid, double[] definingX, double[] definingY, double[] movingMotion, double mass, Color color, double stiffness, double targetDensity, double pointRadius, double shape_match_strength) {
        int type = 3;
        if (isSolid) type = 2;
        physicsObjects.add(new PhysicsObject(new Softbody(type, definingX, definingY, movingMotion, pointRadius, targetDensity, stiffness, 0.0, mass, color, 2.0, 0.1, ID)));
        double massPer = (mass / Softbody.get(Softbody.num - 1).size());
        Softbody.get(Softbody.num - 1).SHAPE_MATCH_STRENGTH = shape_match_strength;
        Softbody.get(Softbody.num - 1).SHAPE_MATCH_DAMPING = SHAPE_DAMPING_COEFFICIENT_RELATOR * Math.sqrt(shape_match_strength);
        if (showCreationWarnings) {
            System.out.println("Shaped softbodies are the most stable model in practice.");
            System.out.println("Be wary of how softbodies will collide with moving rigidbodies and interlock with other softbodies.");
        }
        if (showCreationInfo) {
            System.out.println("Physics Object " + (physicsObjects.size() - 1) + ": Dyanmic 'Softbody' " + (Softbody.num - 1));
            System.out.println("Size: " + Softbody.get(Softbody.num - 1).size() + " member Points and " + Softbody.get(Softbody.num - 1).boundarySize() + " boundary member Points with " + massPer + " mass per member.");
            System.out.println("Stiffness: " + stiffness + ", Mass: " + mass + " | Type: Shape-Matching (2).");
            System.out.println();
        }
    }
    public void addRope(double[] x, double[] y, double[] movingMotion, double massPerPoint, Color color, double stiffness, double allowance_multiplier, double pointRadius, boolean[] lockedInPlace) {
        int startIndex = Point.num;
        int beforeSize = physicsObjects.size();
        double HOOKE_CONSTANT = stiffness * massPerPoint;
        double SPRING_DAMPING = DAMPING_COEFFICIENT_RELATOR * Math.sqrt(HOOKE_CONSTANT * massPerPoint);
        double minDist = 1.0 - allowance_multiplier;
        double maxDist = 1.0 + allowance_multiplier;
        for (int i = 0; i < x.length; i = i + 1) {
            Point ropePoint = new Point(new double[]{x[i], y[i], movingMotion[0], movingMotion[1], movingMotion[2], movingMotion[3], 0.0, 0.0}, pointRadius, massPerPoint, color, true, ID);
            ropePoint.lockRotation(true);
            physicsObjects.add(new PhysicsObject(ropePoint));
            if (i != 0) {
                ropePoint.attach(Point.get(startIndex + i - 1));
            }
            if (lockedInPlace[i]) ropePoint.setIsMovable(false);
            ropePoint.SPRING_MIN_DIST_MULTIPLIER = minDist;
            ropePoint.SPRING_MAX_DIST_MULTIPLIER = maxDist;
            ropePoint.HOOKE_SPRING_CONSTANT = HOOKE_CONSTANT;
            ropePoint.SPRING_DAMPING_COEFFICIENT = SPRING_DAMPING;
        }

        if (showCreationWarnings) System.out.println("Ropes must have some non-zero stiffness value or else drifting may occur.");
        if (showCreationInfo) {
            System.out.println("Physics Object " + beforeSize + " to " + (physicsObjects.size() - 1) + ": Dynamic Rope 'Point' " + startIndex + " to " + (Point.num - 1));
            System.out.println("Mass per node: " + massPerPoint + ", Stiffness: " + stiffness + ", Allowance: " + allowance_multiplier + ", Node size: " + pointRadius + ", Rope Node Count: " + x.length);
            System.out.println();
        }
    }

    public static Simulation get(int ID) {
        return(simulations.get(ID));
    }
    public PhysicsObject getObject(int ID) {
        return(physicsObjects.get(ID));
    }
    public PhysicsObject getObject(String type, int IDinType) throws Exception {
        boolean outOfBounds = true;
        for (int i = 0; i < physicsObjects.size(); i = i + 1) {
            if (physicsObjects.get(i).getType().equals(type) && physicsObjects.get(i).getIDinType() == IDinType) {
                outOfBounds = false;
                return(physicsObjects.get(i));
            }
        }
        throw new Exception("GetObject(String type, int localID) --> either 'type' does not exist or 'localID' is out of bounds.");
    }
    public Hitbox getHitbox(int ID) {
        return(hitboxes.get(ID));
    }
    public Hitbox getHitbox(String type, int IDinType) throws Exception {
        boolean outOfBounds = true;
        for (int i = 0; i < hitboxes.size(); i = i + 1) {
            if (hitboxes.get(i).getType().equals(type) && hitboxes.get(i).getIDinType() == IDinType) {
                outOfBounds = false;
                return(hitboxes.get(i));
            }
        }
        throw new Exception("GetHitbox(String type, int localID) --> either 'type' does not exist or 'localID' is out of bounds.");
    }
    public static void createMaterial(String name, double density, double restitution, double dynamic_friction) {
        boolean alreadyCreated = false;
        if (!materials.contains(defaultMaterial)) materials.add(defaultMaterial);
        for (Material material : materials) {
            if (material.name.equals(name)) {
                alreadyCreated = true;
                break;
            }
        }
        if (!alreadyCreated) {
            materials.add(new Material(name, density, restitution, dynamic_friction));
        }
        else System.out.println("A material with that name already exists.");

    }
    public static void listAllMaterials() {
        if (!materials.contains(defaultMaterial)) materials.add(defaultMaterial);
        for (Material material : materials) {
            material.print();
        }
    }
    public void showVelocityVector(double scaling) {
        display.debugVector = 1;
        display.debugVectorScaling = scaling;
    }
    public void showAccelerationVector(double scaling) {
        display.debugVector = 2;
        display.debugVectorScaling = scaling;
    }
    public void hideDebugVectors() {
        display.debugVector = 0;
    }

    public void sleepAllObjects() {
        for (PhysicsObject physicsObject : physicsObjects) {
            physicsObject.sleep();
        }
    }
    public void wakeAllObjects() {
        for (PhysicsObject physicsObject : physicsObjects) {
            physicsObject.wake();
        }
    }
    public int size() {
        int count = 0;
        for (PhysicsObject object : physicsObjects) {
            switch(object.getType()) {
                case("Rigidbody"): {
                    if (object.rigidbody.simID >= 0) count += 1;
                    break;
                }
                case("Point"): {
                    if (object.point.simID >= 0) count += 1;
                    break;
                }
                case("Softbody"): {
                    if (object.softbody.simID >= 0) count += 1;
                    break;
                }
            }
        }
        for (Hitbox object : hitboxes) {
            switch(object.getType()) {
                case("Rigidbody"): {
                    if (object.rigidbody.simID >= 0) count += 1;
                    break;
                }
                case("Point"): {
                    if (object.point.simID >= 0) count += 1;
                    break;
                }
            }
        }
        return(count);
    }
    public void pressKey(char input) {
        display.pressKey(input);
    }
    public void releaseKey(char input) {
        display.releaseKey(input);
    }
}
