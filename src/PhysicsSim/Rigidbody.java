package PhysicsSim;
import java.util.ArrayList;
import java.awt.*;

class Rigidbody {
    public int simID;

    public double COEFFICIENT_OF_RESTITUTION = 0.75; //in my personal opinion, this is mislabeled. In the literature, it is labeled this way,
    //but in order to preserve a linear interpolation in between the minimum and maximum energy conservation values (in keeping with conservation of momentum)
    //the squared version of this constant should be used. However, since it only ever appears within a square root, the constant was simplified to be the square root of this linear interpolation/
    //It's almost as bad of a general-consensus choice, in my opinion, as calling standard deviation standard and absolute mean not as standard (although it should be).
    public double COEFFICIENT_OF_FRICTION_DYNAMIC = 0.35;
    public double COEFFICIENT_OF_FRICTION_STATIC = 0.5;
    public double GRAVITATIONAL_CONSTANT = 10.0;
    public double AIR_DENSITY = 0.01204;
    public double DRAG_COEFFICIENT = 0.95;
    public double CLAMP_LIMIT = 0.0;
    public double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public double HOLD_DAMPING = 1.0;
    public double FLING_SPEED_LIMIT = 500.0;
    public double MTV_EPSILON = 0.00165;

    public boolean universalGravity = false;
    public boolean airResistance = false;
    public boolean buoyancy = false;
    public boolean bounds = true;

    private static final ArrayList<Rigidbody> rigidbodies = new ArrayList<>();
    public static int num = 0;
    public final int ID;
    private boolean invertNormals = false;
    private boolean isMovable = true;
    protected boolean isHitbox = false;
    protected boolean draw = true;
    //these ArrayLists are not constant, but the pointer stored to them is, hence the "final"
    protected final ArrayList<Controller> controllers = new ArrayList<>();
    private final ArrayList<Double> xPoints = new ArrayList<>();
    private final ArrayList<Double> yPoints = new ArrayList<>();
    private final ArrayList<Triangle> triangles = new ArrayList<>();
    private final ArrayList<Double[]> contactPoints = new ArrayList<>();
    private final ArrayList<Double[]> MTVs = new ArrayList<>();
    protected final ArrayList<Integer> collidingIDs = new ArrayList<>();
    //for collidingIDs, 0 - infinity inclusive is reserved for other rigidbodies and -2 - -infinity is reserved for points. -1 is reserved for the walls
    protected double mass;
    protected double inertia;
    private final Color color;
    private final double area;
    private double largestDistance = 0.0;
    private double bottomBoundBox;
    private double topBoundBox;
    private double leftBoundBox;
    private double rightBoundBox;
    private double lastMouseX = 250.0;
    private double lastMouseY = 250.0;
    private double currentMouseX = 250.0;
    private double currentMouseY = 250.0;
    private double flingX = 0.0;
    private double flingY = 0.0;
    private boolean mouseHold = false;
    private boolean mouseRelease = false;
    private boolean lockedRotation = false;
    private boolean adoptOnlyOtherSurface = false;

    public double worldTopBound = 0.0;
    public double worldBottomBound = 460.0;
    public double worldLeftBound = 0.0;
    public double worldRightBound = 485.0;

    protected double posX;
    protected double posY;
    protected double vX;
    protected double vY;
    protected double aX;
    protected double aY;
    protected double angularV;
    protected double angularA;
    private final double[] initialExternalForces = new double[2];
    private final double initialExternalTorque;

    private double newposX;
    private double newposY;
    private double newvX;
    private double newvY;
    private double newaX;
    private double newaY;
    private double newangularV;
    private double newangularA;

    private double otherUpdatePosX;
    private double otherUpdatePosY;
    private double otherUpdateVX;
    private double otherUpdateVY;
    private double otherUpdateAX;
    private double otherUpdateAY;

    private final double cX;
    private final double cY;

    public Rigidbody(double[] inputX, double[] inputY, double[] motion, double mass, Color color, int simID) {
        this.simID = simID;
        Triangle.setMTVEpsilon(MTV_EPSILON);
        ID = num;
        num = num + 1;
        this.mass = mass;
        this.color = color;
        posX = motion[0];
        posY = motion[1];
        vX = motion[2];
        vY = motion[3];
        initialExternalForces[0] = motion[4];
        initialExternalForces[1] = motion[5];
        angularV = motion[6];
        initialExternalTorque = motion[7];
        COEFFICIENT_OF_RESTITUTION = Simulation.get(simID).COEFFICIENT_OF_RESTITUTION;
        COEFFICIENT_OF_FRICTION_DYNAMIC = Simulation.get(simID).COEFFICIENT_OF_FRICTION_DYNAMIC;
        COEFFICIENT_OF_FRICTION_STATIC = Simulation.get(simID).COEFFICIENT_OF_FRICTION_STATIC;

        int length = Math.min(inputX.length, inputY.length);
        for (int i = 0; i < length; i = i + 1) {
            xPoints.add(inputX[i]);
            yPoints.add(inputY[i]);
        }
        double totalArea = 0.0; //the signed area of the polygon to determine the winding order
        for (int i = 0; i < length; i = i + 1) {
            //find the signed area of the trapezoid
            double trapezoidArea = 0.5 * (xPoints.get(mod(i + 1, length)) - xPoints.get(i)) * (yPoints.get(mod(i + 1, length)) + yPoints.get(i));
            //add to the total signed area of the polygon
            totalArea = totalArea + trapezoidArea;
        }
        if (totalArea < 0.0) invertNormals = true;
        area = Math.abs(totalArea);

        //create a list of triangles in which each one is unassigned
        for (int i = 0; i < length; i = i + 1) {
            triangles.add(new Triangle());
        }

        //begin earmarking triangulation process.
        int numTriangles = 0;
        int[] pointExclusions = new int[length]; //3 is covered (excluded), 2 is convex but invalid (excluded as main), 1 is concave (excluded as main), and 0 is not investigated yet
        for (int i = 0; i < 10 * length; i = i + 1) {
            //find a suitable point in front of and behind the point in question in the order for attempted triangulation
            if (numTriangles == length - 2) break;
            for (int j = 0; j < length; j = j + 1) {
                if (pointExclusions[mod(i + j,length)] != 3) {
                    i = i + j;
                    break;
                }
            }
            int backwardIndex = mod(i,length);
            int forwardIndex = mod(i,length);
            for (int j = 1; j < length; j = j + 1) {
                if (pointExclusions[mod(i - j, length)] != 3) {
                    backwardIndex = mod(i - j, length);
                    break;
                }
            }
            for (int j = 1; j < length; j = j + 1) {
                if (pointExclusions[mod(i + j, length)] != 3) {
                    forwardIndex = mod(i + j, length);
                    break;
                }
            }
            //check if the point in question has a concave angle in the proposed triangle. Check normals on centroid
            double[] center = new double[]{(xPoints.get(mod(i,length)) + xPoints.get(mod(forwardIndex,length)) + xPoints.get(mod(backwardIndex,length))) / 3.0, (yPoints.get(mod(i,length)) + yPoints.get(mod(forwardIndex,length)) + yPoints.get(mod(backwardIndex,length))) / 3.0};
            boolean concave = false;
            concave = isOnNormalSide(center,mod(backwardIndex, length), mod(i,length));
            if (!concave) {
                concave = isOnNormalSide(center,mod(i,length),mod(forwardIndex, length));
            }
            if (!concave) {
                concave = isOnNormalSide(center,mod(forwardIndex, length),mod(backwardIndex, length));
            }
            //if concave, then mark it and move on to the next point.
            if (concave) pointExclusions[mod(i,length)] = 1;
            else {
                //check if any other polygon points are within the proposed triangle to determine the triangle's validity
                boolean valid = true;
                for (int j = 0; j < length; j = j + 1) {
                    if (j != mod(i,length) && j != mod(backwardIndex, length) && j != mod(forwardIndex, length) && pointExclusions[j] != 3) { //all checked points have to not be part of the triangle and not already covered
                        double[] point = new double[]{xPoints.get(j), yPoints.get(j)};
                        boolean validPoint = isOnNormalSide(point, mod(backwardIndex, length), mod(i,length));
                        if (!validPoint) validPoint = isOnNormalSide(point, mod(i,length), mod(forwardIndex,length));
                        if (!validPoint) validPoint = isOnNormalSide(point, mod(forwardIndex,length), mod(backwardIndex,length));
                        if (!validPoint) {
                            valid = false;
                            pointExclusions[i] = 2; //2 means the point in question is convex, but invalid because it contains another polygon point in the proposed triangle
                            break;
                        }
                    }
                }
                if (valid) { //here, the proposed triangle has been verified to be both within the polygon (concave test)
                             // and to not contain any other polygon points inside it, so as to not be self-intersecting
                    //Save as a triangle and mark point as covered (3)
                    pointExclusions[mod(i,length)] = 3; //it means this point is the parent point of a triangle that is now removed from the polygon in the triangulation process
                    triangles.set(mod(i,length), new Triangle(mod(backwardIndex, length), mod(i,length), mod(forwardIndex,length),ID, center));
                    numTriangles = numTriangles + 1;
                }
            }
        }
        //calculate properties of polygon
        //center of mass
        rigidbodies.add(this);
        double centerOfMassX = 0.0;
        double centerOfMassY = 0.0;
        double calcInertia = 0.0;
        //center of mass
        for (int i = 0; i < length; i = i + 1) {
            if (pointExclusions[i] == 3) {
                triangles.get(i).calculateProperties();
                centerOfMassX = centerOfMassX + (triangles.get(i).getArea() / area) * triangles.get(i).getCenterX();
                centerOfMassY = centerOfMassY + (triangles.get(i).getArea() / area) * triangles.get(i).getCenterY();
            }
        }
        //moment of inertia
        for (int i = 0; i < length; i = i + 1) {
            double squaredDistance = (triangles.get(i).getCenterX() - centerOfMassX) * (triangles.get(i).getCenterX() - centerOfMassX);
            squaredDistance = squaredDistance + (triangles.get(i).getCenterY() - centerOfMassY) * (triangles.get(i).getCenterY() - centerOfMassY);
            calcInertia = calcInertia + triangles.get(i).getInertia() + (triangles.get(i).getArea() / area) * mass * squaredDistance;
        }
        inertia = calcInertia;
        //shift all points in the polygon's coordinate plane such that the center of mass is the origin and find the largest squared distance
        cX = centerOfMassX;
        cY = centerOfMassY;
        for (int i = 0; i < length; i = i + 1) {
            xPoints.set(i, xPoints.get(i) - centerOfMassX);
            if (Double.isNaN(leftBoundBox) || xPoints.get(i) < leftBoundBox) leftBoundBox = xPoints.get(i);
            if (Double.isNaN(rightBoundBox) || xPoints.get(i) > rightBoundBox) rightBoundBox = xPoints.get(i);
            yPoints.set(i, yPoints.get(i) - centerOfMassY);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) < topBoundBox) topBoundBox = yPoints.get(i);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) > bottomBoundBox) bottomBoundBox = yPoints.get(i);
            triangles.get(i).shift(centerOfMassX, centerOfMassY);
            double temp = xPoints.get(i) * xPoints.get(i) + yPoints.get(i) * yPoints.get(i);
            if (temp > largestDistance) largestDistance = temp;
        }

        if (Simulation.showCreationInfo) {
            System.out.println("Area: " + area + ", Moment of Inertia: " + inertia + ", Mass: " + this.mass + ", Difference against air density: " + ((mass / area) - AIR_DENSITY));
            System.out.print("Triangulation Categorization: [");
            //3 is covered (excluded), 2 is convex but invalid (excluded as main), 1 is concave (excluded as main), and 0 is not investigated yet
            for (int i = 0; i < pointExclusions.length; i = i + 1) {
                switch (pointExclusions[i]) {
                    case (0): {
                        System.out.print("Skipped");
                        break;
                    }
                    case (1): {
                        System.out.print("Concave (not skipped, invalid)");
                        break;
                    }
                    case (2): {
                        System.out.print("Convex (not skipped, self-intersecting -> invalid)");
                        break;
                    }
                    case (3): {
                        System.out.print("Valid ear vertex (convex, not self-intersecting -> valid)");
                        break;
                    }
                }
                if (i < pointExclusions.length - 1) System.out.print(" | ");
                else System.out.println("]");
            }
        }

    }

    //general motion. Copied and altered to fit Point in that file
    public static void step(double dt, int simID) {
        for (int i = 0; i < rigidbodies.size(); i = i + 1) {
            if (rigidbodies.get(i).simID == simID) {
                if (rigidbodies.get(i).lockedRotation) {
                    rigidbodies.get(i).angularV = 0.0;
                    rigidbodies.get(i).angularA = 0.0;
                }
                if (rigidbodies.get(i).isMovable() && !rigidbodies.get(i).isHitbox) {
                    rigidbodies.get(i).calcMotion(dt);
                }
                else if (rigidbodies.get(i).vX != 0.0 || rigidbodies.get(i).vY != 0.0 || rigidbodies.get(i).angularV != 0.0) {
                    rigidbodies.get(i).calcMotionFreeMove(dt, true);
                }
                if (rigidbodies.get(i).isHitbox) rigidbodies.get(i).findCollisions();
            }
        }
    }
    public static void updateMotion(double dt, int simID) {
        for (int i = 0; i < rigidbodies.size(); i = i + 1) {
            if (rigidbodies.get(i).simID == simID) {
                if (rigidbodies.get(i).isMovable()) rigidbodies.get(i).updateMotion(dt);
                else if (rigidbodies.get(i).vX != 0.0 || rigidbodies.get(i).vY != 0.0 || rigidbodies.get(i).angularV != 0.0) rigidbodies.get(i).updateMotion(dt);
            }
        }
    }
    private boolean findCollisions() {
        //clear the list of point information
        contactPoints.clear();
        MTVs.clear();
        collidingIDs.clear();

        //determine point information
        boolean intersecting = false;
        for (int i = 0; i < rigidbodies.size(); i = i + 1) {
            if (i != ID && Rigidbody.get(i).simID == simID && (isHitbox || !Rigidbody.get(i).isHitbox)) {
                if (!intersecting) intersecting = checkForCollisionsBroad(Rigidbody.get(i));
                else checkForCollisionsBroad(Rigidbody.get(i));
            }
        }
        if (!intersecting && bounds) intersecting = checkForCollisionsWall();
        else if (bounds) checkForCollisionsWall();
        for (int i = 0; i < Point.num; i = i + 1) {
            if (!(Point.get(i).simID == simID && (isHitbox || !Point.get(i).isHitbox))) continue;
            if (!intersecting) intersecting = checkForCollisionsBroad(Point.get(i));
            else checkForCollisionsBroad(Point.get(i));
        }
        for (int i = 0; i < Softbody.num; i = i + 1) {
            if (!(Softbody.get(i).simID == simID)) continue;
            if (!intersecting) intersecting = checkForCollisionsBroad(Softbody.get(i));
            else checkForCollisionsBroad(Softbody.get(i));
        }
        //prune the list for points too close to one another (same point, but different triangle with floating point precision differences)
        if (!contactPoints.isEmpty()) {
            for (int i = 0; i < contactPoints.size(); i = i + 1) {
                for (int j = i + 1; j < contactPoints.size(); j = j + 1) {
                    if (i != j && !Double.isNaN(contactPoints.get(i)[0]) && !Double.isNaN(contactPoints.get(j)[0])) {
                        double temp1 = contactPoints.get(i)[0] - contactPoints.get(j)[0];
                        double temp2 = contactPoints.get(i)[1] - contactPoints.get(j)[1];
                        double distance = temp1 * temp1 + temp2 * temp2;
                        distance = Math.sqrt(Math.max(distance, 0.0));
                        if (distance <= CONTACT_POINTS_MERGE_DISTANCE) {
                            contactPoints.set(j, new Double[]{Double.NaN, Double.NaN});
                        }
                    }
                }
            }
        }
        int length = contactPoints.size();
        for (int i = 0; i < length; i = i + 1) {
            if (i >= contactPoints.size()) break;
            if (Double.isNaN(contactPoints.get(i)[0])) {
                contactPoints.remove(i);
                MTVs.remove(i);
                collidingIDs.remove(i);
                i = i - 1;
            }
        }
        return(intersecting);
    }
    private void calcMotion(double dt) {
        //check for collisions and do one of two options for updating motion based on whether the rigidbody is colliding with another
        boolean intersecting = findCollisions();

        newposX = posX;
        newposY = posY;
        newvX = vX;
        newvY = vY;
        newangularV = angularV;
        //dt is divided by the number of collisions so that acceleration and velocity are not repeatedly
        //applied, which would ultimately result in (for example) a 3x acceleration with 3 collision points
        if (intersecting) dt = dt / contactPoints.size();
        if (intersecting) for (int h = 0; h < contactPoints.size(); h = h + 1) {
            newposX = newposX + MTVs.get(h)[0];
            newposY = newposY + MTVs.get(h)[1];
            double magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
            if (Double.isNaN(magnitude)) magnitude = 0.0;
            double nX = MTVs.get(h)[0] / magnitude;
            double nY = MTVs.get(h)[1] / magnitude;
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            //ensure the objects are actually moving towards each other
            if ((vX - getVX(collidingIDs.get(h))) * nX + (vY - getVY(collidingIDs.get(h))) * nY > 0.0) {
                calcMotionFreeMove(dt, false);
                continue;
            }
            if (collidingIDs.get(h) == -1 || !getIsMovable(collidingIDs.get(h))) {
                calcMotionInfiniteMass(h, dt);
                continue;
            }
            double rX = contactPoints.get(h)[0] - newposX;
            double rY = contactPoints.get(h)[1] - newposY;
            double r = Math.sqrt(rX * rX + rY * rY);
            if (Double.isNaN(r)) r = 0.0;
            //the normal here is assumed to point towards this rigidbody and away from the other. For all intents and purposes,
            //this choice should not matter so long as it is treated consistently
            double rPerp1x = -rY;
            double rPerp1y = rX;
            if (!Double.isNaN(MTVs.get(h)[0]) && !Double.isNaN(contactPoints.get(h)[0])) {
                //use parallel axis theorem with angular momentum to determine rotation about the point and apply.
                //Here, the translational velocity is split into along the normal (for rotation) and tangent to it (for sliding).
                magnitude = newvX * nX + newvY * nY;
                double ndotVx = magnitude * nX;
                double ndotVy = magnitude * nY;
                magnitude = newvX * -nY + newvY * nX;
                double tdotVx = magnitude * -nY;
                double tdotVy = magnitude * nX;
                //calculate the amount to pivot by
                double temp = aX * nX + aY * nY;
                double ndotAx = temp * nX;
                double ndotAy = temp * nY;
                if (temp > 0.0) {
                    ndotAx = 0.0;
                    ndotAy = 0.0;
                }
                double pivotAngularA = (inertia * angularA + mass * (ndotAx * rY + ndotAy * -rX)) / (inertia + mass * r * r);
                double pivotAngularV = ((inertia * newangularV + mass * (ndotVx * rY + ndotVy * -rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
                pivotAboutPoint(pivotAngularV * dt, contactPoints.get(h)[0], contactPoints.get(h)[1]);
                newposX = newposX + tdotVx * dt;
                newposY = newposY + tdotVy * dt;

                //next, update the velocity in an impulse-based reaction model

                double rPerp2x = 0.0;
                double rPerp2y = 0.0;
                if (collidingIDs.get(h) <= -2 && mod(collidingIDs.get(h), 2) == 1) {
                    double radius = Point.get(((-collidingIDs.get(h) - 1) / 2) - 1).getSolidRadius();
                    rPerp2x = -nY * radius;
                    rPerp2y = nX * radius;
                }
                else {
                    rPerp2x = -(contactPoints.get(h)[1] - getPosY(collidingIDs.get(h)));
                    rPerp2y = contactPoints.get(h)[0] - getPosX(collidingIDs.get(h));
                }
                double jr = (newvX + rPerp1x * newangularV - getVX(collidingIDs.get(h)) - rPerp2x * getAngularV(collidingIDs.get(h))) * nX + (newvY + rPerp1y * newangularV - getVY(collidingIDs.get(h)) - rPerp2y * getAngularV(collidingIDs.get(h))) * nY;
                jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(h)));
                double temp1 = rPerp1x * nX + rPerp1y * nY;
                double temp2 = rPerp2x * nX + rPerp2y * nY;
                jr = jr / ((1.0 / mass) + (1.0 / getMass(collidingIDs.get(h))) + (1.0 / inertia) * temp1 * temp1 + (1.0 / getInertia(collidingIDs.get(h))) * temp2 * temp2);
                double vtrel = (newvX + rPerp1x * newangularV) * -nY + (newvY + rPerp1y * newangularV) * nX;
                vtrel = vtrel - ((getVX(collidingIDs.get(h)) + rPerp2x * getAngularV(collidingIDs.get(h))) * -nY + (getVY(collidingIDs.get(h)) + rPerp2y * getAngularV(collidingIDs.get(h))) * nX);
                temp1 = -temp;
                double friction = 0.0;
                if (temp1 > 0.0) {
                    if (vtrel > -CLAMP_LIMIT && vtrel < CLAMP_LIMIT && Math.abs(aX * -nY + aY * nX) <= getCOEFFICIENT_OF_FRICTION_STATIC(collidingIDs.get(h)) * temp1) friction = mass * (aX * -nY + aY * nX) * -Math.signum(vtrel);
                    else friction = mass * getCOEFFICIENT_OF_FRICTION_DYNAMIC(collidingIDs.get(h)) * temp1 * -Math.signum(vtrel);
                }
                double fx = friction * -nY * dt;
                double fy = friction * nX * dt;
                double adotTx = aX - ndotAx;
                double adotTy = aY - ndotAy;

                newvX = newvX + (jr / mass) * nX + (fx / mass) + adotTx * dt;
                newvY = newvY + (jr / mass) * nY + (fy / mass) + adotTy * dt;
                if (!lockedRotation) newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + ((fx * rPerp1x + fy * rPerp1y) / inertia) + angularA * dt;

                if (collidingIDs.get(h) <= -2 && mod(collidingIDs.get(h), 2) == 1) {
                    int tempIndex = ((-collidingIDs.get(h) - 1) / 2) - 1;
                    int softbodyIndex = Point.get(tempIndex).parentSoftbody;
                    int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(tempIndex);
                    Point point1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex));
                    Point point2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % Softbody.get(softbodyIndex).boundarySize()));
                    double magnitude2 = getAX(collidingIDs.get(h)) * nX + getAY(collidingIDs.get(h)) * nY;
                    double ndotAx2 = magnitude2 * nX;
                    double ndotAy2 = magnitude2 * nY;
                    if (magnitude < 0.0) {
                        ndotAx2 = 0.0;
                        ndotAy2 = 0.0;
                    }
                    double tdotAx2 = getAX(collidingIDs.get(h)) - ndotAx2;
                    double tdotAy2 = getAY(collidingIDs.get(h)) - ndotAy2;
                    double update = -(jr / getMass(collidingIDs.get(h))) * nX - (fx / getMass(collidingIDs.get(h))) + tdotAx2 * dt;
                    point1.changeVX(update);
                    point2.changeVX(update);
                    update = -(jr / getMass(collidingIDs.get(h))) * nY - (fy / getMass(collidingIDs.get(h))) + tdotAy2 * dt;
                    point1.changeVY(update);
                    point2.changeVY(update);

                    magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
                    double multiplier = -((magnitude - MTV_EPSILON) / magnitude) * (mass / (getMass(collidingIDs.get(h))));
                    multiplier *= 1.0 + (MTV_EPSILON / Math.abs(multiplier));
                    point1.changeX(MTVs.get(h)[0] * multiplier);
                    point2.changeX(MTVs.get(h)[0] * multiplier);
                    point1.changeY(MTVs.get(h)[1] * multiplier);
                    point2.changeY(MTVs.get(h)[1] * multiplier);
                }
            }
        }
        if (intersecting) dt = dt * contactPoints.size();

        if (!intersecting && !mouseHold)  {
            calcMotionFreeMove(dt, true);
        }

        newaX = initialExternalForces[0];
        newaY = initialExternalForces[1];
        if (!lockedRotation) newangularA = initialExternalTorque;
        if (universalGravity) {
            double[] results = calculateGravity();
            newaX += results[0];
            newaY += results[1];
        }
        if (airResistance || buoyancy) {
            calculateAirResistance();
        }

        if (mouseHold) {
            double vmX = (currentMouseX - lastMouseX) / dt;
            double vmY = (currentMouseY - lastMouseY) / dt;
            double rX = newposX - currentMouseX;
            double rY = newposY - currentMouseY;
            double r = Math.sqrt(rX * rX + rY * rY);
            double pivotAngularA = (inertia * newangularA + mass * (aX * -rY + aY * rX)) / (inertia + mass * r * r);
            double pivotAngularV = (inertia * newangularV + mass * ((newvX - vmX) * -rY + (newvY - vmY) * rX)) / (inertia + mass * r * r) + pivotAngularA * dt;
            newposX = newposX + (currentMouseX - lastMouseX);
            newposY = newposY + (currentMouseY - lastMouseY);
            pivotAboutPoint(pivotAngularV * dt, currentMouseX, currentMouseY);


            double damping = Math.min(Math.abs(newangularV), HOLD_DAMPING) * -Math.signum(newangularV);
            newangularV = newangularV + (damping * dt);
            double adotX = (aX * -rY + aY * rX) * (-rY / (r * r));
            double adotY = (aX * -rY + aY * rX) * (rX / (r * r));
            double vdotX = (newvX * -rY + newvY * rX) * (-rY / (r * r));
            double vdotY = (newvX * -rY + newvY * rX) * (rX / (r * r));
            newvX = vdotX + adotX * dt;
            newvY = vdotY + adotY * dt;
        }
        if (mouseRelease) {
            newvX = newvX + flingX;
            newvY = newvY + flingY;
            mouseRelease = false;
        }


    }
    private boolean checkForCollisionsBroad(Rigidbody otherObject) {
        boolean results = false;
        double actualDistance = (posX - otherObject.getPosX()) * (posX - otherObject.getPosX()) + (posY - otherObject.getPosY()) * (posY - otherObject.getPosY());
        actualDistance = Math.sqrt(actualDistance);
        if (Double.isNaN(actualDistance)) actualDistance = 0.0;
        if (actualDistance <= largestDistance + otherObject.getLargestDistance()) {
            if (otherObject.isAABB(leftBoundBox + posX,rightBoundBox + posX,topBoundBox + posY,bottomBoundBox + posY)) {
                results = checkForCollisionsNarrow(otherObject);
            }
        }
        return(results);
    }
    private boolean checkForCollisionsNarrow(Rigidbody otherObject) {
        boolean intersecting = false;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            for (int j = 0; j < otherObject.getNumPoints(); j = j + 1) {
                if (triangles.get(i).doesExist() && otherObject.getTriangles().get(j).doesExist()){
                    Triplet results = triangles.get(i).checkCollisions(otherObject.getTriangles().get(j));
                    if (results.getFirstBoolean()) {
                        intersecting = true;
                        contactPoints.add(results.getThirdDoubleArrayReference());
                        MTVs.add(results.getSecondDoubleArrayReference());
                        collidingIDs.add(otherObject.ID);
                    }
                }
            }
        }
        return(intersecting);
    }
    private boolean checkForCollisionsBroad(Point otherPoint) {
        boolean results = false;
        double radius = otherPoint.getSolidRadius();
        double actualDistance = (newposX - otherPoint.getX()) * (newposX - otherPoint.getX()) + (newposY - otherPoint.getY()) * (newposY - otherPoint.getY());
        actualDistance = Math.sqrt(actualDistance);
        if (Double.isNaN(actualDistance)) actualDistance = 0.0;
        if (actualDistance <= largestDistance + otherPoint.getSolidRadius()) {
            if (isAABB(otherPoint.getX() - radius, otherPoint.getX() + radius, otherPoint.getY() - radius, otherPoint.getY() + radius)) {
                results = checkForCollisionsNarrow(otherPoint);
            }
        }
        return(results);
    }
    private boolean checkForCollisionsNarrow(Point otherPoint) {
        boolean intersecting = false;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            if (triangles.get(i).doesExist()) {
                Triplet results = triangles.get(i).checkCollisions(otherPoint);
                if (results.getFirstBoolean()) {
                    intersecting = true;
                    contactPoints.add(results.getThirdDoubleArrayReference());
                    MTVs.add(results.getSecondDoubleArrayReference());
                    collidingIDs.add(otherPoint.ID * -2 - 2);
                }
            }
        }
        return(intersecting);
    }
    private boolean checkForCollisionsBroad(Softbody softbody) {
        if (!softbody.boundaryCollision) return(false);
        boolean intersecting = false;
        double dx = softbody.cM[0] - posX;
        double dy = softbody.cM[1] - posY;
        if (dx * dx + dy * dy <= softbody.largestSquaredDistance + getLargestDistance()) {
            if (leftBoundBox + posX < softbody.maxX && rightBoundBox + posX > softbody.minX) {
                if (topBoundBox + posY < softbody.maxY && bottomBoundBox + posY > softbody.minY) {
                    intersecting = checkForCollisionsNarrow(softbody);
                }
            }
        }
        return(intersecting);
    }
    private boolean checkForCollisionsNarrow(Softbody softbody) {
        boolean intersecting = false;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            Triplet results = softbody.resolvePointInside(new double[]{xPoints.get(i) + posX, yPoints.get(i) + posY}, 0.0);
            if (results.getFirstBoolean()) {
                Double[] MTV = new Double[2];
                double multiplier = (getMass(results.getThirdInt()) / (getMass(results.getThirdInt()) + mass));
                MTV[0] = results.getSecondDoubleArray()[0] * multiplier;
                MTV[1] = results.getSecondDoubleArray()[1] * multiplier;
                Double[] contactPoint = new Double[]{xPoints.get(i) + posX + MTV[0], yPoints.get(i) + posY + MTV[1]};
                contactPoints.add(contactPoint);
                MTVs.add(MTV);
                collidingIDs.add(results.getThirdInt());
                intersecting = true;
            }
        }
        return(intersecting);
    }
    private boolean checkForCollisionsWall() {
        boolean intersecting = false;
        boolean wallIntersection = false;
        double left = 0.0;
        double right = 0.0;
        double bottom = 0.0;
        double top = 0.0;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            double x = xPoints.get(i) + posX;
            double y = yPoints.get(i) + posY;
            if (y >= worldBottomBound) {
                if (Math.abs(worldBottomBound - y) > Math.abs(bottom)){
                    bottom = worldBottomBound - y;
                    MTVs.add(new Double[]{0.0, bottom - MTV_EPSILON});
                    contactPoints.add(new Double[]{x, worldBottomBound});
                    collidingIDs.add(-1);
                    intersecting = true;
                }
            }
            if (y <= worldTopBound) {
                if (Math.abs(worldTopBound - y) > Math.abs(top)){
                    top = worldTopBound - y;
                    MTVs.add(new Double[]{0.0, top + MTV_EPSILON});
                    contactPoints.add(new Double[]{x, worldTopBound});
                    collidingIDs.add(-1);
                    intersecting = true;
                }
            }
            if (x <= worldLeftBound) {
                if (Math.abs(worldLeftBound - x) > Math.abs(left)){
                    left = worldLeftBound - x;
                    MTVs.add(new Double[]{left + MTV_EPSILON, 0.0});
                    contactPoints.add(new Double[]{worldLeftBound, y});
                    collidingIDs.add(-1);
                    intersecting = true;
                }
            }
            if (x >= worldRightBound) {
                if (Math.abs(worldRightBound - x) > Math.abs(right)){
                    right = worldRightBound - x;
                    MTVs.add(new Double[]{right - MTV_EPSILON, 0.0});
                    contactPoints.add(new Double[]{worldRightBound, y});
                    collidingIDs.add(-1);
                    intersecting = true;
                }
            }
        }
        return(intersecting);
    }
    private void calcMotionInfiniteMass(int index, double dt) {
        double rX = contactPoints.get(index)[0] - newposX;
        double rY = contactPoints.get(index)[1] - newposY;
        double r = Math.sqrt(rX * rX + rY * rY);
        double rPerp1x = -rY;
        double rPerp1y = rX;
        double magnitude = Math.sqrt(MTVs.get(index)[0] * MTVs.get(index)[0] + MTVs.get(index)[1] * MTVs.get(index)[1]);
        double nX = MTVs.get(index)[0] / magnitude;
        double nY = MTVs.get(index)[1] / magnitude;
        if (Double.isNaN(nX) || Double.isNaN(nY)) {
            nX = 0.0;
            nY = 0.0;
        }
        magnitude = newvX * nX + newvY * nY;
        double ndotVx = magnitude * nX;
        double ndotVy = magnitude * nY;
        magnitude = newvX * -nY + newvY * nX;
        double tdotVx = magnitude * -nY;
        double tdotVy = magnitude * nX;
        //calculate the amount to pivot by
        double temp = aX * nX + aY * nY;
        double ndotAx = temp * nX;
        double ndotAy = temp * nY;
        if (temp > 0.0) {
            ndotAx = 0.0;
            ndotAy = 0.0;
        }
        double pivotAngularA = (inertia * angularA + mass * (ndotAx * -rY + ndotAy * rX)) / (inertia + mass * r * r);
        double pivotAngularV = ((inertia * newangularV + mass * (ndotVx * -rY + ndotVy * rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
        pivotAboutPoint(pivotAngularV * dt, contactPoints.get(index)[0], contactPoints.get(index)[1]);
        newposX = newposX + tdotVx * dt;
        newposY = newposY + tdotVy * dt;

        double rPerp2x = -(contactPoints.get(index)[1] - Rigidbody.getPosY(collidingIDs.get(index)));
        double rPerp2y = contactPoints.get(index)[0] - Rigidbody.getPosX(collidingIDs.get(index));
        if (Double.isNaN(rPerp2x)) {
            rPerp2x = 0.0;
            rPerp2y = 0.0;
        }
        double jr = (newvX + rPerp1x * newangularV - getVX(collidingIDs.get(index)) - rPerp2x * getAngularV(collidingIDs.get(index))) * nX + (newvY + rPerp1y * newangularV - getVY(collidingIDs.get(index)) - rPerp2y * getAngularV(collidingIDs.get(index))) * nY;
        jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(index)));
        double temp1 = rPerp1x * nX + rPerp1y * nY;
        jr = jr / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);
        double vtrel = (newvX + rPerp1x * newangularV) * -nY + (newvY + rPerp1y * newangularV) * nX;
        vtrel = vtrel - ((getVX(collidingIDs.get(index)) + rPerp2x * getAngularV(collidingIDs.get(index))) * -nY + (getVY(collidingIDs.get(index)) + rPerp2y * getAngularV(collidingIDs.get(index))) * nX);
        double temp2 = -temp;
        double friction = 0.0;
        if (temp2 > 0.0) {
            if (vtrel > -CLAMP_LIMIT && vtrel < CLAMP_LIMIT && Math.abs(aX * -nY + aY * nX) <= getCOEFFICIENT_OF_FRICTION_STATIC(collidingIDs.get(index)) * temp2) friction = mass * (aX * -nY + aY * nX) * -Math.signum(vtrel);
            else friction = mass * getCOEFFICIENT_OF_FRICTION_DYNAMIC(collidingIDs.get(index)) * temp2 * -Math.signum(vtrel);
        }
        double fx = friction * -nY * dt;
        double fy = friction * nX * dt;
        double adotTx = aX - ndotAx;
        double adotTy = aY - ndotAy;

        newvX = newvX + (jr / mass) * nX + (fx / mass) + adotTx * dt;
        newvY = newvY + (jr / mass) * nY + (fy / mass) + adotTy * dt;
        if (!lockedRotation) {
            newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + ((fx * rPerp1x + fy * rPerp1y) / inertia) + angularA * dt;
        }

    }
    private void calcMotionFreeMove(double dt, boolean canUpdatePosition) {
        if (canUpdatePosition) {
            //first update linear position and rotational position
            newposX = posX + vX * dt + 0.5 * aX * dt * dt;
            newposY = posY + vY * dt + 0.5 * aY * dt * dt;
            pivotAboutPoint(angularV * dt + 0.5 * angularA * dt * dt, posX, posY);
            newvX = vX + aX * dt;
            newvY = vY + aY * dt;
            newangularV = angularV + angularA * dt;
            //then update linear and angular velocity
            //finally, update linear and angular acceleration (constant for now)
            newaX = aX;
            newaY = aY;
            newangularA = angularA;
        }
        else {
            //first update linear position and rotational position
            newposX = newposX + newvX * dt + 0.5 * aX * dt * dt;
            newposY = newposY + newvY * dt + 0.5 * aY * dt * dt;
            pivotAboutPoint(newangularV * dt + 0.5 * angularA * dt * dt, newposX, newposY);
            newvX = newvX + aX * dt;
            newvY = newvY + aY * dt;
            newangularV = newangularV + angularA * dt;
        }
    }
    private void updateMotion(double dt) {
        newposX += otherUpdatePosX;
        newposY += otherUpdatePosY;
        newvX += otherUpdateVX;
        newvY += otherUpdateVY;
        newaX += otherUpdateAX;
        newaY += otherUpdateAY;
        //clamp some values
        if (newvX * newvX + newvY * newvY < CLAMP_LIMIT * CLAMP_LIMIT) {
            newvX = 0.0;
            newvY = 0.0;
        }
        if (mass / area >= 1.5 * AIR_DENSITY && newangularV != 0.0 && Math.abs(newangularV) * largestDistance < CLAMP_LIMIT) {
            newangularV = 0.0;
        }
        if (!Double.isNaN(newposX)) posX = newposX;
        if (!Double.isNaN(newposY)) posY = newposY;
        if (!Double.isNaN(newvX)) vX = newvX;
        if (!Double.isNaN(newvY))vY = newvY;
        if (!Double.isNaN(newaX)) aX = newaX;
        if (!Double.isNaN(newaY)) aY = newaY;
        if (!Double.isNaN(newangularV)) angularV = newangularV;
        if (!Double.isNaN(newangularA)) angularA = newangularA;

        otherUpdatePosX = 0.0;
        otherUpdatePosY = 0.0;
        otherUpdateVX = 0.0;
        otherUpdateVY = 0.0;
        otherUpdateAX = 0.0;
        otherUpdateAY = 0.0;

        if (!controllers.isEmpty()) {
            //take into account controllers first once 'onGround' is known
            boolean onGround = false;
            double[] maxGroundVelocity = new double[]{0.0, 0.0};
            double maxGroundVelocityMagnitude = 0.0;
            for (int i = 0; i < collidingIDs.size(); i = i + 1) {
                if (Rigidbody.getMaterial(collidingIDs.get(i)).name.contains("Ground")) {
                    if (contactPoints.get(i)[0] * initialExternalForces[0] + contactPoints.get(i)[1] * initialExternalForces[1] > posX * initialExternalForces[0] + posY * initialExternalForces[1]) {
                        onGround = true;
                        double gvX = Rigidbody.getVX(collidingIDs.get(i));
                        double gvY = Rigidbody.getVY(collidingIDs.get(i));
                        double magnitude = Math.sqrt(gvX * gvX + gvY * gvY);
                        if (magnitude > maxGroundVelocityMagnitude) {
                            maxGroundVelocityMagnitude = magnitude;
                            maxGroundVelocity[0] = gvX;
                            maxGroundVelocity[1] = gvY;
                        }
                    }
                }
            }
            for (int i = 0; i < controllers.size(); i = i + 1) {
                controllers.get(i).respondToKey(dt, Simulation.get(simID).display.keysCache, Simulation.get(simID).display.firstPress, Simulation.get(simID).display.keyReleasedFirstTime, onGround, maxGroundVelocity);
            }
        }
    }
    private double[] calculateGravity() {
        double sumaX = 0.0;
        double sumaY = 0.0;
        for (int i = 0; i < num; i = i + 1) {
            if (i != ID && Rigidbody.get(i).simID == simID) {
                double rSquared = (posX - Rigidbody.get(i).getPosX()) * (posX - Rigidbody.get(i).getPosX()) + (posY - Rigidbody.get(i).getPosY()) * (posY - Rigidbody.get(i).getPosY());
                if (rSquared > 0.0) {
                    double r = Math.sqrt(rSquared);
                    double magnitude = (GRAVITATIONAL_CONSTANT * Rigidbody.get(i).getMass()) / (rSquared);
                    sumaX = sumaX + (magnitude / r) * (Rigidbody.get(i).getPosX() - posX);
                    sumaY = sumaY + (magnitude / r) * (Rigidbody.get(i).getPosY() - posY);
                }
            }
        }
        for (int i = 0; i < Point.num; i = i + 1) {
            if (Point.get(i).simID != simID) continue;
            double rSquared = (posX - Point.get(i).getX()) * (posX - Point.get(i).getX()) + (posY - Point.get(i).getY()) * (posY - Point.get(i).getY());
            if (rSquared > 0.0) {
                double r = Math.sqrt(rSquared);
                double magnitude = (GRAVITATIONAL_CONSTANT * Point.get(i).getMass()) / (rSquared);
                sumaX = sumaX + (magnitude / r) * (Point.get(i).getX() - posX);
                sumaY = sumaY + (magnitude / r) * (Point.get(i).getY() - posY);
            }
        }
        return(new double[]{sumaX, sumaY});
    }
    private void calculateAirResistance() {
        double[] fD = new double[]{0.0, 0.0};
        double torqueSum = 0.0;
        if (airResistance) {
            int length = xPoints.size();
            double magnitude1 = Math.sqrt(vX * vX + vY * vY);
            if (Double.isNaN(magnitude1)) magnitude1 = 0.0001;
            double uX = vX / magnitude1;
            double uY = vY / magnitude1;
            if (Double.isNaN(uX)) uX = 0.0;
            if (Double.isNaN(uY)) uY = 0.0;
            double crossArea = 0.0;
            for (int i = 0; i < length; i = i + 1) {
                double nX = -(yPoints.get((i + 1) % length) - yPoints.get(i));
                double nY = xPoints.get((i + 1) % length) - xPoints.get(i);
                if (Double.isNaN(nX) || Double.isNaN(nY)) {
                    nX = 0.0;
                    nY = 0.0;
                }
                if (invertNormals) {
                    nX = -nX;
                    nY = -nY;
                }
                if (nX * uX + nY * uY >= 0.0) {
                    crossArea = crossArea + Math.abs((xPoints.get((i + 1) % length) - xPoints.get(i)) * -uY + (yPoints.get((i + 1) % length) - yPoints.get(i)) * uX);
                }
            }
            double magnitude2 = 0.5 * AIR_DENSITY * crossArea * DRAG_COEFFICIENT * magnitude1 * magnitude1;
            fD = new double[]{-uX * magnitude2, -uY * magnitude2};
            for (int i = 0; i < length; i = i + 1) {
                double rPerpX = -yPoints.get(i);
                double rPerpY = xPoints.get(i);
                double r = Math.sqrt(rPerpX * rPerpX + rPerpY * rPerpY);
                if (Double.isNaN(r)) r = 0.0;
                double nX = -(yPoints.get((i + 1) % length) - yPoints.get(i));
                double nY = xPoints.get((i + 1) % length) - xPoints.get(i);
                if (Double.isNaN(nX) || Double.isNaN(nY)) {
                    nX = 0.0;
                    nY = 0.0;
                }
                if (invertNormals) {
                    nX = -nX;
                    nY = -nY;
                }
                if (nX * uX + nY * uY >= 0.0) {
                    double torque = 0.5 * Math.abs((xPoints.get((i + 1) % length) - xPoints.get(i)) * -uY + (yPoints.get((i + 1) % length) - yPoints.get(i)) * uX) / crossArea;
                    double perpForce1 = (fD[0] * rPerpX + fD[1] * rPerpY);
                    double rPerpX2 = -yPoints.get((i + 1) % length);
                    double rPerpY2 = xPoints.get((i + 1) % length);
                    double perpForce2 = (fD[0] * rPerpX2 + fD[1] * rPerpY2);
                    torque = torque * (perpForce1 + perpForce2);
                    torqueSum = torqueSum + torque;
                }
            }
        }

        double[] fB = new double[]{0.0, 0.0};
        if (buoyancy) {
            double magnitude1 = Math.sqrt(newaX * newaX + newaY * newaY);
            if (Double.isNaN(magnitude1)) magnitude1 = 0.0;
            double magnitude2 = AIR_DENSITY * magnitude1 * area;
            fB[0] = magnitude2 * (-newaX / magnitude1);
            fB[1] = magnitude2 * (-newaY / magnitude1);
        }

        double update = (fD[0] + fB[0]) / mass;
        if (!Double.isNaN(update)) newaX += update;
        update = (fD[1] + fB[1]) / mass;
        if (!Double.isNaN(update)) newaY += update;
        update = torqueSum / inertia;
        if (!Double.isNaN(update)) newangularA += update;
    }
    public static boolean moveByMouse(double x, double y, boolean reset, boolean mousePressed, double dt, int simID) {
        boolean output = false;
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (Rigidbody.get(i).simID == simID) {
                output = Rigidbody.get(i).localMoveByMouse(x, y, reset, mousePressed, dt);
                if (output) break;
            }
        }
        return(output);
    }
    private boolean localMoveByMouse(double x, double y, boolean reset, boolean mousePressed, double dt) {
        flingX = (x - lastMouseX) / dt;
        flingY = (y - lastMouseY) / dt;
        double multiplier = Math.sqrt(flingX * flingX + flingY * flingY);
        multiplier = Math.min(multiplier, FLING_SPEED_LIMIT) / multiplier;
        flingX = flingX * multiplier;
        if (Double.isNaN(flingX)) flingX = 0.0;
        flingY = flingY * multiplier;
        if (Double.isNaN(flingY)) flingY = 0.0;
        if (!reset) {
            lastMouseX = currentMouseX;
            lastMouseY = currentMouseY;
        }
        else {
            lastMouseX = x;
            lastMouseY = y;
        }
        currentMouseX = x;
        currentMouseY = y;
        boolean isInside = false;
        if (pointInside(new double[]{currentMouseX, currentMouseY})) {
            isInside = true;
            if (mousePressed) {
                mouseHold = true;
            }
            else {
                if (mouseHold) mouseRelease = true;
                mouseHold = false;
            }
        }
        else {
            mouseHold = false;
        }
        return (isInside);
    }
    private void pivotAboutPoint(double theta, double pointX, double pointY) {
        if (!lockedRotation) {
            double sin = Math.sin(theta);
            double cos = Math.cos(theta);
            double centerX = -(pointX - newposX);
            double centerY = -(pointY - newposY);
            double shiftX = centerX * cos - centerY * sin;
            double shiftY = centerX * sin + centerY * cos;
            for (int i = 0; i < xPoints.size(); i = i + 1) {
                double x = xPoints.get(i);
                x = x - (pointX - newposX);
                double y = yPoints.get(i);
                y = y - (pointY - newposY);
                xPoints.set(i, x * cos - y * sin - shiftX);
                if (Double.isNaN(leftBoundBox) || xPoints.get(i) < leftBoundBox) {
                    leftBoundBox = xPoints.get(i);
                }
                if (Double.isNaN(rightBoundBox) || xPoints.get(i) > rightBoundBox) {
                    rightBoundBox = xPoints.get(i);
                }
                yPoints.set(i, y * cos + x * sin - shiftY);
                if (Double.isNaN(topBoundBox) || yPoints.get(i) < topBoundBox) {
                    topBoundBox = yPoints.get(i);
                }
                if (Double.isNaN(bottomBoundBox) || yPoints.get(i) > bottomBoundBox) {
                    bottomBoundBox = yPoints.get(i);
                }
                if (triangles.get(i).doesExist()) {
                    triangles.get(i).shift(pointX - newposX, pointY - newposY);
                    triangles.get(i).rotate(theta);
                    triangles.get(i).shift(shiftX, shiftY);
                }
            }
            double x = newposX;
            double y = newposY;
            newposX = (x - pointX) * cos - (y - pointY) * sin + pointX;
            newposY = (y - pointY) * cos + (x - pointX) * sin + pointY;
        }
    }


    private boolean isOnNormalSide(double[] point, int index1, int index2) {
        double dotProduct = (yPoints.get(index1) - point[1]) * (xPoints.get(index2) - xPoints.get(index1));
        dotProduct = dotProduct - (xPoints.get(index1) - point[0]) * (yPoints.get(index2) - yPoints.get(index1));
        if ((dotProduct > 0.0 && invertNormals) || (dotProduct < 0.0 && !invertNormals)) return(true);
        else return(false);
    }
    private boolean pointInside(double[] point) {
        double[] testPoint = new double[]{point[0] - posX, point[1] - posY};
        int raycastCount = 0;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            double minX = xPoints.get(i);
            int minXindex = i;
            double maxX = xPoints.get((i + 1) % xPoints.size());
            int maxXindex = (i + 1) % xPoints.size();
            if (maxX < minX) {
                double temp = minX;
                int tempIndex = minXindex;
                minX = maxX;
                minXindex = maxXindex;
                maxX = temp;
                maxXindex = tempIndex;
            }
            if (testPoint[0] > minX && testPoint[0] < maxX && ((yPoints.get(maxXindex) - yPoints.get(minXindex)) / (maxX - minX)) * (testPoint[0] - maxX) + yPoints.get(maxXindex) > testPoint[1]) {
                raycastCount = raycastCount + 1;
            }
        }
        return (raycastCount % 2 == 1);
    }
    public void lockRotation(boolean lockRotation) {
        lockedRotation = lockRotation;
    }
    public void makeAdoptOtherSurfaceOnly(boolean a) {
        adoptOnlyOtherSurface = a;
    }
    public static int mod(int a, int b) {
        if (a >= 0) return(a % b);
        else {
            int tempResult = b - (-a % b);
            if (tempResult == b) return(0);
            else return(tempResult);
        }
    }

    public static Rigidbody get(int index) {
        return(rigidbodies.get(index));
    }
    public int[] getDrawX(double shiftX, double resolutionScaling, double resolutionCenterX, double pixelShiftX) {
        int[] x = new int[xPoints.size()];
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            x[i] = (int) Math.round(((xPoints.get(i) + posX - shiftX - resolutionCenterX) / resolutionScaling) + resolutionCenterX + pixelShiftX);
        }
        return(x);
    }
    public int[] getDrawY(double shiftY, double resolutionScaling, double resolutionCenterY, double pixelShiftY) {
        int[] y = new int[yPoints.size()];
        for (int i = 0; i < yPoints.size(); i = i + 1) {
            y[i] = (int) Math.round(((yPoints.get(i) + posY - shiftY - resolutionCenterY) / resolutionScaling) + resolutionCenterY + pixelShiftY);
        }
        return(y);
    }
    public double getPosX() {
        return(posX);
    }
    public double getPosY() {
        return(posY);
    }
    public void setPosX(double posX) {
        this.posX = posX;
    }
    public void setPosY(double posY) {
        this.posY = posY;
    }
    public double getCX() {
        return(cX);
    }
    public double getCY() {
        return(cY);
    }
    public double getVX() {
        return(vX);
    }
    public double getVY() {
        return(vY);
    }
    public double getAX() {
        return(aX);
    }
    public double getAY() {
        return(aY);
    }
    public double getAngularV() {
        return(angularV);
    }
    public double getAngularA() {
        return(angularA);
    }
    public Color getColor() {
        return(color);
    }
    public int getNumPoints() {
        if (xPoints.size() == yPoints.size()) return(xPoints.size());
        else {
            System.out.println("xPoints and yPoints do not match in length");
            return (-1);
        }
    }
    public int getID() {
        return(ID);
    }
    public double getXPoints(int index) {
        return(xPoints.get(index));
    }
    public ArrayList<Double> getXPoints() {
        return(xPoints);
    }
    public double getYPoints(int index) {
        return(yPoints.get(index));
    }
    public ArrayList<Double> getYPoints() {
        return(yPoints);
    }
    public ArrayList<Triangle> getTriangles() {
        return(triangles);
    }

    public double getMass() {
        return(mass);
    }
    public double getInertia() {
        return(inertia);
    }
    public double getArea() {
        return(area);
    }
    public double getLargestDistance() {
        return(largestDistance);
    }
    public boolean isAABB(double leftBoundBox, double rightBoundBox, double topBoundBox, double bottomBoundBox)  {
        return (this.leftBoundBox + posX <= rightBoundBox && leftBoundBox <= this.rightBoundBox + posX && this.topBoundBox + posY <= bottomBoundBox && topBoundBox <= this.bottomBoundBox + posY);
    }
    public boolean isMovable() {
        return(isMovable);
    }
    public void setIsMovable(boolean a) {
        isMovable = a;
    }

    public static double getPosX(int index) {
        if (index >= 0) return(Rigidbody.get(index).getPosX());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getX());
        else if (index <= -2 && mod(index, 2) == 1) {
            System.out.println("Calling the position of a softbody edge collision should never happen.");
            return(Double.NaN);
        }
        else return(Double.NaN);
    }
    public static double getPosY(int index) {
        if (index >= 0) return(Rigidbody.get(index).getPosY());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getY());
        else if (index <= -2 && mod(index, 2) == 1) {
            System.out.println("This has to be altered specifically.");
            return(Double.NaN);
        }
        else return(Double.NaN);
    }
    public static double getVX(int index) {
        if (index >= 0) return(Rigidbody.get(index).getVX());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getVX());
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getVX();
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getVX();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
    }
    public static double getVY(int index) {
        if (index >= 0) return(Rigidbody.get(index).getVY());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getVY());
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getVY();
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getVY();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
    }
    public static double getAX(int index) {
        if (index >= 0) return(Rigidbody.get(index).getVX());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getVX());
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getAX();
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getAX();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
    }
    public static double getAY(int index) {
        if (index >= 0) return(Rigidbody.get(index).getVY());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getVY());
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getAY();
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getAY();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
    }
    public static double getAngularV(int index) {
        if (index >= 0) return(Rigidbody.get(index).getAngularV());
        else return(0.0);
    }
    public static double getMass(int index) {
        if (index >= 0) return(Rigidbody.get(index).getMass());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getMass());
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getMass();
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getMass();
            return(0.5 * (val1 + val2));
        }
        else return(Double.NaN);
    }
    public static double getInertia(int index) {
        if (index >= 0) return(Rigidbody.get(index).getInertia());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).getInertia());
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getInertia();
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getInertia();
            return(0.5 * (val1 + val2));
        }
        else return(Double.NaN);
    }
    public static Material getMaterial(int index) {
        try {
            Material returnMaterial;
            if (index >= 0) returnMaterial = Simulation.get(Rigidbody.get(index).simID).getObject("Rigidbody", index).material;
            else if (index <= -2 && mod(index, 2) == 0) returnMaterial = Simulation.get(Point.get((-index / 2) - 1).simID).getObject("Point", (-index / 2) - 1).material;
            else if (index <= -2 && mod(index, 2) == 1) {
                int tempIndex = (-index - 1) / 2 - 1;
                returnMaterial = Simulation.get(Point.get(tempIndex).simID).getObject("Point", tempIndex).material;
            }
            else returnMaterial = Simulation.defaultMaterial;
            if (returnMaterial != null) return (returnMaterial);
            else {
                System.out.println("Material of that object is unassigned, so the default was assumed.");
                return(Simulation.defaultMaterial);
            }
        }
        catch (Exception e) {
            System.out.println(e);
        }
        return(null);
    }
    public double getCOEFFICIENT_OF_RESTITUTION(int index) {
        if (index >= 0) {
            double a = Rigidbody.get(index).COEFFICIENT_OF_RESTITUTION;
            double b = COEFFICIENT_OF_RESTITUTION;
            if (adoptOnlyOtherSurface) return(a);
            else return(Math.sqrt((a * a + b * b) * 0.5));
        }
        else if (index <= -2 && mod(index, 2) == 0) {
            double a = Point.get((-index / 2) - 1).COEFFICIENT_OF_RESTITUTION;
            double b = COEFFICIENT_OF_RESTITUTION;
            if (adoptOnlyOtherSurface) return(a);
            else return(Math.sqrt((a * a + b * b) * 0.5));
        }
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).COEFFICIENT_OF_RESTITUTION;
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).COEFFICIENT_OF_RESTITUTION;
            double a = 0.5 * (val1 + val2);
            double b = COEFFICIENT_OF_RESTITUTION;
            if (adoptOnlyOtherSurface) return(a);
            else return(Math.sqrt((a * a + b * b) * 0.5));
        }
        else {
            double a = Simulation.get(simID).COEFFICIENT_OF_RESTITUTION;
            double b = COEFFICIENT_OF_RESTITUTION;
            if (adoptOnlyOtherSurface) return(a);
            else return(Math.sqrt((a * a + b * b) * 0.5));
        }
    }
    public double getCOEFFICIENT_OF_FRICTION_DYNAMIC(int index) {
        if (index >= 0) {
            if (adoptOnlyOtherSurface) return(Rigidbody.get(index).COEFFICIENT_OF_FRICTION_DYNAMIC);
            else return((Rigidbody.get(index).COEFFICIENT_OF_FRICTION_DYNAMIC + COEFFICIENT_OF_FRICTION_DYNAMIC) * 0.5);
        }
        else if (index <= -2 && mod(index, 2) == 0) {
            if (adoptOnlyOtherSurface) return(Point.get((-index / 2) - 1).COEFFICIENT_OF_FRICTION_DYNAMIC);
            else return((Point.get((-index / 2) - 1).COEFFICIENT_OF_FRICTION_DYNAMIC + COEFFICIENT_OF_FRICTION_DYNAMIC) * 0.5);
        }
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).COEFFICIENT_OF_FRICTION_DYNAMIC;
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).COEFFICIENT_OF_FRICTION_DYNAMIC;
            double a = 0.5 * (val1 + val2);
            if (adoptOnlyOtherSurface) return(a);
            else return(0.5 * (a + COEFFICIENT_OF_FRICTION_DYNAMIC));
        }
        else {
            if (adoptOnlyOtherSurface) return(Simulation.get(simID).COEFFICIENT_OF_FRICTION_DYNAMIC);
            else return(Simulation.get(simID).COEFFICIENT_OF_FRICTION_DYNAMIC + COEFFICIENT_OF_FRICTION_DYNAMIC * 0.5);
        }
    }
    public double getCOEFFICIENT_OF_FRICTION_STATIC(int index) {
        if (index >= 0) {
            if (adoptOnlyOtherSurface) return(Rigidbody.get(index).COEFFICIENT_OF_FRICTION_STATIC);
            else return((Rigidbody.get(index).COEFFICIENT_OF_FRICTION_STATIC + COEFFICIENT_OF_FRICTION_STATIC) * 0.5);
        }
        else if (index <= -2 && mod(index, 2) == 0) {
            if (adoptOnlyOtherSurface) return(Point.get(-index - 2).COEFFICIENT_OF_FRICTION_STATIC);
            else return((Point.get((-index / 2) - 1).COEFFICIENT_OF_FRICTION_STATIC + COEFFICIENT_OF_FRICTION_STATIC) * 0.5);
        }
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).COEFFICIENT_OF_FRICTION_STATIC;
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).COEFFICIENT_OF_FRICTION_STATIC;
            double a = 0.5 * (val1 + val2);
            if (adoptOnlyOtherSurface) return(a);
            else return(0.5 * (a + COEFFICIENT_OF_FRICTION_STATIC));
        }
        else {
            if (adoptOnlyOtherSurface) return(Simulation.get(simID).COEFFICIENT_OF_FRICTION_STATIC);
            else return(Simulation.get(simID).COEFFICIENT_OF_FRICTION_STATIC + COEFFICIENT_OF_FRICTION_STATIC * 0.5);
        }
    }
    public static boolean getIsMovable(int index) {
        if (index >= 0) return(Rigidbody.get(index).isMovable());
        else if (index <= -2 && mod(index, 2) == 0) return(Point.get((-index / 2) - 1).isMovable());
        else if (index <= -2 && mod(index, 2) == 1) return(Point.get(((- index - 1) / 2) - 1).isMovable());
        else return(false);
    }
}
