package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;

class Point {
    public int simID;

    protected double mass;
    private final double area;
    protected double inertia;
    private Color color;
    private final double radius;
    private final double repulseRadius;
    protected final ArrayList<Controller> controllers = new ArrayList<>();
    private static final ArrayList<Point> points = new ArrayList<>();
    private final ArrayList<Double[]> contactPoints = new ArrayList<>();
    private final ArrayList<Double[]> MTVs = new ArrayList<>();
    protected final ArrayList<Integer> collidingIDs = new ArrayList<>();
    //same collidingIDs convention as Rigidbody
    private final ArrayList<Integer> attachedIndices = new ArrayList<>();
    private final ArrayList<Double> idealSpringLengths = new ArrayList<>();
    public int parentSoftbody = -1;
    public static int num = 0;
    public int ID = 0;

    public double COEFFICIENT_OF_RESTITUTION = 0.75;
    public double COEFFICIENT_OF_FRICTION_DYNAMIC = 0.35;
    public double COEFFICIENT_OF_FRICTION_STATIC = 0.5;
    public double GRAVITATIONAL_CONSTANT = 10.0;
    public double AIR_DENSITY = 0.01204;
    public double DRAG_COEFFICIENT = 0.95;
    public double CLAMP_LIMIT = 0.0;
    public double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public double MTV_EPSILON = 0.00165;
    public double FLING_SPEED_LIMIT = 500.0;
    public double POINT_MIN_RADIUS = 2.5;
    public double HOOKE_SPRING_CONSTANT = 1.0;
    public double SPRING_DAMPING_COEFFICIENT = 1.0;
    public double SPRING_MAX_DIST_MULTIPLIER = 2.0;
    public double SPRING_MIN_DIST_MULTIPLIER = 0.0;
    public double REPULSION_STRENGTH = 0.0;
    public double REPULSE_RADIUS_MULTIPLIER = 5.0;

    public double worldTopBound = 0.0;
    public double worldBottomBound = 460.0;
    public double worldLeftBound = 0.0;
    public double worldRightBound = 485.0;
    private double lastMouseX = 250.0;
    private double lastMouseY = 250.0;
    private double currentMouseX = 250.0;
    private double currentMouseY = 250.0;
    private double flingX = 0.0;
    private double flingY = 0.0;
    private boolean mouseHold = false;
    private boolean mouseRelease = false;
    private boolean adoptOnlyOtherSurface = false;

    private boolean isMovable = true;
    protected boolean isHitbox = false;
    protected boolean draw = true;
    private final boolean isSolidBall;
    private boolean attached = false;
    public boolean futureBoundary = false;
    public int minIndex1 = -1;
    public int minIndex2 = -1;
    public boolean universalGravity = false;
    public boolean airResistance = false;
    public boolean buoyancy = false;
    public boolean bounds = true;

    protected double posX;
    protected double posY;
    protected double vX;
    protected double vY;
    protected double aX;
    protected double aY;

    private double newposX;
    private double newposY;
    private double newvX;
    private double newvY;
    private double newaX;
    private double newaY;
    private final double[] initialExternalForces = new double[2];

    private double otherUpdatePosX;
    private double otherUpdatePosY;
    private double otherUpdateVX;
    private int vUpdateCount = 0;
    private double otherUpdateVY;
    private double otherUpdateAX;
    private double otherUpdateAY;

    public Point(double[] motion, double radius, double mass, Color color, boolean isSolidBall, int simID) {
        this.simID = simID;
        ID = num;
        num = num + 1;
        posX = motion[0];
        posY = motion[1];
        vX = motion[2];
        vY = motion[3];
        initialExternalForces[0] = motion[4];
        initialExternalForces[1] = motion[5];
        COEFFICIENT_OF_RESTITUTION = Simulation.get(simID).COEFFICIENT_OF_RESTITUTION;
        COEFFICIENT_OF_FRICTION_DYNAMIC = Simulation.get(simID).COEFFICIENT_OF_FRICTION_DYNAMIC;
        COEFFICIENT_OF_FRICTION_STATIC = Simulation.get(simID).COEFFICIENT_OF_FRICTION_STATIC;
        this.radius = radius;
        repulseRadius = REPULSE_RADIUS_MULTIPLIER * radius;
        this.mass = mass;
        inertia = 0.5 * mass * radius * radius;
        this.color = color;
        this.isSolidBall = isSolidBall;
        area = Math.PI * getSolidRadius() * getSolidRadius();
        points.add(this);
    }

    //general motion copied from Rigidbody and adjusted to account for 0 angular movement and other factors specific to Point
    public static void step(double dt, int simID) {
        for (int i = 0; i < points.size(); i = i + 1) {
            if (points.get(i).simID == simID) {
                if (points.get(i).isMovable() && !points.get(i).isHitbox) {
                    points.get(i).calcMotion(dt);
                }
                else if (points.get(i).vX != 0.0 || points.get(i).vY != 0.0) {
                    points.get(i).calcMotionFreeMove(dt, true);
                }
                if (points.get(i).isHitbox) points.get(i).findCollisions();
            }
        }
    }
    public static void updateMotion(double dt, int simID) {
        for (int i = 0; i < points.size(); i = i + 1) {
            if (points.get(i).simID == simID) {
                if (points.get(i).isMovable()) points.get(i).updateMotion(dt);
                else if (points.get(i).vX != 0.0 || points.get(i).vY != 0.0) points.get(i).updateMotion(dt);
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
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (!(Rigidbody.get(i).simID == simID && (isHitbox || !Rigidbody.get(i).isHitbox))) continue;
            if (!intersecting) intersecting = checkForCollisionsBroad(Rigidbody.get(i));
            else checkForCollisionsBroad(Rigidbody.get(i));
        }
        if (!intersecting && bounds) intersecting = checkForCollisionsWall();
        else if (bounds) checkForCollisionsWall();
        for (int i = 0; i < Point.num; i = i + 1) {
            if (i != ID && Point.get(i).isSolidBall && Point.get(i).simID == simID && (isHitbox || !Point.get(i).isHitbox)) {
                if (!intersecting) intersecting = checkForCollisions(Point.get(i));
                else checkForCollisions(Point.get(i));
            }
        }
        for (int i = 0; i < Softbody.num; i = i + 1) {
            if (Softbody.get(i).simID != simID) continue;
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
        //corrects for repeatedly applied motion
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
            if ((vX - Rigidbody.getVX(collidingIDs.get(h))) * nX + (vY - Rigidbody.getVY(collidingIDs.get(h))) * nY > 0.0) {
                calcMotionFreeMove(dt, false);
                continue;
            }
            if (collidingIDs.get(h) == -1 || !Rigidbody.getIsMovable(collidingIDs.get(h))) {
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
                double pivotAngularA = (mass * (ndotAx * rY + ndotAy * -rX)) / (inertia + mass * r * r);
                double pivotAngularV = ((mass * (ndotVx * rY + ndotVy * -rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
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
                    rPerp2x = -(contactPoints.get(h)[1] - Rigidbody.getPosY(collidingIDs.get(h)));
                    rPerp2y = contactPoints.get(h)[0] - Rigidbody.getPosX(collidingIDs.get(h));
                }
                double jr = (newvX - Rigidbody.getVX(collidingIDs.get(h)) - rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h))) * nX + (newvY - Rigidbody.getVY(collidingIDs.get(h)) - rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h))) * nY;
                jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(h)));
                double temp1 = rPerp1x * nX + rPerp1y * nY;
                double temp2 = rPerp2x * nX + rPerp2y * nY;
                jr = jr / ((1.0 / mass) + (1.0 / Rigidbody.getMass(collidingIDs.get(h))) + (1.0 / inertia) * temp1 * temp1 + (1.0 / Rigidbody.getInertia(collidingIDs.get(h))) * temp2 * temp2);
                double vtrel = newvX * -nY + newvY * nX;
                vtrel = vtrel - ((Rigidbody.getVX(collidingIDs.get(h)) + rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h))) * -nY + (Rigidbody.getVY(collidingIDs.get(h)) + rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h))) * nX);
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

                if (collidingIDs.get(h) <= -2 && mod(collidingIDs.get(h), 2) == 1) {
                    int tempIndex = ((-collidingIDs.get(h) - 1) / 2) - 1;
                    int softbodyIndex = Point.get(tempIndex).parentSoftbody;
                    int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(tempIndex);
                    Point point1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex));
                    Point point2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % Softbody.get(softbodyIndex).boundarySize()));
                    double magnitude2 = Rigidbody.getAX(collidingIDs.get(h)) * nX + Rigidbody.getAY(collidingIDs.get(h)) * nY;
                    double ndotAx2 = magnitude2 * nX;
                    double ndotAy2 = magnitude2 * nY;
                    if (magnitude < 0.0) {
                        ndotAx2 = 0.0;
                        ndotAy2 = 0.0;
                    }
                    double tdotAx2 = Rigidbody.getAX(collidingIDs.get(h)) - ndotAx2;
                    double tdotAy2 = Rigidbody.getAY(collidingIDs.get(h)) - ndotAy2;
                    double update = (-(jr / Rigidbody.getMass(collidingIDs.get(h))) * nX - (fx / Rigidbody.getMass(collidingIDs.get(h))) + tdotAx2 * dt);
                    point1.changeVX(update);
                    point2.changeVX(update);
                    update = (-(jr / Rigidbody.getMass(collidingIDs.get(h))) * nY - (fy / Rigidbody.getMass(collidingIDs.get(h))) + tdotAy2 * dt);
                    point1.changeVY(update);
                    point2.changeVY(update);
                }
            }
        }
        if (intersecting) dt = dt * contactPoints.size();

        if (!intersecting && !mouseHold)  {
            calcMotionFreeMove(dt, true);
        }
        newaX = initialExternalForces[0];
        newaY = initialExternalForces[1];
        if (universalGravity) {
            double[] results = calculateGravity();
            newaX += results[0];
            newaY += results[1];
        }
        calculateRepulsion();
        if (attached) {
            calculateSpringForce();
        }
        if (isSolidBall && (airResistance || buoyancy)) {
            calculateAirResistance();
        }
        if (mouseHold) {
            double vmX = (currentMouseX - lastMouseX) / dt;
            double vmY = (currentMouseY - lastMouseY) / dt;
            double rX = newposX - currentMouseX;
            double rY = newposY - currentMouseY;
            double r = Math.sqrt(rX * rX + rY * rY);
            double pivotAngularA = (mass * (aX * -rY + aY * rX)) / (inertia + mass * r * r);
            double pivotAngularV = (mass * ((newvX - vmX) * -rY + (newvY - vmY) * rX)) / (inertia + mass * r * r) + 0.5 * pivotAngularA * dt;
            newposX = newposX + currentMouseX - lastMouseX;
            newposY = newposY + currentMouseY - lastMouseY;
            pivotAboutPoint(pivotAngularV * dt, currentMouseX, currentMouseY);

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
        double pivotAngularA = (mass * (ndotAx * -rY + ndotAy * rX)) / (inertia + mass * r * r);
        double pivotAngularV = ((mass * (ndotVx * -rY + ndotVy * rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
        pivotAboutPoint(pivotAngularV * dt, contactPoints.get(index)[0], contactPoints.get(index)[1]);
        newposX = newposX + tdotVx * dt;
        newposY = newposY + tdotVy * dt;

        double rPerp2x = -(contactPoints.get(index)[1] - Rigidbody.getPosY(collidingIDs.get(index)));
        double rPerp2y = contactPoints.get(index)[0] - Rigidbody.getPosX(collidingIDs.get(index));
        if (Double.isNaN(rPerp2x)) {
            rPerp2x = 0.0;
            rPerp2y = 0.0;
        }
        double jr = (newvX - Rigidbody.getVX(collidingIDs.get(index)) - rPerp2x * Rigidbody.getAngularV(collidingIDs.get(index))) * nX + (newvY - Rigidbody.getVY(collidingIDs.get(index)) - rPerp2y * Rigidbody.getAngularV(collidingIDs.get(index))) * nY;
        jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(index)));
        double temp1 = rPerp1x * nX + rPerp1y * nY;
        jr = jr / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);
        double vtrel = newvX * -nY + newvY * nX;
        vtrel = vtrel - ((Rigidbody.getVX(collidingIDs.get(index)) + rPerp2x * Rigidbody.getAngularV(collidingIDs.get(index))) * -nY + (Rigidbody.getVY(collidingIDs.get(index)) + rPerp2y * Rigidbody.getAngularV(collidingIDs.get(index))) * nX);
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

    }
    private void calcMotionFreeMove(double dt, boolean canUpdatePosition) {
        //first update linear position and rotational position
        if (canUpdatePosition) {
            newposX = posX + vX * dt + 0.5 * aX * dt * dt;
            newposY = posY + vY * dt + 0.5 * aY * dt * dt;
            newvX = vX + aX * dt;
            newvY = vY + aY * dt;
            //then update linear
            //finally, update linear acceleration
            newaX = aX;
            newaY = aY;
        }
        else {
            newposX = newposX + newvX * dt + 0.5 * aX * dt * dt;
            newposY = newposY + newvY * dt + 0.5 * aY * dt * dt;
            newvX = newvX + aX * dt;
            newvY = newvY + aY * dt;
        }
    }
    private boolean checkForCollisionsBroad(Rigidbody otherObject) {
        boolean results = false;
        double radius = getSolidRadius();
        double actualDistance = (otherObject.getPosX() - newposX) * (otherObject.getPosX() - newposX) + (otherObject.getPosY() - newposY) * (otherObject.getPosY() - newposY);
        actualDistance = Math.sqrt(actualDistance);
        if (Double.isNaN(actualDistance)) actualDistance = 0.0;
        if (actualDistance <= otherObject.getLargestDistance() + getSolidRadius()) {
            if (otherObject.isAABB(newposX - radius, newposX + radius, newposY - radius, newposY + radius)) {
                results = checkForCollisionsNarrow(otherObject);
            }
        }
        return(results);
    }
    private boolean checkForCollisionsNarrow(Rigidbody otherObject) {
        boolean intersecting = false;
        for (int i = 0; i < otherObject.getNumPoints(); i = i + 1) {
            if (otherObject.getTriangles().get(i).doesExist()) {
                Triplet results = otherObject.getTriangles().get(i).checkCollisions(this);
                if (results.getFirstBoolean()) {
                    intersecting = true;
                    contactPoints.add(results.getThirdDoubleArrayReference());
                    Double[] MTV = results.getSecondDoubleArrayReference();
                    if (otherObject.isMovable()){
                        double magnitude1 = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
                        double magnitude2 = -(magnitude1 - MTV_EPSILON) * (otherObject.getMass() / mass) - MTV_EPSILON;
                        MTV[0] = MTV[0] * (magnitude2 / magnitude1);
                        MTV[1] = MTV[1] * (magnitude2 / magnitude1);
                        MTVs.add(MTV);
                        collidingIDs.add(otherObject.getID());
                    }
                    else {
                        MTVs.add(MTV);
                        collidingIDs.add(otherObject.getID());
                    }

                }
            }
        }
        return(intersecting);

    }
    private boolean checkForCollisions(Point otherPoint) {
        boolean intersecting = false;
        double distance = (posX - otherPoint.getX()) * (posX - otherPoint.getX()) + (posY - otherPoint.getY()) * (posY - otherPoint.getY());
        distance = Math.sqrt(distance);
        double overlap = distance - (otherPoint.getSolidRadius() + getSolidRadius());
        if (overlap <= 0.0) {
            intersecting = true;
            Double[] MTV = new Double[2];
            Double[] pointOfContact = new Double[2];
            double temp = overlap;
            if (otherPoint.isMovable()){
                temp = (overlap / distance) * (otherPoint.getMass() / (otherPoint.getMass() + mass));
            }
            else {
                temp = (overlap / distance);
            }
            MTV[0] = (otherPoint.getX() - posX) * temp;
            MTV[0] = MTV[0] + MTV_EPSILON * Math.signum(MTV[0]);
            MTV[1] = (otherPoint.getY() - posY) * temp;
            MTV[1] = MTV[1] + MTV_EPSILON * Math.signum(MTV[1]);
            pointOfContact[0] = (otherPoint.getX() - posX) * (getSolidRadius() / distance) + MTV[0] + posX;
            pointOfContact[1] = (otherPoint.getY() - posY) * (getSolidRadius() / distance) + MTV[1] + posY;
            MTVs.add(MTV);
            contactPoints.add(pointOfContact);
            collidingIDs.add(otherPoint.ID * -2 - 2);
        }
        return(intersecting);
    }
    private boolean checkForCollisionsBroad(Softbody softbody) {
        if (!softbody.boundaryCollision) return(false);
        if (parentSoftbody != -1 && softbody.ID == parentSoftbody) return(false);
        boolean intersecting = false;
        double dx = softbody.cM[0] - posX;
        double dy = softbody.cM[1] - posY;
        if (dx * dx + dy * dy <= softbody.largestSquaredDistance + getSolidRadius()) {
            if (posX - getSolidRadius() < softbody.maxX && posX + getSolidRadius() > softbody.minX) {
                if (posY - getSolidRadius() < softbody.maxY && posY + getSolidRadius() > softbody.minY) {
                    intersecting = checkForCollisionsNarrow(softbody);
                }
            }
        }
        return(intersecting);
    }
    private boolean checkForCollisionsNarrow(Softbody softbody) {
        boolean intersecting = false;
        Triplet results = softbody.resolvePointInside(new double[]{posX, posY}, getSolidRadius());
        if (results.getFirstBoolean()) {
            Double[] MTV = new Double[2];
            double multiplier = (Rigidbody.getMass(results.getThirdInt()) / (Rigidbody.getMass(results.getThirdInt()) + mass));
            MTV[0] = results.getSecondDoubleArray()[0] * multiplier;
            MTV[1] = results.getSecondDoubleArray()[1] * multiplier;
            double magnitude = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
            double nX = MTV[0] / magnitude;
            double nY = MTV[1] / magnitude;
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            Double[] contactPoint = new Double[]{posX + MTV[0] - getSolidRadius() * nX, posY + MTV[1] - getSolidRadius() * nY};
            contactPoints.add(contactPoint);
            MTVs.add(MTV);
            collidingIDs.add(results.getThirdInt());
            intersecting = true;
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
        Double[] pointOfContact = new Double[]{Double.NaN, Double.NaN};
        if (posY + getSolidRadius() >= worldBottomBound) {
            if (Math.abs(worldBottomBound - (posY + getSolidRadius())) > Math.abs(bottom)){
                bottom = worldBottomBound - (posY + getSolidRadius());
                MTVs.add(new Double[]{0.0, bottom - MTV_EPSILON});
                contactPoints.add(new Double[]{posX, worldBottomBound});
                collidingIDs.add(-1);
                intersecting = true;
            }
        }
        if (posY - getSolidRadius() <= worldTopBound) {
            if (Math.abs(worldTopBound - (posY - getSolidRadius())) > Math.abs(top)){
                top = worldTopBound - (posY - getSolidRadius());
                MTVs.add(new Double[]{0.0, top + MTV_EPSILON});
                contactPoints.add(new Double[]{posX, worldTopBound});
                collidingIDs.add(-1);
                intersecting = true;
            }
        }
        if (posX - getSolidRadius() <= worldLeftBound) {
            if (Math.abs(worldLeftBound - (posX - getSolidRadius())) > Math.abs(left)){
                left = worldLeftBound - (posX - getSolidRadius());
                MTVs.add(new Double[]{left + MTV_EPSILON, 0.0});
                contactPoints.add(new Double[]{worldLeftBound, posY});
                collidingIDs.add(-1);
                intersecting = true;
            }
        }
        if (posX + getSolidRadius() >= worldRightBound) {
            if (Math.abs(worldRightBound - (posX + getSolidRadius())) > Math.abs(right)) {
                right = worldRightBound - (posX + getSolidRadius());
                MTVs.add(new Double[]{right - MTV_EPSILON, 0.0});
                contactPoints.add(new Double[]{worldRightBound, posY});
                collidingIDs.add(-1);
                intersecting = true;
            }
        }
        return(intersecting);
    }
    private double[] calculateGravity() {
        double sumaX = 0.0;
        double sumaY = 0.0;
        for (int i = 0; i < num; i = i + 1) {
            if (i != ID && Point.get(i).simID == simID) {
                double rSquared = (posX - Point.get(i).getX()) * (posX - Point.get(i).getX()) + (posY - Point.get(i).getY()) * (posY - Point.get(i).getY());
                if (rSquared > 0.0) {
                    double r = Math.sqrt(rSquared);
                    double magnitude = (GRAVITATIONAL_CONSTANT * Point.get(i).getMass()) / (rSquared);
                    sumaX = sumaX + (magnitude / r) * (Point.get(i).getX() - posX);
                    sumaY = sumaY + (magnitude / r) * (Point.get(i).getY() - posY);
                }
            }
        }
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (Rigidbody.get(i).simID != simID) continue;
            double rSquared = (posX - Rigidbody.get(i).getPosX()) * (posX - Rigidbody.get(i).getPosX()) + (posY - Rigidbody.get(i).getPosY()) * (posY - Rigidbody.get(i).getPosY());
            if (rSquared > 0.0) {
                double r = Math.sqrt(rSquared);
                double magnitude = (GRAVITATIONAL_CONSTANT * Rigidbody.get(i).getMass()) / (rSquared);
                sumaX = sumaX + (magnitude / r) * (Rigidbody.get(i).getPosX() - posX);
                sumaY = sumaY + (magnitude / r) * (Rigidbody.get(i).getPosY() - posY);
            }
        }
        return(new double[]{sumaX, sumaY});
    }
    private void calculateAirResistance() {
        double[] fD = new double[]{0.0, 0.0};
        if (airResistance) {
            double crossArea = 2.0 * getSolidRadius();
            double magnitude = Math.sqrt(vX * vX + vY * vY);
            double forceMagnitude = 0.5 * AIR_DENSITY * crossArea * DRAG_COEFFICIENT * magnitude * magnitude;
            double uX = vX / magnitude;
            double uY = vY / magnitude;
            fD = new double[]{-uX * forceMagnitude, -uY * forceMagnitude};
            if (Double.isNaN(fD[0])) fD[0] = 0.0;
            if (Double.isNaN(fD[1])) fD[1] = 0.0;
        }

        double[] fB = new double[]{0.0, 0.0};
        if (buoyancy) {
            double magnitude = AIR_DENSITY * area;
            fB = new double[]{-newaX * magnitude, -newaY * magnitude};
        }

        newaX += (fD[0] + fB[0]) / mass;
        newaY += (fD[1] + fB[1]) / mass;

    }
    private void calculateRepulsion() {
        if (parentSoftbody != -1) for (int i = 0; i < num; i = i + 1) {
            if (i != ID && Point.get(i).parentSoftbody != parentSoftbody && Point.get(i).parentSoftbody != -1 && Point.get(i).simID == simID) {
                double dx = posX - Point.get(i).getX();
                double dy = posY - Point.get(i).getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance > 0.0 && distance < repulseRadius + Point.get(i).getRepulseRadius()) {
                    double aMagnitude = Math.max(0.0, REPULSION_STRENGTH * (((Point.get(i).getRepulseRadius() + repulseRadius) / distance) - 1.0));
                    newaX += aMagnitude * (dx / distance);
                    newaY += aMagnitude * (dy / distance);
                }
            }
        }
    }
    private void calculateSpringForce() {
        //double local_SPRING_DAMPING_COEFFICIENT = Math.min(Math.sqrt(mass * HOOKE_SPRING_CONSTANT), SPRING_DAMPING_COEFFICIENT);
        double local_SPRING_DAMPING_COEFFICIENT = SPRING_DAMPING_COEFFICIENT;
        for (int i = 0; i < attachedIndices.size(); i = i + 1) {
            double dx = points.get(attachedIndices.get(i)).posX - posX;
            double dy = points.get(attachedIndices.get(i)).posY - posY;
            double distance = Math.sqrt(dx * dx + dy * dy);
            double nX = dx / distance;
            double nY = dy / distance;
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            double[] fS = new double[2];
            fS[0] = HOOKE_SPRING_CONSTANT * (dx - idealSpringLengths.get(i) * nX);
            fS[1] = HOOKE_SPRING_CONSTANT * (dy - idealSpringLengths.get(i) * nY);
            double multiplier = HOOKE_SPRING_CONSTANT * SPRING_MAX_DIST_MULTIPLIER * idealSpringLengths.get(i);
            double[] maxF = new double[]{multiplier * nX, multiplier * nY};
            fS[0] = Math.min(Math.abs(fS[0]), Math.abs(maxF[0])) * Math.signum(fS[0]);
            fS[1] = Math.min(Math.abs(fS[1]), Math.abs(maxF[1])) * Math.signum(fS[1]);
            multiplier = ((vX - points.get(attachedIndices.get(i)).vX) * nX + (vY - points.get(attachedIndices.get(i)).vY) * nY) * local_SPRING_DAMPING_COEFFICIENT;
            double[] fD = new double[]{-nX * multiplier, -nY * multiplier};
            if (fD[0] * fD[0] + fD[1] * fD[1] < fS[0] * fS[0] + fS[1] * fS[1]) {
                newaX += (fS[0] + fD[0]) / mass;
                newaY += (fS[1] + fD[1]) / mass;
            }

            double minDist = SPRING_MIN_DIST_MULTIPLIER * idealSpringLengths.get(i);
            double maxDist = SPRING_MAX_DIST_MULTIPLIER * idealSpringLengths.get(i);
            if (distance < minDist) {
                newposX = newposX + (distance - minDist) * nX * 0.5;
                newposY = newposY + (distance - minDist) * nY * 0.5;
            }
            if (distance > maxDist) {
                newposX = newposX + (distance - maxDist) * nX * 0.5;
                newposY = newposY + (distance - maxDist) * nY * 0.5;
            }
        }
    }
    private void pivotAboutPoint(double theta, double pointX, double pointY) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double x = newposX;
        double y = newposY;
        newposX = (x - pointX) * cos - (y - pointY) * sin + pointX;
        newposY = (y - pointY) * cos + (x - pointX) * sin + pointY;
    }
    private boolean pointInside(double[] inputPoint) {
        double distance = ((inputPoint[0] - posX) * (inputPoint[0] - posX) + (inputPoint[1] - posY) * (inputPoint[1] - posY));
        if (distance < 0.0) distance = 0.0;
        else distance = Math.sqrt(distance);
        return(distance <= radius);
    }
    public static boolean moveByMouse(double x, double y, boolean reset, boolean mousePressed, double dt, int simID) {
        boolean output = false;
        for (int i = 0; i < Point.num; i = i + 1) {
            if (Point.get(i).simID == simID) {
                output = Point.get(i).localMoveByMouse(x, y, reset, mousePressed, dt);
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
        return(isInside);
    }
    private void updateMotion(double dt) {
        newposX += otherUpdatePosX;
        newposY += otherUpdatePosY;
        if (vUpdateCount == 0) vUpdateCount = 1;
        newvX += otherUpdateVX / vUpdateCount;
        newvY += otherUpdateVY / vUpdateCount;
        vUpdateCount = 0;
        newaX += otherUpdateAX;
        newaY += otherUpdateAY;
        if (newvX * newvX + newvY * newvY < CLAMP_LIMIT * CLAMP_LIMIT) {
            newvX = 0.0;
            newvY = 0.0;
        }
        if (!Double.isNaN(newposX)) posX = newposX;
        if (!Double.isNaN(newposY)) posY = newposY;
        if (!Double.isNaN(newvX)) vX = newvX;
        if (!Double.isNaN(newvY)) vY = newvY;
        if (!Double.isNaN(newaX)) aX = newaX;
        if (!Double.isNaN(newaY)) aY = newaY;

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

    public void attach(Point other) {
        if (!attachedIndices.contains(other.ID) && !other.attachedIndices.contains(ID) && other.simID == simID) {
            attached = true;
            attachedIndices.add(other.ID);
            other.attached = true;
            other.attachedIndices.add(ID);
            double distance = (other.posX - posX) * (other.posX - posX) + (other.posY - posY) * (other.posY - posY);
            distance = Math.sqrt(distance);
            idealSpringLengths.add(distance);
            other.idealSpringLengths.add(distance);
        }
    }
    public void generatePoints(double theta, double r, double[] borderX, double[] borderY, boolean invertNormals) {
        int angleDivisions = 4;
        boolean[] onBoundaryPoints = new boolean[angleDivisions];
        boolean[] inShapePoints = new boolean[angleDivisions];
        double[] x = new double[angleDivisions];
        double[] y = new double[angleDivisions];
        for (int i = 0; i < angleDivisions; i = i + 1) {
            double angle = theta + ((2.0 * Math.PI) / angleDivisions) * i;
            Triplet results = pointInsideBorderConstruct(posX + r * Math.cos(angle), posY + r * Math.sin(angle), borderX, borderY, invertNormals);
            //first boolean is if the proposed corrected point is on the boundary, second coords is where the corrected point is
            //third boolean is if the proposed point (before correction) is within the shape
            onBoundaryPoints[i] = results.getFirstBoolean();
            inShapePoints[i] = results.getThirdBoolean();
            x[i] = results.getSecondDoubleArray()[0];
            y[i] = results.getSecondDoubleArray()[1];
            if (results.getFirstBoolean()) {
                futureBoundary = true;
                break;
            }
        }
        ArrayList<Point> generatedPoints = new ArrayList<>();
        for (int i = 0; i < angleDivisions; i = i + 1) {
            if (inShapePoints[i] && !futureBoundary) {
                Point generatedPoint = new Point(new double[]{x[i], y[i], 0.0, 0.0, 0.0, 0.0}, Softbody.get(parentSoftbody).getPointRadius(), 1.0, Softbody.get(parentSoftbody).getColor(), true, simID);
                Simulation.get(simID).physicsObjects.add(new PhysicsObject(generatedPoint));
                attach(generatedPoint);
                Softbody.get(parentSoftbody).addMember(generatedPoint, futureBoundary);
                generatedPoints.add(generatedPoint);
            }
        }
        for (Point generatedPoint : generatedPoints) {
            generatedPoint.generatePoints(mod(theta - Math.PI,2.0 * Math.PI), r, borderX, borderY, invertNormals);
        }
    }
    private Triplet pointInsideBorderConstruct(double x, double y, double[] borderX, double[] borderY, boolean invertNormals) {
        int count = pointInsideConstruct(x, y, borderX, borderY);
        if (count % 2 == 0) {
            double distance = (x - 50.0) * (x - 50.0) + (y - 20.0) * (y - 20.0);
            distance = Math.sqrt(distance);
            if (distance < 20.0) {
                double a = 1;
            }
        }
        //if the point is not inside, then put it on the closest edge point
        boolean onBoundary = count % 2 == 0;
        boolean inShape = true;
        double newx = x;
        double newy = y;
        double maxDot = Double.NaN;
        if (onBoundary) {
            inShape = false;
            for (int i = 0; i < borderX.length; i = i + 1) {
                double nX = -(borderY[(i + 1) % borderX.length] - borderY[i]);
                double nY = borderX[(i + 1) % borderX.length] - borderX[i];
                double magnitude = Math.sqrt(nX * nX + nY * nY);
                nX = nX / magnitude;
                nY = nY / magnitude;
                if (Double.isNaN(nX) || Double.isNaN(nY)) {
                    nX = 0.0;
                    nY = 0.0;
                }
                if (invertNormals) {
                    nX = -nX;
                    nY = -nY;
                }
                double orthoDot = x * -nY + y * nX;
                double orthoDotMin = borderX[i] * -nY + borderY[i] * nX;
                double orthoDotMax = borderX[(i + 1) % borderX.length] * -nY + borderY[(i + 1) % borderX.length] * nX;
                if (orthoDotMax < orthoDotMin) {
                    double temp = orthoDotMin;
                    orthoDotMin = orthoDotMax;
                    orthoDotMax = temp;
                }
                if (orthoDot < orthoDotMax && orthoDot > orthoDotMin) {
                    double dot = (x - borderX[i]) * nX + (y - borderY[i]) * nY;
                    if (dot > 0.0 && (Double.isNaN(maxDot) || dot < maxDot)) {
                        newx = newx - dot * nX;
                        newy = newy - dot * nY;
                        maxDot = dot;
                        onBoundary = true;
                    }
                }
            }
        }
        x = newx;
        y = newy;

        //check if the point has already been created (temporary, later going to use local geometry to determine rather than distance check)
        boolean check = false;
        if (inShape || onBoundary) for (int i = 0; i < Point.num; i = i + 1) {
            if (i != ID && Point.get(i).parentSoftbody == parentSoftbody && Point.get(i).simID == simID) {
                double distance = (x - Point.get(i).posX) * (x - Point.get(i).posX) + (y - Point.get(i).posY) * (y - Point.get(i).posY);
                distance = Math.sqrt(distance);
                if (distance < Softbody.get(parentSoftbody).getPointRadius() || (onBoundary && distance < 2.0 * Softbody.get(parentSoftbody).getPointRadius())) {
                    inShape = false;
                    check = true;
                    break;
                }
            }
        }
        if (onBoundary) {
            double a = 1;
        }
        //if (onBoundary && !check) inShape = true;

        return(new Triplet(onBoundary, new double[]{x, y}, inShape));
    }
    private int pointInsideConstruct(double x, double y, double[] borderX, double[] borderY) {
        int count = 0;
        for (int i = 0; i < borderX.length; i = i + 1) {
            double minX = borderX[i];
            double maxX = borderX[(i + 1) % borderX.length];
            if (maxX < minX) {
                minX = maxX;
                maxX = borderX[i];
            }
            double minY = borderY[i];
            double maxY = borderY[(i + 1) % borderX.length];
            if (maxY < minY) {
                minY = maxY;
                maxY = borderY[i];
            }
            if (x >= minX && x <= maxX) {
                if (y < minY) count = count + 1;
                else if (borderX[(i + 1) % borderX.length] - borderX[i] != 0.0) {
                    double m = (borderY[(i + 1) % borderX.length] - borderY[i]) / (borderX[(i + 1) % borderX.length] - borderX[i]);
                    double b = borderY[i] - m * (borderX[i]);
                    if (m * x + b > y) count = count + 1;
                }
            }
        }
        return(count);
    }
    private double mod(double a, double b) {
        if (a >= 0) return(a % b);
        else {
            double tempResult = b - (-a % b);
            if (tempResult == b) return(0);
            else return(tempResult);
        }
    }

    public static Point get(int index) {
        return(points.get(index));
    }
    public Color getColor() {
        return(color);
    }
    public double getMass() {
        return(mass);
    }
    public double getRadius() {
        return(radius);
    }
    public double getSolidRadius() {
        if (isSolidBall) {
            if (attached && parentSoftbody != -1) return(Softbody.get(parentSoftbody).getPointRadius());
            else return(radius);
        }
        else return(POINT_MIN_RADIUS);
    }
    public double getRepulseRadius() {
        return(repulseRadius);
    }
    public double getX() {
        return(posX);
    }
    public double getY() {
        return(posY);
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
    public double getInertia() {
        return(inertia);
    }
    public boolean isMovable() {
        return(isMovable);
    }
    public boolean isSolidBall() {
        return(isSolidBall);
    }
    public boolean isAttached() {
        return(attached);
    }
    public Point getAttachment(int index) {
        return(points.get(attachedIndices.get(index)));
    }
    public ArrayList<Integer> getAttachments() {
        return(attachedIndices);
    }
    public int getAttachmentNum() {
        return(attachedIndices.size());
    }
    public int getBoundaryAttachmentNum() {
        int count = 0;
        if (Softbody.get(parentSoftbody).boundaryMembers.contains(ID)) {
            for (int i = 0; i < attachedIndices.size(); i = i + 1) {
                if (Softbody.get(parentSoftbody).boundaryMembers.contains(attachedIndices.get(i))) count += 1;
            }
        }
        else count = -ID;
        return(count);
    }

    public void setMass(double mass) {
        this.mass = mass;
    }
    public void setMovingMotion(double[] motion) {
        vX = motion[0];
        vY = motion[1];
        aX = motion[2];
        initialExternalForces[0] = aX;
        aY = motion[3];
        initialExternalForces[1] = aY;
    }
    public void changeX(double update) {
        otherUpdatePosX += update;
    }
    public void changeY(double update) {
        otherUpdatePosY += update;
    }
    public void changeVX(double update) {
        otherUpdateVX += update;
        vUpdateCount += 1;
    }
    public void changeVY(double update) {
        otherUpdateVY += update;
    }
    public void changeAX(double update) {
        otherUpdateAX += update;
    }
    public void changeAY(double update) {
        otherUpdateAY += update;
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
    public void makeAdoptOtherSurfaceOnly(boolean a) {
        adoptOnlyOtherSurface = a;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public void setIsMovable(boolean a) {
        isMovable = a;
    }

    public double getArea() {
        return(area);
    }
}
