package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;

class Point {
    private double mass;
    private final double area;
    private final double inertia;
    private Color color;
    private final double radius;
    private final double repulseRadius;
    private static final ArrayList<Point> points = new ArrayList<>();
    private final ArrayList<Double[]> contactPoints = new ArrayList<>();
    private final ArrayList<Double[]> MTVs = new ArrayList<>();
    private final ArrayList<Integer> collidingIDs = new ArrayList<>();
    //same collidingIDs convention as Rigidbody
    private final ArrayList<Integer> attachedIndices = new ArrayList<>();
    private final ArrayList<Double> idealSpringLengths = new ArrayList<>();
    public int parentSoftbody = -1;
    public static int num = 0;
    public int ID = 0;

    public static double COEFFICIENT_OF_RESTITUTION = 0.75;
    public static double COEFFICIENT_OF_FRICTION_DYNAMIC = 0.35;
    public static double COEFFICIENT_OF_FRICTION_STATIC = 0.5;
    public static double GRAVITATIONAL_CONSTANT = 10.0;
    public static double AIR_DENSITY = 0.01204;
    public static double DRAG_COEFFICIENT = 0.95;
    public static double CLAMP_LIMIT = 0.5;
    public static double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public static double MTV_EPSILON = 0.00165;
    public static double POINT_MIN_RADIUS = 2.5;
    public double HOOKE_SPRING_CONSTANT = 1.0;
    public double SPRING_DAMPING_COEFFICIENT = 1.0;
    public double SPRING_MAX_DIST_MULTIPLIER = 2.0;
    public double SPRING_MIN_DIST_MULTIPLIER = 0.0;
    public static double REPULSION_STRENGTH = 5.0;

    public static double worldTopBound = 0.0;
    public static double worldBottomBound = 460.0;
    public static double worldLeftBound = 0.0;
    public static double worldRightBound = 485.0;
    private static double lastMouseX = 250.0;
    private static double lastMouseY = 250.0;
    private static double currentMouseX = 250.0;
    private static double currentMouseY = 250.0;
    private static double flingX = 0.0;
    private static double flingY = 0.0;
    private boolean mouseHold = false;
    private boolean mouseRelease = false;

    private boolean isMovable = true;
    private final boolean isSolidBall;
    private boolean attached = false;
    public boolean futureBoundary = false;
    public int minIndex1 = -1;
    public int minIndex2 = -1;
    public static boolean universalGravity = false;
    public static boolean airResistance = false;
    public static boolean buoyancy = false;
    public static boolean bounds = true;

    private double posX;
    private double posY;
    private double vX;
    private double vY;
    private double aX;
    private double aY;

    private double newposX;
    private double newposY;
    private double newvX;
    private double newvY;
    private double newaX;
    private double newaY;

    public Point(double[] motion, double radius, double mass, Color color, boolean isSolidBall) {
        ID = num;
        num = num + 1;
        posX = motion[0];
        posY = motion[1];
        vX = motion[2];
        vY = motion[3];
        aX = motion[4];
        aY = motion[5];
        this.radius = radius;
        repulseRadius = 5.0;
        this.mass = mass;
        inertia = 0.5 * mass * radius * radius;
        this.color = color;
        this.isSolidBall = isSolidBall;
        area = Math.PI * getSolidRadius() * getSolidRadius();
        points.add(this);
    }

    //general motion copied from Rigidbody and adjusted to account for 0 angular movement and other factors specific to Point
    public static void step(double dt) {
        for (int i = 0; i < points.size(); i = i + 1) {
            if(points.get(i).isMovable()) points.get(i).calcMotion(dt);
        }
        for (int i = 0; i < points.size(); i = i + 1) {
            if (points.get(i).isMovable()) points.get(i).updateMotion();
        }
    }
    private void calcMotion(double dt) {
        //check for collisions and do one of two options for updating motion based on whether the rigidbody is colliding with another
        //clear the list of point information
        for (int h = 0; !contactPoints.isEmpty(); h = h) {
            contactPoints.removeFirst();
        }
        for (int h = 0; !MTVs.isEmpty(); h = h) {
            MTVs.removeFirst();
        }
        for (int h = 0; !collidingIDs.isEmpty(); h = h) {
            collidingIDs.removeFirst();
        }
        //determine point information
        boolean intersecting = false;
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (!intersecting) intersecting = checkForCollisionsBroad(Rigidbody.get(i));
            else checkForCollisionsBroad(Rigidbody.get(i));
        }
        if (!intersecting && bounds) intersecting = checkForCollisionsWall();
        else if (bounds) checkForCollisionsWall();
        for (int i = 0; i < Point.num; i = i + 1) {
            if (i != ID && Point.get(i).isSolidBall) {
                if (!intersecting) intersecting = checkForCollisions(Point.get(i));
                else checkForCollisions(Point.get(i));
            }
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
        newposX = posX;
        newposY = posY;
        newvX = vX;
        newvY = vY;
        if (intersecting) for (int h = 0; h < contactPoints.size(); h = h + 1) {
            Display.pointOfContactDraw = contactPoints.get(h);
            if (collidingIDs.get(h) == -1 || !Rigidbody.getIsMovable(collidingIDs.get(h))) {
                calcMotionInfiniteMass(h, dt);
                continue;
            }
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
                double ndotAx = (aX * nX + aY * nY) * nX;
                double ndotAy = (aX * nX + aY * nY) * nY;
                double pivotAngularA = (mass * (ndotAx * rY + ndotAy * -rX)) / (inertia + mass * r * r);
                double pivotAngularV = ((mass * (ndotVx * rY + ndotVy * -rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
                pivotAboutPoint(pivotAngularV * dt, contactPoints.get(h)[0], contactPoints.get(h)[1]);
                newposX = newposX + tdotVx * dt;
                newposY = newposY + tdotVy * dt;

                //next, update the velocity in an impulse-based reaction model

                double rPerp2x = -(contactPoints.get(h)[1] - Rigidbody.getPosY(collidingIDs.get(h)));
                double rPerp2y = contactPoints.get(h)[0] - Rigidbody.getPosX(collidingIDs.get(h));
                double jr = (newvX - Rigidbody.getVX(collidingIDs.get(h)) - rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h))) * nX + (newvY - Rigidbody.getVY(collidingIDs.get(h)) - rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h))) * nY;
                jr = jr * -(1.0 + COEFFICIENT_OF_RESTITUTION);
                double temp1 = rPerp1x * nX + rPerp1y * nY;
                double temp2 = rPerp2x * nX + rPerp2y * nY;
                jr = jr / ((1.0 / mass) + (1.0 / Rigidbody.getMass(collidingIDs.get(h))) + (1.0 / inertia) * temp1 * temp1 + (1.0 / Rigidbody.getInertia(collidingIDs.get(h))) * temp2 * temp2);
                double vtrel = newvX * -nY + newvY * nX;
                vtrel = vtrel - ((Rigidbody.getVX(collidingIDs.get(h)) + rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h))) * -nY + (Rigidbody.getVY(collidingIDs.get(h)) + rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h))) * nX);
                temp1 = aX * -nX + aY * -nY;
                double friction = 0.0;
                if (temp1 > 0.0) {
                    if (vtrel > -CLAMP_LIMIT && vtrel < CLAMP_LIMIT && Math.abs(aX * -nY + aY * nX) <= COEFFICIENT_OF_FRICTION_STATIC * temp1) friction = mass * (aX * -nY + aY * nX) * -Math.signum(vtrel);
                    else friction = mass * COEFFICIENT_OF_FRICTION_DYNAMIC * temp1 * -Math.signum(vtrel);
                }
                double fx = friction * -nY * dt;
                double fy = friction * nX * dt;
                double adotTx = (aX * -nY + aY * nX) * -nY;
                double adotTy = (aX * -nY + aY * nX) * nX;

                newvX = newvX + (jr / mass) * nX + (fx / mass) + adotTx * dt;
                newvY = newvY + (jr / mass) * nY + (fy / mass) + adotTy * dt;

                newaX = aX;
                newaY = aY;
            }
        }

        if (!intersecting && !mouseHold)  {
            //first update linear position and rotational position
            newposX = posX + vX * dt + 0.5 * aX * dt * dt;
            newposY = posY + vY * dt + 0.5 * aY * dt * dt;
            newvX = vX + aX * dt;
            newvY = vY + aY * dt;
            //then update linear
            //finally, update linear acceleration
            newaX = aX;
            newaY = aY;
        }
        if (universalGravity) {
            double[] results = calculateGravity();
            newaX = results[0];
            newaY = results[1];
        }
        if (airResistance && isSolidBall && vX * vX + vY * vY > CLAMP_LIMIT * CLAMP_LIMIT) {
            calculateAirResistance(dt);
        }
        if (!isSolidBall) {
            calculateRepulsion(dt);
        }
        if (attached) {
            calculateSpringForce(dt);
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

            if (mouseRelease) {
                newvX = newvX + 10.0 * flingX;
                newvY = newvY + 10.0 * flingY;
                mouseRelease = false;
            }
        }

    }
    private void calcMotionInfiniteMass(int index, double dt) {
        newposX = newposX + MTVs.get(index)[0];
        newposY = newposY + MTVs.get(index)[1];
        double rX = newposX - contactPoints.get(index)[0];
        double rY = newposY - contactPoints.get(index)[1];
        double r = Math.sqrt(rX * rX + rY * rY);
        double magnitude = Math.sqrt(MTVs.get(index)[0] * MTVs.get(index)[0] + MTVs.get(index)[1] * MTVs.get(index)[1]);
        double nX = MTVs.get(index)[0] / magnitude;
        double nY = MTVs.get(index)[1] / magnitude;
        magnitude = newvX * nX + newvY * nY;
        double ndotVx = magnitude * nX;
        double ndotVy = magnitude * nY;
        magnitude = newvX * -nY + newvY * nX;
        double tdotVx = magnitude * -nY;
        double tdotVy = magnitude * nX;
        //calculate the amount to pivot by
        double ndotAx = (aX * nX + aY * nY) * nX;
        double ndotAy = (aX * nX + aY * nY) * nY;
        double pivotAngularA = (mass * (ndotAx * -rY + ndotAy * rX)) / (inertia + mass * r * r);
        double pivotAngularV = ((mass * (ndotVx * -rY + ndotVy * rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
        pivotAboutPoint(pivotAngularV * dt, contactPoints.get(index)[0], contactPoints.get(index)[1]);
        newposX = newposX + tdotVx * dt;
        newposY = newposY + tdotVy * dt;

        double jr = newvX * nX + newvY * nY;
        jr = jr * -(1.0 + COEFFICIENT_OF_RESTITUTION);
        double temp1 = rY * nX + -rX * nY;
        jr = jr / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);
        double vtrel = newvX * -nY + newvY * nX;
        double temp2 = aX * -nX + aY * -nY;
        double friction = 0.0;
        if (temp2 > 0.0) {
            if (vtrel > -CLAMP_LIMIT && vtrel < CLAMP_LIMIT && Math.abs(aX * -nY + aY * nX) <= COEFFICIENT_OF_FRICTION_STATIC * temp2) friction = mass * (aX * -nY + aY * nX) * -Math.signum(vtrel);
            else friction = mass * COEFFICIENT_OF_FRICTION_DYNAMIC * temp2 * -Math.signum(vtrel);
        }
        double fx = friction * -nY * dt;
        double fy = friction * nX * dt;
        double adotTx = (aX * -nY + aY * nX) * -nY;
        double adotTy = (aX * -nY + aY * nX) * nX;

        newvX = newvX + (jr / mass) * nX + (fx / mass) + adotTx * dt;
        newvY = newvY + (jr / mass) * nY + (fy / mass) + adotTy * dt;

        newaX = aX;
        newaY = aY;
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
            collidingIDs.add(-otherPoint.ID - 2);
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
            if (i != ID) {
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
    private void calculateAirResistance(double dt) {
        double crossArea = 2.0 * getSolidRadius();
        double magnitude = Math.sqrt(newvX * newvX + newvY * newvY);
        double forceMagnitude = 0.5 * AIR_DENSITY * crossArea * DRAG_COEFFICIENT * magnitude * magnitude;
        double update = (forceMagnitude / mass) * -(newvX / magnitude) * dt;
        if (Math.abs(update) > Math.abs(newvX)) update = 0.0;
        newvX = newvX + update;
        update = (forceMagnitude / mass) * -(newvY / magnitude) * dt;
        if (Math.abs(update) > Math.abs(newvY)) update = 0.0;
        newvY = newvY + update;
        if (buoyancy) {
            magnitude = AIR_DENSITY * area;
            double[] fB = new double[]{-aX * magnitude, -aY * magnitude};
            newvX = newvX + fB[0] * (dt / mass);
            newvY = newvY + fB[1] * (dt / mass);
        }

    }
    private void calculateRepulsion(double dt) {
        for (int i = 0; i < num; i = i + 1) {
            if (i != ID && !Point.get(i).isSolidBall) {
                double dx = posX - Point.get(i).getX();
                double dy = posY - Point.get(i).getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance > 0.0) {
                    double aMagnitude = Math.max(0.0, REPULSION_STRENGTH * (((Point.get(i).getRepulseRadius() + repulseRadius) / distance) - 1.0));
                    newvX = newvX + aMagnitude * (dx / distance);
                    newvY = newvY + aMagnitude * (dy / distance);
                }
            }
        }
    }
    private void calculateSpringForce(double dt) {
        //double local_SPRING_DAMPING_COEFFICIENT = Math.min(Math.sqrt(mass * HOOKE_SPRING_CONSTANT), SPRING_DAMPING_COEFFICIENT);
        double local_SPRING_DAMPING_COEFFICIENT = SPRING_DAMPING_COEFFICIENT;
        for (int i = 0; i < attachedIndices.size(); i = i + 1) {
            double distance = (posX - points.get(attachedIndices.get(i)).posX) * (posX - points.get(attachedIndices.get(i)).posX);
            distance = distance + (posY - points.get(attachedIndices.get(i)).posY) * (posY - points.get(attachedIndices.get(i)).posY);
            distance = Math.sqrt(distance);
            double[] fS = new double[]{posX - points.get(attachedIndices.get(i)).posX, posY - points.get(attachedIndices.get(i)).posY};
            double multiplier = (HOOKE_SPRING_CONSTANT) * (1.0 - (idealSpringLengths.get(i) / distance));
            double nX = fS[0] / distance;
            double nY = fS[1] / distance;
            fS[0] = multiplier * fS[0];
            fS[1] = multiplier * fS[1];
            multiplier = (HOOKE_SPRING_CONSTANT * (SPRING_MAX_DIST_MULTIPLIER - 1.0) * idealSpringLengths.get(i)) / distance;
            double[] maxF = new double[]{(points.get(attachedIndices.get(i)).posX - posX) * multiplier, (points.get(attachedIndices.get(i)).posY - posY) * multiplier};
            fS[0] = Math.min(Math.abs(fS[0]), Math.abs(maxF[0])) * Math.signum(fS[0]);
            fS[1] = Math.min(Math.abs(fS[1]), Math.abs(maxF[1])) * Math.signum(fS[1]);
            multiplier = ((vX - points.get(attachedIndices.get(i)).vX) * nX + (vY - points.get(attachedIndices.get(i)).vY) * nY) * local_SPRING_DAMPING_COEFFICIENT;
            double[] fD = new double[]{-nX * multiplier, -nY * multiplier};
            if (fD[0] * fD[0] + fD[1] * fD[1] < fS[0] * fS[0] + fS[1] * fS[1]) {
                newvX = newvX + (fD[0] - fS[0]) * (dt / mass);
                newvY = newvY + (fD[1] - fS[1]) * (dt / mass);
            }

            double minDist = SPRING_MIN_DIST_MULTIPLIER * idealSpringLengths.get(i);
            double maxDist = SPRING_MAX_DIST_MULTIPLIER * idealSpringLengths.get(i);
            if (distance < minDist) {
                newposX = newposX + (minDist - distance) * nX * 0.5;
                newposY = newposY + (minDist - distance) * nY * 0.5;
            }
            if (distance > maxDist) {
                newposX = newposX + (maxDist - distance) * nX * 0.5;
                newposY = newposY + (maxDist - distance) * nY * 0.5;
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
    public static void moveByMouse(double x, double y, boolean reset, boolean mousePressed, double dt) {
        flingX = (x - lastMouseX) / dt;
        flingY = (y - lastMouseY) / dt;
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
        for (int i = 0; i < num; i = i + 1) {
            if (points.get(i).pointInside(new double[]{currentMouseX, currentMouseY})) {
                if (mousePressed) {
                    points.get(i).mouseHold = true;
                    break;
                }
                else {
                    if (points.get(i).mouseHold) points.get(i).mouseRelease = true;
                    points.get(i).mouseHold = false;
                }
            }
            else {
                points.get(i).mouseHold = false;
            }
        }
    }
    private void updateMotion() {
        if (newvX * newvX + newvY * newvY < CLAMP_LIMIT * CLAMP_LIMIT) {
            newvX = 0.0;
            newvY = 0.0;
        }
        posX = newposX;
        posY = newposY;
        vX = newvX;
        vY = newvY;
        aX = newaX;
        aY = newaY;
    }

    public void attach(Point other) {
        if (!attachedIndices.contains(other.ID) && !other.attachedIndices.contains(ID)) {
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
        ArrayList<Point> generatedPoints = new ArrayList<>();
        boolean[] onBoundaryPoints = new boolean[angleDivisions];
        boolean[] inShapePoints = new boolean[angleDivisions];
        for (int i = 0; i < angleDivisions; i = i + 1) {
            double angle = theta + ((2.0 * Math.PI) / angleDivisions) * i;
            Triplet results = pointInsideBorderConstruct(posX + r * Math.cos(angle), posY + r * Math.sin(angle), borderX, borderY, invertNormals);
            if (results.getFirstBoolean() && results.getThirdBoolean()) {
                futureBoundary = true;
            }
            if (results.getThirdBoolean() && !results.getFirstBoolean()) {
                Point generatedPoint = new Point(new double[]{results.getSecondDoubleArray()[0], results.getSecondDoubleArray()[1], 0.0, 0.0, 0.0, 0.0}, Softbody.get(parentSoftbody).getPointRadius(), 1.0, Softbody.get(parentSoftbody).getColor(), true);
                Softbody.get(parentSoftbody).addMember(generatedPoint, results.getFirstBoolean());
                generatedPoints.add(generatedPoint);
                onBoundaryPoints[i] = results.getFirstBoolean();
                inShapePoints[i] = results.getThirdBoolean();
                attach(generatedPoint);
            }
            else {
                generatedPoints.add(null);
            }
        }
        for (int i = 0; i < angleDivisions; i = i + 1) {
            if (!onBoundaryPoints[i] && inShapePoints[i] && generatedPoints.get(i) != null) generatedPoints.get(i).generatePoints(mod(theta - Math.PI,2.0 * Math.PI), r, borderX, borderY, invertNormals);
        }
    }
    private Triplet pointInsideBorderConstruct(double x, double y, double[] borderX, double[] borderY, boolean invertNormals) {
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
            if (x > minX && x < maxX) {
                if (y < minY) count = count + 1;
                else if (borderX[(i + 1) % borderX.length] - borderX[i] != 0.0) {
                    double m = (borderY[(i + 1) % borderX.length] - borderY[i]) / (borderX[(i + 1) % borderX.length] - borderX[i]);
                    double b = borderY[i] - m * (borderX[i]);
                    if (m * x + b > y) count = count + 1;
                }
            }
        }
        //if the point is not inside, then put it on the closest edge point
        boolean onBoundary = false;
        boolean inShape = true;
        if (count % 2 == 0) {
            inShape = false;
            for (int i = 0; i < borderX.length; i = i + 1) {
                double nX = -(borderY[(i + 1) % borderX.length] - borderY[i]);
                double nY = borderX[(i + 1) % borderX.length] - borderX[i];
                double magnitude = Math.sqrt(nX * nX + nY * nY);
                nX = nX / magnitude;
                nY = nY / magnitude;
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
                    if (dot > 0.0) {
                        x = x - dot * nX * 0.1;
                        y = y - dot * nY * 0.1;
                        onBoundary = true;
                    }
                }
            }
        }

        //check if the point has already been created (temporary, later going to use local geometry to determine rather than distance check)
        boolean check = false;
        if (inShape || onBoundary) for (int i = 0; i < Point.num; i = i + 1) {
            if (i != ID && Point.get(i).parentSoftbody == parentSoftbody) {
                double distance = (x - Point.get(i).posX) * (x - Point.get(i).posX) + (y - Point.get(i).posY) * (y - Point.get(i).posY);
                distance = Math.sqrt(distance);
                if (distance < Softbody.get(parentSoftbody).getPointRadius() || (onBoundary && distance < 2.0 * Softbody.get(parentSoftbody).getPointRadius())) {
                    inShape = false;
                    check = true;
                    break;
                }
            }
        }
        if (onBoundary && !check) inShape = true;

        return(new Triplet(onBoundary, new double[]{x, y}, inShape));
    }
    private double mod(double a, double b) {
        if (a > 0.0) return(a % b);
        else return(b - (-a % b));
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
    public int getAttachmentNum() {
        return(attachedIndices.size());
    }

    public void setMass(double mass) {
        this.mass = mass;
    }
    public void setMovingMotion(double[] motion) {
        vX = motion[0];
        vY = motion[1];
        aX = motion[2];
        aY = motion[3];
    }
    public void changeVX(double update) {
        vX = vX + update;
    }
    public void changeVY(double update) {
        vY = vY + update;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    public void setIsMovable(boolean a) {
        isMovable = a;
    }
}
