package PhysicsSim;
import java.util.ArrayList;
import java.awt.*;
import java.util.Arrays;

class Rigidbody {
    public static double COEFFICIENT_OF_RESTITUTION = 0.75; //in my personal opinion, this is mislabeled. In the literature, it is labeled this way,
    //but in order to preserve a linear interpolation in between the minimum and maximum energy conservation values (in keeping with conservation of momentum)
    //the squared version of this constant should be used. However, since it only ever appears within a square root, the constant was simplified to be the square root of this linear interpolation/
    //It's almost as bad of a general-consensus choice, in my opinion, as calling standard deviation standard and absolute mean not as standard (although it should be).
    public static double COEFFICIENT_OF_FRICTION_DYNAMIC = 0.35;
    public static double COEFFICIENT_OF_FRICTION_STATIC = 0.5;
    public static double GRAVITATIONAL_CONSTANT = 10.0;
    public static double AIR_DENSITY = 0.01204;
    public static double DRAG_COEFFICIENT = 0.95;
    public static double CLAMP_LIMIT = 0.5;
    public static double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public static double HOLD_DAMPING = 1.0;
    public static double MTV_EPSILON = 0.00165;


    public static boolean universalGravity = false;
    public static boolean airResistance = false;
    public static boolean buoyancy = false;
    public static boolean bounds = true;

    private static final ArrayList<Rigidbody> rigidbodies = new ArrayList<>();
    public static int num = 0;
    private final int ID;
    private boolean invertNormals = false;
    private boolean isMovable = true;
    //these ArrayLists are not constant, but the pointer stored to them is, hence the "final"
    private final ArrayList<Double> xPoints = new ArrayList<>();
    private final ArrayList<Double> yPoints = new ArrayList<>();
    private final ArrayList<Triangle> triangles = new ArrayList<>();
    private final ArrayList<Double[]> contactPoints = new ArrayList<>();
    private final ArrayList<Double[]> MTVs = new ArrayList<>();
    private final ArrayList<Integer> collidingIDs = new ArrayList<>();
    //for collidingIDs, 0 - infinity inclusive is reserved for other rigidbodies and -2 - -infinity is reserved for points. -1 is reserved for the walls
    private final double mass;
    private final double inertia;
    private final Color color;
    private final double area;
    private double largestDistance = 0.0;
    private double bottomBoundBox;
    private double topBoundBox;
    private double leftBoundBox;
    private double rightBoundBox;
    private static double lastMouseX = 250.0;
    private static double lastMouseY = 250.0;
    private static double currentMouseX = 250.0;
    private static double currentMouseY = 250.0;
    private static double flingX = 0.0;
    private static double flingY = 0.0;
    private boolean mouseHold = false;
    private boolean mouseRelease = false;

    public static double worldTopBound = 0.0;
    public static double worldBottomBound = 460.0;
    public static double worldLeftBound = 0.0;
    public static double worldRightBound = 485.0;

    private double posX;
    private double posY;
    private double vX;
    private double vY;
    private double aX;
    private double aY;
    private double angularV;
    private double angularA;

    private double newposX;
    private double newposY;
    private double newvX;
    private double newvY;
    private double newaX;
    private double newaY;
    private double newangularV;
    private double newangularA;

    private final double cX;
    private final double cY;

    public Rigidbody(double[] inputX, double[] inputY, double[] motion, double mass, Color color) {
        Triangle.setMTVEpsilon(MTV_EPSILON);
        ID = num;
        num = num + 1;
        this.mass = mass;
        this.color = color;
        posX = motion[0];
        posY = motion[1];
        vX = motion[2];
        vY = motion[3];
        aX = motion[4];
        aY = motion[5];
        angularV = motion[6];
        angularA = motion[7];

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

        System.out.println("Area: " + area + ", Moment of Inertia: " + inertia + ", Mass: " + this.mass + ", Difference against air density: " + ((mass / area) - AIR_DENSITY));
        System.out.println("Triangulation Categorization: " + Arrays.toString(pointExclusions));

    }

    //general motion. Copied and altered to fit Point in that file
    public static void step(double dt) {
        for (int i = 0; i < rigidbodies.size(); i = i + 1) {
            if(rigidbodies.get(i).isMovable()) rigidbodies.get(i).calcMotion(dt);
        }
        for (int i = 0; i < rigidbodies.size(); i = i + 1) {
            if (rigidbodies.get(i).isMovable()) rigidbodies.get(i).updateMotion();
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
        for (int i = 0; i < rigidbodies.size(); i = i + 1) {
            if (i != ID) {
                if (!intersecting) intersecting = checkForCollisionsBroad(Rigidbody.get(i));
                else checkForCollisionsBroad(Rigidbody.get(i));
            }
        }
        if (!intersecting && bounds) intersecting = checkForCollisionsWall();
        else if (bounds) checkForCollisionsWall();
        for (int i = 0; i < Point.num; i = i + 1) {
            if (!intersecting) intersecting = checkForCollisionsBroad(Point.get(i));
            else checkForCollisionsBroad(Point.get(i));
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
        newangularV = angularV;
        if (intersecting) for (int h = 0; h < contactPoints.size(); h = h + 1) {
            if (collidingIDs.get(h) == -1 || !getIsMovable(collidingIDs.get(h))) {
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
                double pivotAngularA = (inertia * angularA + mass * (ndotAx * rY + ndotAy * -rX)) / (inertia + mass * r * r);
                double pivotAngularV = ((inertia * newangularV + mass * (ndotVx * rY + ndotVy * -rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
                pivotAboutPoint(pivotAngularV * dt, contactPoints.get(h)[0], contactPoints.get(h)[1]);
                newposX = newposX + tdotVx * dt;
                newposY = newposY + tdotVy * dt;

                //next, update the velocity in an impulse-based reaction model

                double rPerp2x = -(contactPoints.get(h)[1] - getPosY(collidingIDs.get(h)));
                double rPerp2y = contactPoints.get(h)[0] - getPosX(collidingIDs.get(h));
                double jr = (newvX + rPerp1x * newangularV - getVX(collidingIDs.get(h)) - rPerp2x * getAngularV(collidingIDs.get(h))) * nX + (newvY + rPerp1y * newangularV - getVY(collidingIDs.get(h)) - rPerp2y * getAngularV(collidingIDs.get(h))) * nY;
                jr = jr * -(1.0 + COEFFICIENT_OF_RESTITUTION);
                double temp1 = rPerp1x * nX + rPerp1y * nY;
                double temp2 = rPerp2x * nX + rPerp2y * nY;
                jr = jr / ((1.0 / mass) + (1.0 / getMass(collidingIDs.get(h))) + (1.0 / inertia) * temp1 * temp1 + (1.0 / getInertia(collidingIDs.get(h))) * temp2 * temp2);
                double vtrel = (newvX + rPerp1x * newangularV) * -nY + (newvY + rPerp1y * newangularV) * nX;
                vtrel = vtrel - ((getVX(collidingIDs.get(h)) + rPerp2x * getAngularV(collidingIDs.get(h))) * -nY + (getVY(collidingIDs.get(h)) + rPerp2y * getAngularV(collidingIDs.get(h))) * nX);
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
                newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + ((fx * rPerp1x + fy * rPerp1y) / inertia) + angularA * dt;

                newaX = aX;
                newaY = aY;
                newangularA = angularA;
            }
        }

        if (!intersecting && !mouseHold)  {
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
        if (universalGravity) {
            double[] results = calculateGravity();
            newaX = results[0];
            newaY = results[1];
        }
        if (airResistance && vX * vX + vY * vY > CLAMP_LIMIT * CLAMP_LIMIT) {
            calculateAirResistance(dt);
        }

        if (mouseHold) {
            double vmX = (currentMouseX - lastMouseX) / dt;
            double vmY = (currentMouseY - lastMouseY) / dt;
            double rX = newposX - currentMouseX;
            double rY = newposY - currentMouseY;
            double r = Math.sqrt(rX * rX + rY * rY);
            double pivotAngularA = (inertia * newangularA + mass * (aX * -rY + aY * rX)) / (inertia + mass * r * r);
            double pivotAngularV = (inertia * newangularV + mass * ((newvX - vmX) * -rY + (newvY - vmY) * rX)) / (inertia + mass * r * r) + pivotAngularA * dt;
            newposX = newposX + currentMouseX - lastMouseX;
            newposY = newposY + currentMouseY - lastMouseY;
            pivotAboutPoint(pivotAngularV * dt, currentMouseX, currentMouseY);


            double damping = Math.min(Math.abs(newangularV), HOLD_DAMPING) * -Math.signum(newangularV);
            newangularV = newangularV + (damping * dt);
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
                    collidingIDs.add(-otherPoint.ID - 2);
                }
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
        Double[] pointOfContact = new Double[]{Double.NaN, Double.NaN};
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
        double pivotAngularA = (inertia * angularA + mass * (ndotAx * -rY + ndotAy * rX)) / (inertia + mass * r * r);
        double pivotAngularV = ((inertia * newangularV + mass * (ndotVx * -rY + ndotVy * rX)) / (inertia + mass * r * r)) + pivotAngularA * dt;
        pivotAboutPoint(pivotAngularV * dt, contactPoints.get(index)[0], contactPoints.get(index)[1]);
        newposX = newposX + tdotVx * dt;
        newposY = newposY + tdotVy * dt;

        double jr = (newvX + rY * newangularV) * nX + (newvY + -rX * newangularV) * nY;
        jr = jr * -(1.0 + COEFFICIENT_OF_RESTITUTION);
        double temp1 = rY * nX + -rX * nY;
        jr = jr / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);
        double vtrel = (newvX + rY * newangularV) * -nY + (newvY + -rX * newangularV) * nX;
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
        newangularV = newangularV + (jr / inertia) * (temp1) + ((fx * rY + fy * -rX) / inertia) + angularA * dt;

        newaX = aX;
        newaY = aY;
        newangularA = angularA;
    }
    private void updateMotion() {
        //clamp some values
        if (newvX * newvX + newvY * newvY < CLAMP_LIMIT * CLAMP_LIMIT) {
            newvX = 0.0;
            newvY = 0.0;
        }
        if (mass / area >= 1.5 * AIR_DENSITY && newangularV != 0.0 && Math.abs(newangularV) * largestDistance < CLAMP_LIMIT) {
            newangularV = 0.0;
        }
        posX = newposX;
        posY = newposY;
        vX = newvX;
        vY = newvY;
        aX = newaX;
        aY = newaY;
        angularV = newangularV;
        angularA = newangularA;
    }
    private double[] calculateGravity() {
        double sumaX = 0.0;
        double sumaY = 0.0;
        for (int i = 0; i < num; i = i + 1) {
            if (i != ID) {
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
    private void calculateAirResistance(double dt) {
        int length = xPoints.size();
        double magnitude1 = Math.sqrt(vX * vX + vY * vY);
        if (Double.isNaN(magnitude1)) magnitude1 = 0.0001;
        double uX = vX / magnitude1;
        double uY = vY / magnitude1;
        double crossArea = 0.0;
        for (int i = 0; i < length; i = i + 1) {
            double nX = -(yPoints.get((i + 1) % length) - yPoints.get(i));
            double nY = xPoints.get((i + 1) % length) - xPoints.get(i);
            if (invertNormals) {
                nX = -nX;
                nY = -nY;
            }
            if (nX * uX + nY * uY >= 0.0) {
                crossArea = crossArea + Math.abs((xPoints.get((i + 1) % length) - xPoints.get(i)) * -uY + (yPoints.get((i + 1) % length) - yPoints.get(i)) * uX);
            }
        }
        double magnitude2 = 0.5 * AIR_DENSITY * crossArea * DRAG_COEFFICIENT * magnitude1 * magnitude1;
        double[] fD = new double[]{-uX * magnitude2, -uY * magnitude2};
        if (Math.abs(fD[0] / mass) >= Math.abs(aX)) fD[0] = -aX;
        if (Math.abs(fD[1] / mass) >= Math.abs(aY)) fD[1] = -aY;
        double torqueSum = 0.0;
        //double[] forceSum = new double[]{0.0,0.0};
        magnitude1 = Math.sqrt(aX * aX + aY * aY);
        if (Double.isNaN(magnitude1)) magnitude1 = 0.0;
        magnitude2 = AIR_DENSITY * magnitude1 * area;
        double[] fB = new double[]{0.0, 0.0};
        if (buoyancy) {
            fB[0] = magnitude2 * (-aX / magnitude1);
            fB[1] = magnitude2 * (-aY / magnitude1);
        }
        for (int i = 0; i < length; i = i + 1) {
            double rPerpX = -yPoints.get(i);
            double rPerpY = xPoints.get(i);
            double r = Math.sqrt(rPerpX * rPerpX + rPerpY * rPerpY);
            if (Double.isNaN(r)) r = 0.0;
            double nX = -(yPoints.get((i + 1) % length) - yPoints.get(i));
            double nY = xPoints.get((i + 1) % length) - xPoints.get(i);
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
                //forceSum[0] = forceSum[0] + (fD[0] - torque * (rPerpX / (r * r)));
                //forceSum[1] = forceSum[1] + (fD[1] - torque * (rPerpY / (r * r)));
            }
        }
        double update = (fD[0] / mass) * dt + (fB[0] / mass) * dt;
        if (!Double.isNaN(update)) newvX = newvX + update;
        update = (fD[1] / mass) * dt + (fB[1] / mass) * dt;
        if (!Double.isNaN(update)) newvY = newvY + update;
        update = (torqueSum / inertia) * dt;
        if (!Double.isNaN(update)) newangularV = newangularV + update;
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
            if (rigidbodies.get(i).pointInside(new double[]{currentMouseX, currentMouseY})) {
                if (mousePressed) {
                    rigidbodies.get(i).mouseHold = true;
                    break;
                }
                else {
                    if (rigidbodies.get(i).mouseHold) rigidbodies.get(i).mouseRelease = true;
                    rigidbodies.get(i).mouseHold = false;
                }
            }
            else {
                rigidbodies.get(i).mouseHold = false;
            }
        }
    }
    private void pivotAboutPoint(double theta, double pointX, double pointY) {
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
    private static int mod(int a, int b) {
        if (a >= 0) return(a % b);
        else return(b - (Math.abs(a) % b));
    }

    public static Rigidbody get(int index) {
        return(rigidbodies.get(index));
    }
    public int[] getX() {
        int[] x = new int[xPoints.size()];
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            x[i] = (int) Math.round(xPoints.get(i) + posX);
        }
        return(x);
    }
    public int[] getY() {
        int[] y = new int[yPoints.size()];
        for (int i = 0; i < yPoints.size(); i = i + 1) {
            y[i] = (int) Math.round(yPoints.get(i) + posY);
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
    public double getAngularV() {
        return(angularV);
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
    public double getYPoints(int index) {
        return(yPoints.get(index));
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
        else if (index <= -2) return(Point.get(-index - 2).getX());
        else return(Double.NaN);
    }
    public static double getPosY(int index) {
        if (index >= 0) return(Rigidbody.get(index).getPosY());
        else if (index <= -2) return(Point.get(-index - 2).getY());
        else return(Double.NaN);
    }
    public static double getVX(int index) {
        if (index >= 0) return(Rigidbody.get(index).getVX());
        else if (index <= -2) return(Point.get(-index - 2).getVX());
        else return(Double.NaN);
    }
    public static double getVY(int index) {
        if (index >= 0) return(Rigidbody.get(index).getVY());
        else if (index <= -2) return(Point.get(-index - 2).getVY());
        else return(Double.NaN);
    }
    public static double getAngularV(int index) {
        if (index >= 0) return(Rigidbody.get(index).getAngularV());
        else if (index <= -2) return(0.0);
        else return(Double.NaN);
    }
    public static double getMass(int index) {
        if (index >= 0) return(Rigidbody.get(index).getMass());
        else if (index <= -2) return(Point.get(-index - 2).getMass());
        else return(Double.NaN);
    }
    public static double getInertia(int index) {
        if (index >= 0) return(Rigidbody.get(index).getInertia());
        else if (index <= -2) return(Point.get(-index - 2).getInertia());
        else return(Double.NaN);
    }
    public static boolean getIsMovable(int index) {
        if (index >= 0) return(Rigidbody.get(index).isMovable());
        else if (index <= -2) return(Point.get(-index - 2).isMovable());
        else return(false);
    }
}
