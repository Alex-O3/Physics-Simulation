package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;

class Point {
    public int simID;

    double mass;
    private final double area;
    double inertia;
    private Color color;
    private final double radius;
    private final double repulseRadius;
    final ArrayList<Controller> controllers = new ArrayList<>();
    private static final ArrayList<Point> points = new ArrayList<>();
    private final ArrayList<Double[]> contactPoints = new ArrayList<>();
    private final ArrayList<Double[]> MTVs = new ArrayList<>();
    final ArrayList<Integer> collidingIDs = new ArrayList<>();
    //same collidingIDs convention as Rigidbody
    private final ArrayList<Integer> attachedIndices = new ArrayList<>();
    private final ArrayList<Double> idealAttachmentLengths = new ArrayList<>();
    private final ArrayList<Double[]> attachmentDisplacementsFromCenter = new ArrayList<>();
    public int parentSoftbody = -1;
    public static int num = 0;
    public int ID = 0;

    public double COEFFICIENT_OF_RESTITUTION = 0.75;
    public double COEFFICIENT_OF_FRICTION = 0.35;
    public double GRAVITATIONAL_CONSTANT = 10.0;
    public double AIR_DENSITY = 0.01204;
    public double DRAG_COEFFICIENT = 0.95;
    public double CLAMP_LIMIT = 0.0;
    public double CONTACT_POINTS_MERGE_DISTANCE = 0.1;
    public double MOUSE_SPEED_LIMIT = 500.0;
    public double MTV_EPSILON = 0.00165;
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
    private boolean lockedRotation = false;
    private boolean adoptOnlyOtherSurface = false;

    private boolean isMovable = true;
    boolean isHitbox = false;
    boolean draw = true;
    private final boolean isSolidBall;
    private boolean attached = false;
    public boolean futureBoundary = false;
    public int minIndex1 = -1;
    public int minIndex2 = -1;
    public boolean universalGravity = false;
    public boolean airResistance = false;
    public boolean buoyancy = false;
    public boolean bounds = true;

    double posX;
    double posY;
    double vX;
    double vY;
    double aX;
    double aY;
    double angularV;
    double angularA;
    private final double[] initialExternalForces = new double[2];
    private final double initialExternalTorque;
    public double[] testRotationWithDrawingPoint = new double[]{0.0, 50.0};

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
    private int vUpdateCount = 0;
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
        angularV = motion[6];
        initialExternalTorque = motion[7];
        newposX = posX;
        newposY = posY;
        newvX = vX;
        newvY = vY;
        newaX = aX;
        newaY = aY;
        newangularV = angularV;
        newangularA = angularA;
        COEFFICIENT_OF_RESTITUTION = Simulation.get(simID).COEFFICIENT_OF_RESTITUTION;
        COEFFICIENT_OF_FRICTION = Simulation.get(simID).COEFFICIENT_OF_FRICTION;
        this.radius = radius;
        repulseRadius = REPULSE_RADIUS_MULTIPLIER * radius;
        this.mass = mass;
        inertia = 0.5 * mass * radius * radius;
        this.color = color;
        this.isSolidBall = isSolidBall;
        area = Math.PI * getSolidRadius() * getSolidRadius();
        points.add(this);
        Simulation.get(simID).getSAPCell(0, 0).addBox(ID * -2 - 2);
        Simulation.get(simID).BVHtrees.get(0).addBox(ID * -2 - 2);
    }

    //general motion copied from Rigidbody and adjusted to account for 0 angular movement and other factors specific to Point
    public static void step(double dt, int simID) {
        for (int i = 0; i < points.size(); i = i + 1) {
            if (points.get(i).simID == simID) {
                if (points.get(i).lockedRotation) {
                    points.get(i).angularV = 0.0;
                    points.get(i).angularA = 0.0;
                }
                if (points.get(i).isMovable() && !points.get(i).isHitbox) {
                    points.get(i).calcMotion(dt);
                }
                else if (points.get(i).vX != 0.0 || points.get(i).vY != 0.0 || points.get(i).angularV != 0.0) {
                    points.get(i).integrateMotion(dt);
                }
                //if (points.get(i).isHitbox) points.get(i).findCollisions();
            }
        }
    }
    public static void updateMotion(double dt, int simID) {
        for (int i = 0; i < points.size(); i = i + 1) {
            if (points.get(i).simID == simID) {
                if (points.get(i).isMovable()) points.get(i).updateMotion(dt);
                else if (points.get(i).vX != 0.0 || points.get(i).vY != 0.0 || points.get(i).angularV != 0.0) points.get(i).updateMotion(dt);
            }
        }
    }
    private void calcMotion(double dt) {
        newposX = posX;
        newposY = posY;
        double[] posChange = new double[]{0.0, 0.0};
        newvX = vX;
        newvY = vY;
        newangularV = angularV;

        //first update joints, starting with mouse
        if (mouseHold) {
            double vmX = (currentMouseX - lastMouseX) / dt;
            double vmY = (currentMouseY - lastMouseY) / dt;
            double magnitude = Math.sqrt(vmX * vmX + vmY * vmY);
            if (magnitude > MOUSE_SPEED_LIMIT) {
                vmX *= MOUSE_SPEED_LIMIT / magnitude;
                vmY *= MOUSE_SPEED_LIMIT / magnitude;
            }
            double rX = currentMouseX - posX;
            double rY = currentMouseY - posY;
            double vimprel = vX - rY * angularV - vmX;
            double jrX = -vimprel / ((1.0 / mass) + (1.0 / inertia) * rY * rY);
            vimprel = vY + rX * angularV - vmY;
            double jrY = -vimprel / ((1.0 / mass) + (1.0 / inertia) * rX * rX);
            newvX += jrX / mass;
            newvY += jrY / mass;
            newangularV += (jrX * -rY + jrY * rX) / inertia;
        }
        if (mouseRelease) {
            newvX = newvX + flingX;
            newvY = newvY + flingY;
            mouseRelease = false;
        }
        /*for (int i = 0; i < attachedIndices.size(); i = i + 1) {
            double rX1 = getAttachmentDisplacement(i)[0];
            double vmX = Rigidbody.getVX(attachedIndices.get(i))
            double vmY = (currentMouseY - lastMouseY) / dt;
            double dX = currentMouseX - posX;
            double dY = currentMouseY - posY;
            double vimprel = vX - dY * angularV - vmX;
            double jrX = -vimprel / ((1.0 / mass) + (1.0 / inertia) * dY * dY);
            vimprel = vY + dX * angularV - vmY;
            double jrY = -vimprel / ((1.0 / mass) + (1.0 / inertia) * dX * dX);
            newvX += jrX / mass;
            newvY += jrY / mass;
            newangularV += (jrX * -dY + jrY * dX) / inertia;
        }*/

        //check for collisions and do one of two options for updating motion based on whether the rigidbody is colliding with another
        boolean intersecting = !collidingIDs.isEmpty();

        if (intersecting) for (int h = 0; h < contactPoints.size(); h = h + 1) {
            posChange[0] += MTVs.get(h)[0];
            posChange[1] += MTVs.get(h)[1];
            double magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
            if (Double.isNaN(magnitude)) magnitude = 0.0;
            double nX = MTVs.get(h)[0] / magnitude;
            double nY = MTVs.get(h)[1] / magnitude;
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            double rX = contactPoints.get(h)[0] - posX;
            double rY = contactPoints.get(h)[1] - posY;
            //the normal here is assumed to point towards this rigidbody and away from the other. For all intents and purposes,
            //this choice should not matter so long as it is treated consistently
            double rPerp1x = -rY;
            double rPerp1y = rX;
            double rPerp2x = 0.0;
            double rPerp2y = 0.0;
            if (collidingIDs.get(h) <= -2 && mod(collidingIDs.get(h), 2) == 1) {
                double radius = Point.get(((-collidingIDs.get(h) - 1) / 2) - 1).getSolidRadius();
                rPerp2x = -nY * radius;
                rPerp2y = nX * radius;
            }
            else if (collidingIDs.get(h) != -1) {
                rPerp2x = -(contactPoints.get(h)[1] - Rigidbody.getPosY(collidingIDs.get(h)));
                rPerp2y = contactPoints.get(h)[0] - Rigidbody.getPosX(collidingIDs.get(h));
            }
            //ensure the objects are actually moving towards each other
            if (((vX + rPerp1x * angularV) - (Rigidbody.getVX(collidingIDs.get(h)) + rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h)))) * nX + ((vY + rPerp1y * angularV) - (Rigidbody.getVY(collidingIDs.get(h)) + rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h)))) * nY > 0.0) {
                continue;
            }
            if (collidingIDs.get(h) == -1 || !Rigidbody.getIsMovable(collidingIDs.get(h))) {
                calcMotionInfiniteMass(h);
                continue;
            }
            double jr = (vX + rPerp1x * angularV - Rigidbody.getVX(collidingIDs.get(h)) - rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h))) * nX + (vY + rPerp1y * angularV - Rigidbody.getVY(collidingIDs.get(h)) - rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h))) * nY;
            jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(h)));
            double temp1 = rPerp1x * nX + rPerp1y * nY;
            double temp2 = rPerp2x * nX + rPerp2y * nY;
            jr = jr / ((1.0 / mass) + (1.0 / Rigidbody.getMass(collidingIDs.get(h))) + (1.0 / inertia) * temp1 * temp1 + (1.0 / Rigidbody.getInertia(collidingIDs.get(h))) * temp2 * temp2);

            double vtrel = ((vX + rPerp1x * angularV) - (Rigidbody.getVX(collidingIDs.get(h)) + rPerp2x * Rigidbody.getAngularV(collidingIDs.get(h)))) * -nY;
            vtrel += ((vY + rPerp1y * angularV) - (Rigidbody.getVY(collidingIDs.get(h)) + rPerp2y * Rigidbody.getAngularV(collidingIDs.get(h)))) * nX;
            double friction = getCOEFFICIENT_OF_FRICTION(collidingIDs.get(h)) * jr * -Math.signum(vtrel);
            temp1 = rPerp1x * -nY + rPerp1y * nX;
            temp2 = rPerp2x * -nY + rPerp2y * nX;
            double frictionMax = -vtrel / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1 + (1.0 / Rigidbody.getMass(collidingIDs.get(h))) + (1.0 / Rigidbody.getInertia(collidingIDs.get(h))) * temp2 * temp2);
            friction = Math.min(Math.abs(frictionMax), Math.abs(friction)) * -Math.signum(vtrel);

            newvX = newvX + (jr / mass) * nX + (friction / mass) * -nY;
            newvY = newvY + (jr / mass) * nY + (friction / mass) * nX;
            if (!lockedRotation) newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + (friction / inertia) * (rPerp1x * -nY + rPerp1y * nX);


            if (collidingIDs.get(h) <= -2 && mod(collidingIDs.get(h), 2) == 1) {
                int tempIndex = ((-collidingIDs.get(h) - 1) / 2) - 1;
                int softbodyIndex = Point.get(tempIndex).parentSoftbody;
                int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(tempIndex);
                Point point1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex));
                Point point2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % Softbody.get(softbodyIndex).boundarySize()));
                double update = -(jr / Rigidbody.getMass(collidingIDs.get(h))) * nX;
                point1.changeVX(update);
                point2.changeVX(update);
                update = -(jr / Rigidbody.getMass(collidingIDs.get(h))) * nY;
                point1.changeVY(update);
                point2.changeVY(update);

                magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
                double multiplier = -((magnitude - MTV_EPSILON) / magnitude) * (mass / (Rigidbody.getMass(collidingIDs.get(h))));
                multiplier *= 1.0 + (MTV_EPSILON / Math.abs(multiplier));
                point1.changeX(MTVs.get(h)[0] * multiplier);
                point2.changeX(MTVs.get(h)[0] * multiplier);
                point1.changeY(MTVs.get(h)[1] * multiplier);
                point2.changeY(MTVs.get(h)[1] * multiplier);
            }
        }

        integrateMotion(dt);

        newposX += posChange[0];
        newposY += posChange[1];

        newaX = initialExternalForces[0];
        newaY = initialExternalForces[1];
        if (!lockedRotation) newangularA = initialExternalTorque;
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

    }
    private void calcMotionInfiniteMass(int index) {
        double rX = contactPoints.get(index)[0] - posX;
        double rY = contactPoints.get(index)[1] - posY;
        double rPerp1x = -rY;
        double rPerp1y = rX;
        double magnitude = Math.sqrt(MTVs.get(index)[0] * MTVs.get(index)[0] + MTVs.get(index)[1] * MTVs.get(index)[1]);
        double nX = MTVs.get(index)[0] / magnitude;
        double nY = MTVs.get(index)[1] / magnitude;
        if (Double.isNaN(nX) || Double.isNaN(nY)) {
            nX = 0.0;
            nY = 0.0;
        }
        double rPerp2x = -(contactPoints.get(index)[1] - Rigidbody.getPosY(collidingIDs.get(index)));
        double rPerp2y = contactPoints.get(index)[0] - Rigidbody.getPosX(collidingIDs.get(index));
        if (Double.isNaN(rPerp2x)) {
            rPerp2x = 0.0;
            rPerp2y = 0.0;
        }
        double jr = (vX + rPerp1x * angularV - Rigidbody.getVX(collidingIDs.get(index)) - rPerp2x * Rigidbody.getAngularV(collidingIDs.get(index))) * nX + (vY + rPerp1y * angularV - Rigidbody.getVY(collidingIDs.get(index)) - rPerp2y * Rigidbody.getAngularV(collidingIDs.get(index))) * nY;
        jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(index)));
        double temp1 = rPerp1x * nX + rPerp1y * nY;
        jr = jr / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);

        double vtrel = ((vX + rPerp1x * angularV) - (Rigidbody.getVX(collidingIDs.get(index)) + rPerp2x * Rigidbody.getAngularV(collidingIDs.get(index)))) * -nY;
        vtrel += ((vY + rPerp1y * angularV) - (Rigidbody.getVY(collidingIDs.get(index)) + rPerp2y * Rigidbody.getAngularV(collidingIDs.get(index)))) * nX;
        double friction = getCOEFFICIENT_OF_FRICTION(collidingIDs.get(index)) * jr * -Math.signum(vtrel);
        temp1 = rPerp1x * -nY + rPerp1y * nX;
        double frictionMax = -vtrel / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);
        friction = Math.min(Math.abs(frictionMax), Math.abs(friction)) * -Math.signum(vtrel);

        newvX = newvX + (jr / mass) * nX + (friction / mass) * -nY;
        newvY = newvY + (jr / mass) * nY + (friction / mass) * nX;
        if (!lockedRotation) newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + (friction / inertia) * (rPerp1x * -nY + rPerp1y * nX);

    }
    private void integrateMotion(double dt) {
        newposX += newvX * dt;
        newposY += newvY * dt;
        rotateAboutCenter(dt);
        newvX += aX * dt;
        newvY += aY * dt;
        newangularV += angularA * dt;
    }
    static void clearCollisionInformation(int simID) {
        for (Point point : points) {
            if (point.simID != simID) continue;
            point.contactPoints.clear();
            point.MTVs.clear();
            point.collidingIDs.clear();
        }
    }
    static void finalizeCollisionInformation(int simID) {
        for (Point point : points) {
            if (point.simID != simID) continue;
            if (point.bounds) point.checkForCollisionsWall();

            //prune the list for points too close to one another (same point, but different triangle with floating point precision differences)
            if (!point.contactPoints.isEmpty()) {
                for (int i = 0; i < point.contactPoints.size(); i = i + 1) {
                    for (int j = i + 1; j < point.contactPoints.size(); j = j + 1) {
                        if (i != j && !Double.isNaN(point.contactPoints.get(i)[0]) && !Double.isNaN(point.contactPoints.get(j)[0])) {
                            double temp1 = point.contactPoints.get(i)[0] - point.contactPoints.get(j)[0];
                            double temp2 = point.contactPoints.get(i)[1] - point.contactPoints.get(j)[1];
                            double distance = temp1 * temp1 + temp2 * temp2;
                            distance = Math.sqrt(Math.max(distance, 0.0));
                            if (distance <= point.CONTACT_POINTS_MERGE_DISTANCE) {
                                point.contactPoints.set(j, new Double[]{Double.NaN, Double.NaN});
                            }
                        }
                    }
                }
            }
            int length = point.contactPoints.size();
            for (int i = 0; i < length; i = i + 1) {
                if (i >= point.contactPoints.size()) break;
                if (Double.isNaN(point.contactPoints.get(i)[0])) {
                    point.contactPoints.remove(i);
                    point.MTVs.remove(i);
                    point.collidingIDs.remove(i);
                    i = i - 1;
                }
            }
        }
    }
    boolean checkForCollisionsNarrow(Rigidbody otherObject) {
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
    boolean checkForCollisions(Point otherPoint) {
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
    boolean checkForCollisionsNarrow(Softbody softbody) {
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
        if (parentSoftbody != -1) for (int i = 0; i < Softbody.num; i = i + 1) {
            if (Softbody.get(i).simID == simID && i != parentSoftbody) for (int j = 0; j < Softbody.get(i).size(); j = j + 1) {
                double dx = posX - Softbody.get(i).getMember(j).getX();
                double dy = posY - Softbody.get(i).getMember(j).getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance > 0.0 && distance < repulseRadius + Softbody.get(i).getMember(j).getRepulseRadius()) {
                    double aMagnitude = Math.max(0.0, REPULSION_STRENGTH * (((Softbody.get(i).getMember(j).getRepulseRadius() + repulseRadius) / distance) - 1.0));
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
            double dx = Rigidbody.getPosX(attachedIndices.get(i)) - posX;
            double dy = Rigidbody.getPosY(attachedIndices.get(i)) - posY;
            double distance = Math.sqrt(dx * dx + dy * dy);
            double nX = dx / distance;
            double nY = dy / distance;
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            double[] fS = new double[2];
            fS[0] = HOOKE_SPRING_CONSTANT * (dx - idealAttachmentLengths.get(i) * nX);
            fS[1] = HOOKE_SPRING_CONSTANT * (dy - idealAttachmentLengths.get(i) * nY);
            double multiplier = HOOKE_SPRING_CONSTANT * SPRING_MAX_DIST_MULTIPLIER * idealAttachmentLengths.get(i);
            double[] maxF = new double[]{multiplier * nX, multiplier * nY};
            fS[0] = Math.min(Math.abs(fS[0]), Math.abs(maxF[0])) * Math.signum(fS[0]);
            fS[1] = Math.min(Math.abs(fS[1]), Math.abs(maxF[1])) * Math.signum(fS[1]);
            multiplier = ((vX - Rigidbody.getVX(attachedIndices.get(i))) * nX + (vY - Rigidbody.getVY(attachedIndices.get(i))) * nY) * local_SPRING_DAMPING_COEFFICIENT;
            double[] fD = new double[]{-nX * multiplier, -nY * multiplier};
            if (fD[0] * fD[0] + fD[1] * fD[1] < fS[0] * fS[0] + fS[1] * fS[1]) {
                newaX += (fS[0] + fD[0]) / mass;
                newaY += (fS[1] + fD[1]) / mass;
            }
        }
    }
    private void rotateAboutCenter(double dt) {
        //temporary
        double x = testRotationWithDrawingPoint[0];
        double y = testRotationWithDrawingPoint[1];
        double theta = angularV * dt;
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        testRotationWithDrawingPoint[0] = x * cos - y * sin;
        testRotationWithDrawingPoint[1] = y * cos + x * sin;
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
        multiplier = Math.min(multiplier, MOUSE_SPEED_LIMIT) / multiplier;
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
        newvX += otherUpdateVX / (vUpdateCount + collidingIDs.size());
        newvY += otherUpdateVY / (vUpdateCount + collidingIDs.size());
        vUpdateCount = 0;
        newaX += otherUpdateAX;
        newaY += otherUpdateAY;
        //clamp some values
        if (newvX * newvX + newvY * newvY < CLAMP_LIMIT * CLAMP_LIMIT) {
            newvX = 0.0;
            newvY = 0.0;
        }
        if (mass / area >= 1.5 * AIR_DENSITY && newangularV != 0.0 && Math.abs(newangularV) * getSolidRadius() < CLAMP_LIMIT) {
            newangularV = 0.0;
        }
        if (!Double.isNaN(newposX)) posX = newposX;
        if (!Double.isNaN(newposY)) posY = newposY;
        if (!Double.isNaN(newvX)) vX = newvX;
        if (!Double.isNaN(newvY)) vY = newvY;
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

    public void attach(Point other) {
        int index = -ID * 2 - 2;
        int otherIndex = -other.ID * 2 - 2;
        if (!attachedIndices.contains(otherIndex) && !other.attachedIndices.contains(index) && other.simID == simID) {
            attached = true;
            attachedIndices.add(otherIndex);
            other.attached = true;
            other.attachedIndices.add(index);
            double distance = (other.posX - posX) * (other.posX - posX) + (other.posY - posY) * (other.posY - posY);
            distance = Math.sqrt(distance);
            idealAttachmentLengths.add(distance);
            other.idealAttachmentLengths.add(distance);
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
                Point generatedPoint = new Point(new double[]{x[i], y[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Softbody.get(parentSoftbody).getPointRadius(), 1.0, Softbody.get(parentSoftbody).getColor(), true, simID);
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
    public double getAngularV() {
        return(angularV);
    }
    public double getAngularA() {
        return(angularA);
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
    public Point getPointAttachment(int index) {
        index = -attachedIndices.get(index) / 2 - 1;
        return(points.get(index));
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
                if (Softbody.get(parentSoftbody).boundaryMembers.contains(-attachedIndices.get(i) / 2 - 1)) count += 1;
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
    public double getCOEFFICIENT_OF_FRICTION(int index) {
        if (index >= 0) {
            if (adoptOnlyOtherSurface) return(Rigidbody.get(index).COEFFICIENT_OF_FRICTION);
            else return((Rigidbody.get(index).COEFFICIENT_OF_FRICTION + COEFFICIENT_OF_FRICTION) * 0.5);
        }
        else if (index <= -2 && mod(index, 2) == 0) {
            if (adoptOnlyOtherSurface) return(Point.get((-index / 2) - 1).COEFFICIENT_OF_FRICTION);
            else return((Point.get((-index / 2) - 1).COEFFICIENT_OF_FRICTION + COEFFICIENT_OF_FRICTION) * 0.5);
        }
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Point.get((-index - 1) / 2 - 1).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf((-index - 1) / 2 - 1);
            double val1 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).COEFFICIENT_OF_FRICTION;
            double val2 = Point.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).COEFFICIENT_OF_FRICTION;
            double a = 0.5 * (val1 + val2);
            if (adoptOnlyOtherSurface) return(a);
            else return(0.5 * (a + COEFFICIENT_OF_FRICTION));
        }
        else {
            if (adoptOnlyOtherSurface) return(Simulation.get(simID).COEFFICIENT_OF_FRICTION);
            else return(Simulation.get(simID).COEFFICIENT_OF_FRICTION + COEFFICIENT_OF_FRICTION * 0.5);
        }
    }
    public void lockRotation(boolean lockRotation) {
        lockedRotation = lockRotation;
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
