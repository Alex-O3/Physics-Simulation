package PhysicsSim;
import java.util.ArrayList;
import java.awt.*;

class Rigidbody {
    int simID;
    final Simulation sim;
    final GeometricType geometry;

    //related to softbodies
    int parentSoftbody = -1;
    int minIndex1 = -1;
    int minIndex2 = -1;
    boolean futureBoundary = false;
    private double repulseRadius = 0.0;

    //local physical constants
    double COEFFICIENT_OF_RESTITUTION;
    double COEFFICIENT_OF_FRICTION;

    //generic state information
    private static final ArrayList<Rigidbody> rigidbodies = new ArrayList<>();
    static int num = 0;
    final int ID;
    private boolean isMovable = true;
    boolean isHitbox = false;
    boolean isAttached = false;
    boolean draw = true;

    //these ArrayLists are not constant, but the pointer stored to them is, hence the "final"
    final ArrayList<Controller> controllers = new ArrayList<>();
    final ArrayList<Double[]> contactPoints = new ArrayList<>();
    final ArrayList<Double[]> MTVs = new ArrayList<>();
    final ArrayList<Integer> collidingIDs = new ArrayList<>();
    final ArrayList<Joint> attachments = new ArrayList<>();
    //for collidingIDs, 0 - infinity inclusive is reserved for other rigidbodies and
    // -2 - -infinity numbers is reserved for softbody edges. These are then converted by n -> -n -2 to get the
    //ID of the rigidbody involved in that collision and the other side of the softbody edge.
    //AABBs refer to softbodies as their localID mapped n -> -n - 2, and not edges.

    double mass;
    double inertia;
    private final double area;

    //mouse fields
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

    //movement info
    private double posX;
    private double posY;
    private double vX;
    private double vY;
    private double aX;
    private double aY;
    private double angularV;
    private double angularA;
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

    public Rigidbody(GeometricType geometry, double[] motion, double mass, Color color, int simID) {
        this.simID = simID;
        ID = num;
        num = num + 1;
        this.mass = mass;
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

        this.geometry = geometry;
        inertia = mass * geometry.getMasslessInertia();
        area = geometry.getArea();
        geometry.setColor(color);
        rigidbodies.add(this);

        Simulation.get(simID).getSAPCell(0, 0).addBox(ID);
        Simulation.get(simID).BVHtrees.get(0).addBox(ID);
        sim = Simulation.get(simID);

        COEFFICIENT_OF_RESTITUTION = sim.COEFFICIENT_OF_RESTITUTION;
        COEFFICIENT_OF_FRICTION = sim.COEFFICIENT_OF_FRICTION;
    }

    //general motion. Copied and altered to fit Point in that file
    private void integrateMotion(double dt) {
        newposX += newvX * dt;
        newposY += newvY * dt;
        rotateAroundCenter(dt);
        newvX += aX * dt;
        newvY += aY * dt;
        newangularV += angularA * dt;
    }
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
                    rigidbodies.get(i).newposX = rigidbodies.get(i).posX;
                    rigidbodies.get(i).newposY = rigidbodies.get(i).posY;
                    rigidbodies.get(i).integrateMotion(dt);
                }
                //if (rigidbodies.get(i).isHitbox) rigidbodies.get(i).findCollisions();
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
    private void updateMotion(double dt) {
        //clamp some values
        if (newvX * newvX + newvY * newvY < sim.CLAMP_LIMIT * sim.CLAMP_LIMIT) {
            newvX = 0.0;
            newvY = 0.0;
        }
        if (mass / area >= 1.5 * sim.AIR_DENSITY && newangularV != 0.0 && Math.abs(newangularV) * geometry.largestDistanceSquared < sim.CLAMP_LIMIT * sim.CLAMP_LIMIT) {
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

        if (!controllers.isEmpty()) {
            //take into account controllers first once 'onGround' is known
            boolean onGround = false;
            double[] maxGroundVelocity = new double[]{0.0, 0.0};
            double maxGroundVelocityMagnitude = 0.0;
            for (int i = 0; i < collidingIDs.size(); i = i + 1) {
                if (Rigidbody.getMaterial(collidingIDs.get(i)).name.contains("Ground")) {
                    if (contactPoints.get(i)[0] * initialExternalForces[0] + contactPoints.get(i)[1] * initialExternalForces[1]
                            > posX * initialExternalForces[0] + posY * initialExternalForces[1]) {
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
                boolean touchingObstacle = false;
                for (int j = 0; j < collidingIDs.size(); j++) {
                    if (!Rigidbody.get(collidingIDs.get(j)).isMovable()) {
                        touchingObstacle = true;
                        break;
                    }
                }
                controllers.get(i).respondToKey(dt, Simulation.get(simID).display.keysCache, Simulation.get(simID).display.firstPress,
                        Simulation.get(simID).display.keyReleasedFirstTime, onGround, touchingObstacle, maxGroundVelocity);
            }
        }
    }
    static void clearCollisionInformation(int simID) {
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody.simID != simID) continue;
            rigidbody.contactPoints.clear();
            rigidbody.MTVs.clear();
            rigidbody.collidingIDs.clear();
        }
    }
    public boolean checkCollisions(Rigidbody otherObject) {
        return(geometry.findCollisions(otherObject.geometry));
    }
    public boolean checkCollisions(Softbody softbody) {
        return(geometry.findCollisions(softbody));
    }
    private boolean checkForCollisionsWall() {
        return(geometry.checkForCollisionsWall());
    }
    static void finalizeCollisionInformation(int simID) {
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody.simID != simID) continue;
            if (rigidbody.sim.bounds) rigidbody.checkForCollisionsWall();

            //prune the list for points too close to one another (same point, but different triangle with floating point precision differences)
            if (!rigidbody.contactPoints.isEmpty()) {
                for (int i = 0; i < rigidbody.contactPoints.size(); i = i + 1) {
                    for (int j = i + 1; j < rigidbody.contactPoints.size(); j = j + 1) {
                        if (i != j && !Double.isNaN(rigidbody.contactPoints.get(i)[0]) && !Double.isNaN(rigidbody.contactPoints.get(j)[0])) {
                            double temp1 = rigidbody.contactPoints.get(i)[0] - rigidbody.contactPoints.get(j)[0];
                            double temp2 = rigidbody.contactPoints.get(i)[1] - rigidbody.contactPoints.get(j)[1];
                            double distance = temp1 * temp1 + temp2 * temp2;
                            distance = Math.sqrt(Math.max(distance, 0.0));
                            if (distance <= rigidbody.sim.CONTACT_POINTS_MERGE_DISTANCE) {
                                rigidbody.contactPoints.set(j, new Double[]{Double.NaN, Double.NaN});
                            }
                        }
                    }
                }
            }
            int length = rigidbody.contactPoints.size();
            for (int i = 0; i < length; i = i + 1) {
                if (i >= rigidbody.contactPoints.size()) break;
                if (Double.isNaN(rigidbody.contactPoints.get(i)[0])) {
                    rigidbody.contactPoints.remove(i);
                    rigidbody.MTVs.remove(i);
                    rigidbody.collidingIDs.remove(i);
                    i = i - 1;
                }
            }
        }
    }
    private void calcMotion(double dt) {
        newposX = posX;
        newposY = posY;
        newvX = vX;
        newvY = vY;
        newangularV = angularV;

        //first handle joints, starting with mouse joint (though not technically a joint)
        if (mouseHold) {
            double vmX = (currentMouseX - lastMouseX) / dt;
            double vmY = (currentMouseY - lastMouseY) / dt;
            double magnitude = Math.sqrt(vmX * vmX + vmY * vmY);
            if (magnitude > sim.MOUSE_SPEED_LIMIT) {
                vmX *= sim.MOUSE_SPEED_LIMIT / magnitude;
                vmY *= sim.MOUSE_SPEED_LIMIT / magnitude;
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

        //check for collisions and do one of two options for updating motion based on whether the rigidbody is colliding with another
        boolean intersecting = !collidingIDs.isEmpty();
        int countOfValidCollisionImpulses = 0;

        if (intersecting) for (int h = 0; h < contactPoints.size(); h = h + 1) {
            newposX += MTVs.get(h)[0];
            newposY += MTVs.get(h)[1];
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
            if (collidingIDs.get(h) >= 0) {
                rPerp2x = -(contactPoints.get(h)[1] - getPosY(collidingIDs.get(h)));
                rPerp2y = contactPoints.get(h)[0] - getPosX(collidingIDs.get(h));
            }
            //ensure the objects are actually moving towards each other
            if (((vX + rPerp1x * angularV) - (getVX(collidingIDs.get(h)) + rPerp2x * getAngularV(collidingIDs.get(h)))) * nX + ((vY + rPerp1y * angularV) - (getVY(collidingIDs.get(h)) + rPerp2y * getAngularV(collidingIDs.get(h)))) * nY > 0.0) {
                countOfValidCollisionImpulses += 1;
                continue;
            }
            if (collidingIDs.get(h) == -1 || !getIsMovable(collidingIDs.get(h))) {
                calcMotionInfiniteMass(h);
                continue;
            }
            double jr = (vX + rPerp1x * angularV - getVX(collidingIDs.get(h)) - rPerp2x * getAngularV(collidingIDs.get(h))) * nX + (vY + rPerp1y * angularV - getVY(collidingIDs.get(h)) - rPerp2y * getAngularV(collidingIDs.get(h))) * nY;
            jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(h)));
            double temp1 = rPerp1x * nX + rPerp1y * nY;
            double temp2 = rPerp2x * nX + rPerp2y * nY;
            jr = jr / ((1.0 / mass) + (1.0 / getMass(collidingIDs.get(h))) + (1.0 / inertia) * temp1 * temp1 + (1.0 / getInertia(collidingIDs.get(h))) * temp2 * temp2);

            double vtrel = ((vX + rPerp1x * angularV) - (getVX(collidingIDs.get(h)) + rPerp2x * getAngularV(collidingIDs.get(h)))) * -nY;
            vtrel += ((vY + rPerp1y * angularV) - (getVY(collidingIDs.get(h)) + rPerp2y * getAngularV(collidingIDs.get(h)))) * nX;
            double friction = getCOEFFICIENT_OF_FRICTION(collidingIDs.get(h)) * jr * -Math.signum(vtrel);
            temp1 = rPerp1x * -nY + rPerp1y * nX;
            temp2 = rPerp2x * -nY + rPerp2y * nX;
            double frictionMax = -vtrel / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1 + (1.0 / getMass(collidingIDs.get(h))) + (1.0 / getInertia(collidingIDs.get(h))) * temp2 * temp2);
            friction = Math.min(Math.abs(frictionMax), Math.abs(friction)) * -Math.signum(vtrel);

            newvX = newvX + (jr / mass) * nX + (friction / mass) * -nY;
            newvY = newvY + (jr / mass) * nY + (friction / mass) * nX;
            if (!lockedRotation) newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + (friction / inertia) * (rPerp1x * -nY + rPerp1y * nX);

            if (collidingIDs.get(h) <= -2) {//for the case of softbodies, handle later
                int tempIndex = convertSoftbodyEdge(collidingIDs.get(h));
                int softbodyIndex = Rigidbody.get(tempIndex).parentSoftbody;
                int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(tempIndex);
                Rigidbody point1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex));
                Rigidbody point2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % Softbody.get(softbodyIndex).boundarySize()));
                double update = -(jr / getMass(collidingIDs.get(h))) * nX;
                point1.newvX += update;
                point2.newvX += update;
                update = -(jr / getMass(collidingIDs.get(h))) * nY;
                point1.newvY += update;
                point2.newvY += update;

                magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
                double multiplier = -((magnitude - sim.MTV_EPSILON) / magnitude) * (mass / (getMass(collidingIDs.get(h))));
                multiplier *= 1.0 + (sim.MTV_EPSILON / Math.abs(multiplier));
                point1.newposX += MTVs.get(h)[0] * multiplier;
                point2.newposX += MTVs.get(h)[0] * multiplier;
                point1.newposY += MTVs.get(h)[1] * multiplier;
                point2.newposY += MTVs.get(h)[1] * multiplier;
            }
        }
        if (countOfValidCollisionImpulses > 0) {
            newvX = vX + (newvX - vX) / countOfValidCollisionImpulses;
            newvY = vY + (newvY - vY) / countOfValidCollisionImpulses;
        }

        newaX = initialExternalForces[0];
        newaY = initialExternalForces[1];
        if (!lockedRotation) newangularA = initialExternalTorque;
        if (sim.universalGravity) {
            double[] results = calculateGravity();
            newaX += results[0];
            newaY += results[1];
        }
        if (sim.airResistance) {
            double[] results = geometry.calculateAirResistance(sim.AIR_DENSITY, sim.DRAG_COEFFICIENT, new double[]{vX, vY});
            newaX += results[0] / mass;
            newaY += results[1] / mass;
            newangularA += results[2] / inertia;
        }
        calculateRepulsion();
        calculateJointForceEffects();

        integrateMotion(dt);
    }
    private void calcMotionInfiniteMass(int index) {
        double rX = contactPoints.get(index)[0] - posX;
        double rY = contactPoints.get(index)[1] - posY;
        double rPerp1x = -rY;
        double rPerp1y = rX;
        double rPerp2x = 0.0;
        double rPerp2y = 0.0;
        if (collidingIDs.get(index) >= 0) {
            rPerp2x = -(contactPoints.get(index)[1] - getPosY(collidingIDs.get(index)));
            rPerp2y = contactPoints.get(index)[0] - getPosX(collidingIDs.get(index));
        }
        double magnitude = Math.sqrt(MTVs.get(index)[0] * MTVs.get(index)[0] + MTVs.get(index)[1] * MTVs.get(index)[1]);
        double nX = MTVs.get(index)[0] / magnitude;
        double nY = MTVs.get(index)[1] / magnitude;
        if (Double.isNaN(nX) || Double.isNaN(nY)) {
            nX = 0.0;
            nY = 0.0;
        }
        double jr = (vX + rPerp1x * angularV) * nX + (vY + rPerp1y * angularV) * nY;
        jr = jr * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(collidingIDs.get(index)));
        double temp1 = rPerp1x * nX + rPerp1y * nY;
        jr = jr / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);

        double vtrel = ((vX + rPerp1x * angularV) - (getVX(collidingIDs.get(index)) + rPerp2x * getAngularV(collidingIDs.get(index)))) * -nY;
        vtrel += ((vY + rPerp1y * angularV) - (getVY(collidingIDs.get(index)) + rPerp2y * getAngularV(collidingIDs.get(index)))) * nX;
        double friction = getCOEFFICIENT_OF_FRICTION(collidingIDs.get(index)) * jr * -Math.signum(vtrel);
        temp1 = rPerp1x * -nY + rPerp1y * nX;
        double frictionMax = -vtrel / ((1.0 / mass) + (1.0 / inertia) * temp1 * temp1);
        friction = Math.min(Math.abs(frictionMax), Math.abs(friction)) * -Math.signum(vtrel);

        newvX = newvX + (jr / mass) * nX + (friction / mass) * -nY;
        newvY = newvY + (jr / mass) * nY + (friction / mass) * nX;
        if (!lockedRotation) newangularV = newangularV + (jr / inertia) * (rPerp1x * nX + rPerp1y * nY) + (friction / inertia) * (temp1);

    }
    private double[] calculateGravity() {
        double sumaX = 0.0;
        double sumaY = 0.0;
        for (int i = 0; i < num; i = i + 1) {
            if (i != ID && Rigidbody.get(i).simID == simID) {
                double rSquared = (posX - Rigidbody.get(i).getPosX()) * (posX - Rigidbody.get(i).getPosX()) + (posY - Rigidbody.get(i).getPosY()) * (posY - Rigidbody.get(i).getPosY());
                if (rSquared > 0.0) {
                    double r = Math.sqrt(rSquared);
                    double magnitude = (sim.GRAVITATIONAL_CONSTANT * Rigidbody.get(i).getMass()) / (rSquared);
                    sumaX = sumaX + (magnitude / r) * (Rigidbody.get(i).getPosX() - posX);
                    sumaY = sumaY + (magnitude / r) * (Rigidbody.get(i).getPosY() - posY);
                }
            }
        }
        return(new double[]{sumaX, sumaY});
    }
    private void calculateRepulsion() {
        double detectRadiusMultiplier = Math.sqrt(sim.REPULSE_RADIUS_MULTIPLIER);
        double REPULSION_STRENGTH = sim.REPULSION_STRENGTH;
        if (parentSoftbody != -1 && REPULSION_STRENGTH > 0.0) for (int i = 0; i < Softbody.num; i = i + 1) {
            if (Softbody.get(i).simID == simID && i != parentSoftbody) {
                double distanceToBodyX = posX - Softbody.get(i).cM[0];
                double distanceToBodyY = posY - Softbody.get(i).cM[1];
                double distanceToBody = Math.sqrt(distanceToBodyX * distanceToBodyX + distanceToBodyY * distanceToBodyY);
                if (distanceToBody <= detectRadiusMultiplier * (geometry.getLargestDistance() +
                        Softbody.get(i).getRadius())) {
                    for (int j = 0; j < Softbody.get(i).size(); j++) {
                        double dx = posX - Softbody.get(i).getMember(j).getPosX();
                        double dy = posY - Softbody.get(i).getMember(j).getPosY();
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (distance > 0.0) {
                            double aMax = REPULSION_STRENGTH * (((Softbody.get(i).getMember(j).repulseRadius + repulseRadius) / distance) - 1.0);
                            double aMagnitude = Math.max(0.0, aMax);
                            if (aMagnitude > 0.0) {
                                double a = aMagnitude / REPULSION_STRENGTH;
                                double b = 1;
                            }
                            newaX += aMagnitude * (dx / distance);
                            newaY += aMagnitude * (dy / distance);
                        }
                    }
                }
            }
        }
    }
    private void calculateJointForceEffects() {
        double[] sumOfForces = new double[3];
        for (Joint joint : attachments) {
            switch (joint.type) {
                case Spring: {
                    double[] result = joint.calculateSpringForce();
                    sumOfForces[0] += result[0];
                    sumOfForces[1] += result[1];
                    sumOfForces[2] += result[2];
                }
            }
        }
        newaX += sumOfForces[0] / mass;
        newaY += sumOfForces[1] / mass;
        newangularA += sumOfForces[2] / inertia;
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
        multiplier = Math.min(multiplier, sim.MOUSE_SPEED_LIMIT) / multiplier;
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
        if (geometry.pointInside(new double[]{currentMouseX, currentMouseY})) {
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
    private void rotateAroundCenter(double dt) {
        if (!lockedRotation) {
            geometry.rotateAroundCenter(angularV * dt);
        }
    }

    //joint related methods
    public void springAttach(Rigidbody other, double SPRING_STRENGTH, double SPRING_DAMPING) {
        double distance = Math.sqrt((other.posX - posX) * (other.posX - posX) + (other.posY - posY) * (other.posY - posY));
        Joint joint1 = new Joint(this, other, new double[]{0.0, 0.0}, new double[]{0.0, 0.0},
                distance, SPRING_STRENGTH, SPRING_DAMPING, 2.0);
        Joint joint2 = new Joint(other, this, new double[]{0.0, 0.0}, new double[]{0.0, 0.0},
                distance, SPRING_STRENGTH, SPRING_DAMPING, 2.0);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
    }
    public void springAttachSoftbodyConstruction(Rigidbody other) {
        boolean valid = true;
        for (Joint spring : attachments) {
            if (spring.type == JointType.Spring && spring.connection.ID == other.ID) {
                valid = false;
                break;
            }
        }
        if (valid) {
            double distance = Math.sqrt((other.posX - posX) * (other.posX - posX) + (other.posY - posY) * (other.posY - posY));
            Joint joint1 = new Joint(this, other, new double[]{0.0, 0.0}, new double[]{0.0, 0.0},
                    distance, 1.0, 1.0, 2.0);
            Joint joint2 = new Joint(other, this, new double[]{0.0, 0.0}, new double[]{0.0, 0.0},
                    distance, 1.0, 1.0, 2.0);
            attachments.add(joint1);
            other.attachments.add(joint2);
            isAttached = true;
            other.isAttached = true;
        }
    }

    //joint methods related specifically to softbody construction and not used elsewhere
    public void setAllSpringJoints(double HOOKE_SPRING_CONSTANT, double SPRING_DAMPING_COEFFICIENT, double SPRING_MIN_DIST_MULT, double SPRING_MAX_DIST_MULT) {
        for (Joint joint : attachments) {
            if (joint.type == JointType.Spring){
                joint.SPRING_CONSTANT = HOOKE_SPRING_CONSTANT;
                joint.SPRING_DAMPING = SPRING_DAMPING_COEFFICIENT;
                joint.minDistMultiplier = SPRING_MIN_DIST_MULT;
                joint.maxDistMultiplier = SPRING_MAX_DIST_MULT;
            }
        }
    }
    public int getAttachmentNum() {
        if (parentSoftbody == -1) return(0);
        int count = 0;
        if (Softbody.get(parentSoftbody).members.contains(this)) {
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Spring &&
                        Softbody.get(parentSoftbody).members.contains(attachment.connection)) count += 1;
            }
        }
        return(count);
    }
    public Rigidbody getAttachment(int index) {
        if (index >= 0 && index < getAttachmentNum() && Softbody.get(parentSoftbody).members.contains(this)) {
            int count = 0;
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Spring &&
                Softbody.get(parentSoftbody).members.contains(attachment.connection)) {
                    if (count == index) return(attachment.connection);
                    count += 1;
                }
            }
        }
        return(null);
    }
    public int getAttachmentInt(int index) {
        if (index >= 0 && index < getAttachmentNum() && Softbody.get(parentSoftbody).members.contains(this)) {
            int count = 0;
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Spring &&
                        Softbody.get(parentSoftbody).members.contains(attachment.connection)) count += 1;
                if (count == index) return(attachment.connection.ID);
            }
        }
        return(-1);
    }
    public int getBoundaryAttachmentNum() {
        if (parentSoftbody == -1) return(0);
        int count = 0;
        if (Softbody.get(parentSoftbody).boundaryMembers.contains(ID)) {
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Spring &&
                Softbody.get(parentSoftbody).boundaryMembers.contains(attachment.connection.ID)) count += 1;
            }
        }
        return(count);
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
        ArrayList<Rigidbody> generatedPoints = new ArrayList<>();
        if (!futureBoundary) for (int i = 0; i < angleDivisions; i = i + 1) {
            if (inShapePoints[i]) {
                Rigidbody generatedPoint = new Rigidbody(new Circle(Softbody.get(parentSoftbody).getPointRadius()),
                        new double[]{x[i], y[i], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0, Softbody.get(parentSoftbody).getColor(), simID);
                Simulation.get(simID).physicsObjects.add(new PhysicsObject(generatedPoint));
                springAttach(generatedPoint, 1.0, 1.0);
                Softbody.get(parentSoftbody).addMember(generatedPoint, futureBoundary);
                generatedPoints.add(generatedPoint);
            }
        }
        for (Rigidbody generatedPoint : generatedPoints) {
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
        if (inShape || onBoundary) for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (i != ID && Rigidbody.get(i).parentSoftbody == parentSoftbody && Rigidbody.get(i).simID == simID) {
                double distance = (x - Rigidbody.get(i).posX) * (x - Rigidbody.get(i).posX) + (y - Rigidbody.get(i).posY) * (y - Rigidbody.get(i).posY);
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
    public static double mod(double a, double b) {
        if (a >= 0) return(a % b);
        else {
            double tempResult = b - (-a % b);
            if (tempResult == b) return(0);
            else return(tempResult);
        }
    }

    public static Rigidbody get(int index) {
        return(rigidbodies.get(index));
    }

    public Triplet getDraw(double shiftX, double resolutionCenterX, double pixelShiftX,
                           double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling) {
        return(geometry.getDrawInt(shiftX, resolutionCenterX, pixelShiftX, shiftY, resolutionCenterY,
                pixelShiftY, resolutionScaling));
    }
    public void setMovingMotion(double[] movingMotion) {
        vX = movingMotion[0];
        vY = movingMotion[1];
        initialExternalForces[0] = movingMotion[2];
        initialExternalForces[1] = movingMotion[3];
        angularV = movingMotion[4];
        angularA = movingMotion[5];
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
    public double getVX() {
        return(vX);
    }
    public double getVY() {
        return(vY);
    }
    public void setVX(double vX) {
        this.vX = vX;
    }
    public void setVY(double vY) {
        this.vY = vY;
    }
    public double getAX() {
        return(aX);
    }
    public double getAY() {
        return(aY);
    }
    public void setAX(double aX) {
        this.aX = aX;
    }
    public void setAY(double aY) {
        this.aY = aY;
    }
    public void changeAX(double update) {
        newaX += update;
    }
    public void changeAY(double update) {
        newaY += update;
    }
    public double getAngularV() {
        return(angularV);
    }
    public void setAngularV(double angularV) {
        this.angularV = angularV;
    }
    public double getAngularA() {
        return(angularA);
    }
    public void setAngularA(double angularA) {
        this.angularA = angularA;
    }
    public int getID() {
        return(ID);
    }

    public double getMass() {
        return(mass);
    }
    public void setRepulseRadius(double REPULSE_RADIUS_MULTIPLIER) {
        repulseRadius = REPULSE_RADIUS_MULTIPLIER * geometry.getLargestDistance();
    }
    public double getInertia() {
        return(inertia);
    }
    public double getArea() {
        return(area);
    }
    public boolean isMovable() {
        return(isMovable);
    }
    public void setIsMovable(boolean a) {
        isMovable = a;
    }

    private static int convertSoftbodyEdge(int index) {
        return(-index - 2);
    }
    public static double getPosX(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2) { //handle later
            System.out.println("Calling the position of a softbody edge collision should never happen.");
        }
        return(rigidbody.getPosX());
    }
    public static double getPosY(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2) { //handle later
            System.out.println("Calling the position of a softbody edge collision should never happen.");
        }
        return(rigidbody.getPosY());
    }
    public static double getVX(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getVX();
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getVX();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
        return(rigidbody.getVX());
    }
    public static double getVY(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getVY();
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getVY();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
        return(rigidbody.getVY());
    }
    public static double getAX(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getAX();
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getAX();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
        return(rigidbody.getAX());
    }
    public static double getAY(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2 && mod(index, 2) == 1) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getAY();
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getAY();
            return(0.5 * (val1 + val2));
        }
        else return(0.0);
        return(rigidbody.getAY());
    }
    public static double getAngularV(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else return(0.0);
        return(rigidbody.getAngularV());
    }
    public static double getMass(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getMass();
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getMass();
            return(0.5 * (val1 + val2));
        }
        if (rigidbody == null) {
            double a = 1;
        }
        return(rigidbody.getMass());
    }
    public static double getInertia(int index) {
        Rigidbody rigidbody = null;
        if (index >= 0) rigidbody = Rigidbody.get(index);
        else if (index <= -2) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).getInertia();
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).getInertia();
            return(0.5 * (val1 + val2));
        }
        return(rigidbody.getInertia());
    }
    public static Material getMaterial(int index) {
        try {
            Material returnMaterial;
            if (index >= 0) returnMaterial = Simulation.get(Rigidbody.get(index).simID).getObject("Rigidbody", index).material;
            else if (index <= -2) {
                returnMaterial = Simulation.get(Softbody.get(index).simID).getObject("Softbody", index).material;
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
        else if (index <= -2) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).COEFFICIENT_OF_RESTITUTION;
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).COEFFICIENT_OF_RESTITUTION;
            double a = 0.5 * (val1 + val2);
            double b = COEFFICIENT_OF_RESTITUTION;
            if (adoptOnlyOtherSurface) return(a);
            else return(Math.sqrt((a * a + b * b) * 0.5));
        }
        else {
            double a = sim.COEFFICIENT_OF_RESTITUTION;
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
        else if (index <= -2) {
            int softbodyIndex = Rigidbody.get(convertSoftbodyEdge(index)).parentSoftbody;
            int size = Softbody.get(softbodyIndex).boundarySize();
            int edgeIndex = Softbody.get(softbodyIndex).boundaryMembers.indexOf(convertSoftbodyEdge(index));
            double val1 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get(edgeIndex)).COEFFICIENT_OF_FRICTION;
            double val2 = Rigidbody.get(Softbody.get(softbodyIndex).boundaryMembers.get((edgeIndex + 1) % size)).COEFFICIENT_OF_FRICTION;
            double a = 0.5 * (val1 + val2);
            if (adoptOnlyOtherSurface) return(a);
            else return(0.5 * (a + COEFFICIENT_OF_FRICTION));
        }
        else {
            if (adoptOnlyOtherSurface) return(Simulation.get(simID).COEFFICIENT_OF_FRICTION);
            else return(sim.COEFFICIENT_OF_FRICTION + COEFFICIENT_OF_FRICTION * 0.5);
        }
    }
    public static boolean getIsMovable(int index) {
        if (index >= 0) return(Rigidbody.get(index).isMovable());
        else return(false);
    }
}

