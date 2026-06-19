package PhysicsSim;
import PhysicsSim.Math.Complex;
import PhysicsSim.Math.Matrix;

import java.util.ArrayList;
import java.awt.*;
import java.util.Arrays;
import java.util.Objects;

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

    //related to joints
    final ArrayList<Joint> attachments = new ArrayList<>();
    CompoundBody compoundBody = null;
    ConnectedBody connectedBody = null;

    //local physical constants
    double COEFFICIENT_OF_RESTITUTION;
    double COEFFICIENT_OF_FRICTION;

    //generic state information
    private static final ArrayList<Rigidbody> rigidbodies = new ArrayList<>();
    private static final ArrayList<CompoundBody> compoundBodies = new ArrayList<>();
    private static final ArrayList<ConnectedBody> connectedBodies = new ArrayList<>();
    final ArrayList<EventListener> events = new ArrayList<>();
    static int num = 0;
    final int ID;
    boolean isMovable = true;
    boolean lockedRotation = false;
    boolean isHitbox = false;
    boolean isAttached = false;
    boolean draw = true;

    final ArrayList<Controller> controllers = new ArrayList<>();
    final ArrayList<double[]> contactPoints = new ArrayList<>();
    final ArrayList<double[]> MTVs = new ArrayList<>();
    final ArrayList<int[]> collidingIDs = new ArrayList<>();
    //for collidingIDs, 0 - infinity inclusive is reserved for other rigidbodies and
    // -2 - -infinity numbers is reserved for solid joints. These are then converted by n -> -n -2 to get the
    //ID of the rigidbodies involved in that collision.
    //AABBs refer to softbodies as their localID mapped n -> -n - 2, and not joints, though this is unused. Joints are referred to by a non-null solidJoint object.

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
    private double initialExternalTorque;

    double newposX;
    double newposY;
    double newvX;
    double newvY;
    double newaX;
    double newaY;
    double newangularV;
    double newangularA;

    Rigidbody(GeometricType geometry, double[] motion, double mass, Color color, int simID) {
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

        sim = Simulation.get(simID);
        sim.getSAPCell(0, 0).addBox(ID);
        sim.BVHtrees.get(0).addBox(ID);

        COEFFICIENT_OF_RESTITUTION = sim.COEFFICIENT_OF_RESTITUTION;
        COEFFICIENT_OF_FRICTION = sim.COEFFICIENT_OF_FRICTION;
    }

    //general motion
    private void integrateMotion(double dt) {
        newposX += newvX * dt;
        newposY += newvY * dt;
        rotateAroundCenter(dt);
        //we do not use newa here because a is state-based, not cumulative over frames
        newvX += aX * dt;
        newvY += aY * dt;
        newangularV += newangularA * dt;
    }
    static void step(double dt, int simID) {
        for (CompoundBody compoundBody : compoundBodies) {
            if (compoundBody.members.getFirst().simID != simID) continue;
            compoundBody.rotateAroundCenter(compoundBody.members.getFirst().getAngularV() * dt);
        }

        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody != null && rigidbody.simID == simID && !rigidbody.events.isEmpty()) {
                rigidbody.respondToEvent(true);
            }
        }

        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody == null || rigidbody.simID != simID) continue;
            if (rigidbody.lockedRotation) {
                rigidbody.angularV = 0.0;
                rigidbody.angularA = 0.0;
            }
            rigidbody.newposX = rigidbody.posX;
            rigidbody.newposY = rigidbody.posY;
            rigidbody.newvX = rigidbody.vX;
            rigidbody.newvY = rigidbody.vY;
            rigidbody.newangularV = rigidbody.angularV;
        }
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody == null || rigidbody.simID != simID) continue;
            if (rigidbody.isMovable() && !rigidbody.isHitbox) {
                rigidbody.calcMotion(dt);
            }
            else {
                rigidbody.newposX = rigidbody.posX;
                rigidbody.newposY = rigidbody.posY;
                if (!rigidbody.collidingIDs.isEmpty()) {
                    for (int h = 0; h < rigidbody.collidingIDs.size(); h++) {
                        if (rigidbody.collidingIDs.get(h)[0] < -1) {
                            double magnitude = Math.sqrt(rigidbody.MTVs.get(h)[0] * rigidbody.MTVs.get(h)[0] + rigidbody.MTVs.get(h)[1] * rigidbody.MTVs.get(h)[1]);
                            if (Double.isNaN(magnitude)) magnitude = 0.0;
                            double nX = rigidbody.MTVs.get(h)[0] / magnitude;
                            double nY = rigidbody.MTVs.get(h)[1] / magnitude;
                            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                                nX = 0.0;
                                nY = 0.0;
                            }
                            //the normal here is assumed to point towards this rigidbody and away from the other. For all intents and purposes,
                            //this choice should not matter so long as it is treated consistently
                            double rPerp1x = -(rigidbody.contactPoints.get(h)[1] - rigidbody.posY);
                            double rPerp1y = rigidbody.contactPoints.get(h)[0] - rigidbody.posX;
                            rigidbody.calcImpulseSolidJointIfObstacle(rPerp1x, rPerp1y, nX, nY, h);
                        }
                    }
                }
            }
        }
        for (CompoundBody compoundBody : compoundBodies) {
            if (compoundBody.members.getFirst().simID != simID) continue;
            compoundBody.equalizeProperties();
        }
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody == null || rigidbody.simID != simID) continue;
            rigidbody.integrateMotion(dt);
        }
        for (CompoundBody compoundBody : compoundBodies) {
            if (compoundBody.members.getFirst().simID != simID) continue;
            compoundBody.calculateProperties();
            compoundBody.spaceProperly();
        }

        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody != null && rigidbody.simID == simID && !rigidbody.events.isEmpty()) rigidbody.respondToEvent(false);
        }
    }
    static void updateMotion(double dt, int simID) {
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody == null || rigidbody.simID != simID) continue;
            rigidbody.updateMotion(dt);
        }

    }
    private void respondToEvent(boolean isPreUpdateLoop) {
        try {
            if (isHitbox && isPreUpdateLoop) {
                Hitbox parent = sim.getHitbox("Rigidbody", ID);
                for (int i = 0; i < collidingIDs.size(); i++) {
                    PhysicsObjectIntersectionEvent objectEvent1 = null;
                    PhysicsObjectIntersectionEvent objectEvent2 = null;
                    HitboxIntersectionEvent hitboxEvent = null;
                    if (collidingIDs.get(i)[0] >= 0 && !Rigidbody.get(collidingIDs.get(i)[0]).isHitbox) objectEvent1 = new PhysicsObjectIntersectionEvent(sim.getObject("Rigidbody", collidingIDs.get(i)[0]), parent);
                    else if (collidingIDs.get(i)[0] >= 0) hitboxEvent = new HitboxIntersectionEvent(parent, sim.getHitbox("Rigidbody", collidingIDs.get(i)[0]));
                    else if (collidingIDs.get(i)[0] <= -2) {
                        objectEvent1 = new PhysicsObjectIntersectionEvent(sim.getObject("Rigidbody", -collidingIDs.get(i)[0] - 2), parent);
                        objectEvent2 = new PhysicsObjectIntersectionEvent(sim.getObject("Rigidbody", -collidingIDs.get(i)[1] - 2), parent);
                    }
                    else if (collidingIDs.get(i)[0] == -1) hitboxEvent = new HitboxIntersectionEvent(parent, null);

                    if (objectEvent1 != null) for (EventListener event : events) event.intersected(objectEvent1);
                    if (objectEvent2 != null) for (EventListener event : events) event.intersected(objectEvent2);
                    if (hitboxEvent != null) for (EventListener event : events) event.intersected(hitboxEvent);
                }
            }
            else if (!isHitbox) {
                PhysicsObject parent = sim.getObject("Rigidbody", ID);
                for (int i = 0; i < collidingIDs.size(); i++) {
                    double[] normal = new double[]{MTVs.get(i)[0], MTVs.get(i)[1]};
                    double magnitude = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
                    normal[0] /= magnitude;
                    normal[1] /= magnitude;
                    double[] pointOfContact = new double[]{contactPoints.get(i)[0], contactPoints.get(i)[1]};
                    CollisionEvent collisionEvent1 = null;
                    CollisionEvent collisionEvent2 = null;
                    if (collidingIDs.get(i)[0] >= 0 && !Rigidbody.get(collidingIDs.get(i)[0]).isHitbox) {
                        collisionEvent1 = new CollisionEvent(parent, sim.getObject("Rigidbody", collidingIDs.get(i)[0]), normal, pointOfContact, false);
                    } else if (collidingIDs.get(i)[0] <= -2) {
                        collisionEvent1 = new CollisionEvent(parent, sim.getObject("Rigidbody", -collidingIDs.get(i)[0] - 2), normal, pointOfContact, true);
                        collisionEvent2 = new CollisionEvent(parent, sim.getObject("Rigidbody", -collidingIDs.get(i)[1] - 2), normal, pointOfContact, true);
                    } else if (collidingIDs.get(i)[0] == -1) {
                        collisionEvent1 = new CollisionEvent(parent, null, normal, pointOfContact, false);
                    }

                    if (collisionEvent1 != null) for (EventListener event : events) {
                        if (isPreUpdateLoop && collisionEvent2 == null) event.collidedPre(collisionEvent1);
                        else if (isPreUpdateLoop) {
                            event.collidedPre(collisionEvent1);
                            event.collidedPre(collisionEvent2);
                        }

                        //null for the second event indicates joint
                        if (!isPreUpdateLoop && collisionEvent2 == null) event.collidedPost(collisionEvent1);
                        else if (!isPreUpdateLoop) {
                            event.collidedPost(collisionEvent1);
                            event.collidedPost(collisionEvent2);
                        }
                    }
                }
            }
        }
        catch (Exception _) {

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
        if (Double.isFinite(newposX)) posX = newposX;
        if (Double.isFinite(newposY)) posY = newposY;
        if (Double.isFinite(newvX)) vX = newvX;
        if (Double.isFinite(newvY)) vY = newvY;
        if (Double.isFinite(newaX)) aX = newaX;
        if (Double.isFinite(newaY)) aY = newaY;
        if (Double.isFinite(newangularV)) angularV = newangularV;
        if (Double.isFinite(newangularA)) angularA = newangularA;

        applyControllers(dt);
    }
    private void applyControllers(double dt) {
        if (!controllers.isEmpty()) {
            //take into account controllers first once 'onGround' is known
            boolean onGround = false;
            double[] maxGroundVelocity = new double[]{0.0, 0.0};
            double maxGroundVelocityMagnitude = 0.0;
            for (int i = 0; i < collidingIDs.size(); i = i + 1) {
                if (Rigidbody.getMaterial(collidingIDs.get(i)[0]).name.contains("Ground")) {
                    if (!Double.isFinite(contactPoints.get(i)[0]) || !Double.isFinite(contactPoints.get(i)[1])) continue;
                    if (contactPoints.get(i)[0] * initialExternalForces[0] + contactPoints.get(i)[1] * initialExternalForces[1]
                            > posX * initialExternalForces[0] + posY * initialExternalForces[1] + 0.5 * geometry.getLargestDistance()) {
                        onGround = true;
                        double gvX = 0.0;
                        double gvY = 0.0;
                        if (collidingIDs.get(i)[0] >= 0) {
                            Rigidbody rigidbody = Rigidbody.get(collidingIDs.get(i)[0]);
                            gvX = rigidbody.getVX() + rigidbody.getAngularV() * -(contactPoints.get(i)[1] - rigidbody.getPosY());
                            gvY = rigidbody.getVY() + rigidbody.getAngularV() * (contactPoints.get(i)[0] - rigidbody.getPosX());
                        }
                        else if (collidingIDs.get(i)[0] <= -2) {
                            Rigidbody joint1 = Rigidbody.get(-collidingIDs.get(i)[0] - 2);
                            Rigidbody joint2 = Rigidbody.get(-collidingIDs.get(i)[1] - 2);

                            //these go from the center of joint 1 (point A body) to point A and from joint 2 to point B
                            //these values will be made perpendicular after the linear interpolation value t is calculated.
                            double[] r2APerp = new double[]{0.0,0.0};
                            double[] r2BPerp = new double[]{0.0,0.0};
                            Joint attachment = joint1.attachments.get(collidingIDs.get(i)[2]);
                            r2APerp[0] = attachment.offsetFromCMParent[0];
                            r2APerp[1] = attachment.offsetFromCMParent[1];
                            r2BPerp[0] = attachment.offsetFromCMOther[0];
                            r2BPerp[1] = attachment.offsetFromCMOther[1];

                            double temp1 = (contactPoints.get(i)[0] - joint1.getPosX() - r2APerp[0]) * (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0])
                                    + (contactPoints.get(i)[1] - joint1.getPosY() - r2APerp[1]) * (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]);
                            double lineMagnitude = (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0]) * (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0])
                                    + (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]) * (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]);
                            double t = temp1 / lineMagnitude;
                            temp1 = r2APerp[0];
                            r2APerp[0] = -r2APerp[1];
                            r2APerp[1] = temp1;
                            temp1 = r2BPerp[0];
                            r2BPerp[0] = -r2BPerp[1];
                            r2BPerp[1] = temp1;

                            temp1 = (joint1.getVX() + joint1.getAngularV() * r2APerp[0]);
                            double temp2 = (joint2.getVX() + joint2.getAngularV() * r2BPerp[0]);
                            gvX = (temp2 - temp1) * t + temp1;

                            temp1 = (joint1.getVY() + joint1.getAngularV() * r2APerp[1]);
                            temp2 = (joint2.getVY() + joint2.getAngularV() * r2BPerp[1]);
                            gvY = (temp2 - temp1) * t + temp1;
                        }
                        double magnitude = Math.sqrt(gvX * gvX + gvY * gvY);
                        if (magnitude > maxGroundVelocityMagnitude) {
                            maxGroundVelocityMagnitude = magnitude;
                            maxGroundVelocity[0] = gvX;
                            maxGroundVelocity[1] = gvY;
                        }
                    }
                }
            }
            boolean switchFirstPress = false;
            for (Controller controller : controllers) {
                boolean makeTheSwitch = controller.respondToKeyImpulse(dt, sim.display.keysCache, sim.display.firstPress,
                        sim.display.keyReleasedFirstTime, onGround, !collidingIDs.isEmpty(), maxGroundVelocity);
                if (makeTheSwitch) switchFirstPress = true;
            }
            if (switchFirstPress) sim.display.firstPress = false;
        }
    }
    static void clearCollisionInformation(int simID) {
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody == null || rigidbody.simID != simID) continue;
            rigidbody.contactPoints.clear();
            rigidbody.MTVs.clear();
            rigidbody.collidingIDs.clear();
        }
    }
    public boolean checkCollisions(Rigidbody otherObject) {
        if (isHitbox || !otherObject.isHitbox) return(geometry.findCollisions(otherObject.geometry));
        return false;
    }
    public boolean checkCollisions(Joint solidJoint) {
        return(geometry.findCollisions(solidJoint));
    }
    private boolean checkForCollisionsWall() {
        return(geometry.checkForCollisionsWall());
    }
    static void finalizeCollisionInformation(int simID) {
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody == null || rigidbody.simID != simID) continue;
            if (rigidbody.sim.bounds && rigidbody.isMovable()) rigidbody.checkForCollisionsWall();
            double[] deleteCollisionMarker = new double[]{Double.NaN, Double.NaN};
            int[] deleteCollisionMarkerFace = new int[]{-10};
            ArrayList<Integer> pinJointBodyIDs = new ArrayList<>();
            for (Joint joint : rigidbody.attachments) {
                if (!joint.collidesWithSelfConnections()) pinJointBodyIDs.add(joint.connection.ID);
            }

            //prune the list for points too close to one another (same point, but different polygon with floating point precision differences or duplicates)
            //at the same time, remove collisions close enough to the translational joint attachment to be considered
            //at the attachment point.
            if (!rigidbody.contactPoints.isEmpty()) for (int i = 0; i < rigidbody.contactPoints.size(); i = i + 1) {
                if (rigidbody.isAttached && pinJointBodyIDs.contains(rigidbody.collidingIDs.get(i)[0])) {
                    rigidbody.contactPoints.set(i, deleteCollisionMarker);
                    continue;
                }
                for (int j = i + 1; j < rigidbody.contactPoints.size(); j = j + 1) {
                    if (i != j) {
                        if (rigidbody.isHitbox && rigidbody.collidingIDs.get(i)[0] == rigidbody.collidingIDs.get(j)[0]) {
                            rigidbody.contactPoints.set(j, deleteCollisionMarker);
                            continue;
                        }
                        double dx = rigidbody.contactPoints.get(i)[0] - rigidbody.contactPoints.get(j)[0];
                        double dy = rigidbody.contactPoints.get(i)[1] - rigidbody.contactPoints.get(j)[1];
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (distance <= rigidbody.sim.CONTACT_POINTS_MERGE_DISTANCE) {
                            rigidbody.contactPoints.set(j, deleteCollisionMarker);
                        }
                    }
                }

                //remove collisions too close to translational joint point of connection
                if (rigidbody.isAttached) for (Joint translational : rigidbody.attachments) {
                    if (translational.type != JointType.Translational || rigidbody.collidingIDs.get(i)[0] != translational.connection.ID) continue;
                    double dx = 0.0;
                    double dy = 0.0;
                    if (translational.isTranslationalParent) {
                        dx = rigidbody.contactPoints.get(i)[0] - (translational.connection.getPosX() + translational.offsetFromCMOther[0]);
                        dy = rigidbody.contactPoints.get(i)[1] - (translational.connection.getPosY() + translational.offsetFromCMOther[1]);
                    }
                    else {
                        dx = rigidbody.contactPoints.get(i)[0] - (rigidbody.getPosX() + translational.offsetFromCMParent[0]);
                        dy = rigidbody.contactPoints.get(i)[1] - (rigidbody.getPosY() + translational.offsetFromCMParent[1]);
                    }
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (distance <= rigidbody.sim.CONTACT_POINTS_MERGE_DISTANCE) {
                        rigidbody.contactPoints.set(i, deleteCollisionMarker);
                    }
                }
            }

            //iterate through collisions and remove "rememberedFacesToPass" for stability with faces
            for (int i = 0; i < rigidbody.geometry.rememberedFacesToPass.size(); i++) {
                boolean foundMatch = false;
                for (int j = 0; j < rigidbody.collidingIDs.size(); j++) {
                    if (Arrays.equals(rigidbody.geometry.rememberedFacesToPass.get(i), rigidbody.collidingIDs.get(j))) {
                        rigidbody.contactPoints.set(j, deleteCollisionMarker);
                        foundMatch = true;
                    }
                }
                if (!foundMatch) rigidbody.geometry.rememberedFacesToPass.set(i, deleteCollisionMarkerFace);
            }
            int length = rigidbody.geometry.rememberedFacesToPass.size();
            for (int i = 0; i < length; i = i + 1) {
                if (i >= rigidbody.geometry.rememberedFacesToPass.size()) break;
                if (rigidbody.geometry.rememberedFacesToPass.get(i) == deleteCollisionMarkerFace) {
                    rigidbody.geometry.rememberedFacesToPass.remove(i);
                    i--;
                }
            }


            length = rigidbody.contactPoints.size();
            for (int i = 0; i < length; i = i + 1) {
                if (i >= rigidbody.contactPoints.size()) break;
                if (rigidbody.contactPoints.get(i) == deleteCollisionMarker || !Double.isFinite(rigidbody.contactPoints.get(i)[0]) || !Double.isFinite(rigidbody.contactPoints.get(i)[1])) {
                    rigidbody.contactPoints.remove(i);
                    rigidbody.MTVs.remove(i);
                    rigidbody.collidingIDs.remove(i);
                    i = i - 1;
                }
            }
        }
    }
    private void calcMotion(double dt) {
        //check for collisions and do one of two options for updating motion based on whether the rigidbody is colliding with another
        boolean intersecting = !collidingIDs.isEmpty();
        int countOfValidShifts = 0;

        //each triplet here is formatted as double[] -> rPerp1c, double[] -> nc, double -> change in v/w. If either double[] is null, then the constraint is expected to be angular, otherwise it is expected to be linear.
        ArrayList<Triplet> constraintInfo = new ArrayList<>();
        //each triplet here is formatted as double[] -> rPerp1a, double[] -> rPerp2a, double[] -> na. If the 1st and 3rd double[] are null, then the application is expected to be angular, otherwise it is expected to be linear.
        //rPerp2a is a bit of a misnomer. If involving a another body it is {rPerp2xa, rPerp2ya, M2^-1, I2^-1}; if a wall it is null; if a solid joint it is {rPerpAxa, rPerpAya, MA^-1, IA^-1, rPerpBxa, rPerpBya, MB^-1, IB^-1, t}.
        //this may also contain the joint1 and joint2 if a solid joint.
        //TO DO: rename Triplet to InformationPacket and refactor its internals to save on memory
        ArrayList<Triplet> applicationInfo = new ArrayList<>();
        //each triplet here represents any friction constraints added, formatted as int -> application index, double -> coefficient of friction, double -> vtrel
        ArrayList<Triplet> frictionInfo = new ArrayList<>();
        //this stores the collidingIDs plus the requisite info for other constraints
        ArrayList<int[]> bodyInfo = new ArrayList<>();
        if (intersecting) for (int h = 0; h < collidingIDs.size(); h = h + 1) {
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
            //the normal here is assumed to point towards this rigidbody and away from the other. For all intents and purposes,
            //this choice should not matter so long as it is treated consistently
            double rPerp1x = -(contactPoints.get(h)[1] - posY);
            double rPerp1y = contactPoints.get(h)[0] - posX;

            //take one of three approaches in calculations: the first if the bounds, the second if another rigidbody, and the third if a solid joint
            if (collidingIDs.get(h)[0] == -1) countOfValidShifts += calcImpulseWall(rPerp1x, rPerp1y, nX, nY, constraintInfo, applicationInfo, frictionInfo, bodyInfo);
            else if (collidingIDs.get(h)[0] >= 0) countOfValidShifts += calcImpulseRigidbody(rPerp1x, rPerp1y, nX, nY, h, constraintInfo, applicationInfo, frictionInfo, bodyInfo);
            else if (collidingIDs.get(h)[0] <= -2) countOfValidShifts += calcImpulseSolidJoint(rPerp1x, rPerp1y, nX, nY, h, constraintInfo, applicationInfo, frictionInfo, bodyInfo);
        }

        newaX = initialExternalForces[0];
        newaY = initialExternalForces[1];
        if (!lockedRotation) newangularA = initialExternalTorque;

        //next handle joints, starting with mouse joint (though not technically a joint)
        if (mouseHold) {
            double vmX = (currentMouseX - lastMouseX) / dt;
            double vmY = (currentMouseY - lastMouseY) / dt;
            double magnitude = Math.sqrt(vmX * vmX + vmY * vmY);
            if (magnitude > sim.MOUSE_SPEED_LIMIT) {
                vmX *= sim.MOUSE_SPEED_LIMIT / magnitude;
                vmY *= sim.MOUSE_SPEED_LIMIT / magnitude;
            }
            double[] rPerp = new double[]{-currentMouseY + posY, currentMouseX - posX};
            double vxRelChange = -(vX + rPerp[0] * angularV - vmX);
            double vyRelChange = -(vY + rPerp[1] * angularV - vmY);
            constraintInfo.add(new Triplet(rPerp, new double[]{1.0, 0.0}, vxRelChange));
            applicationInfo.add(new Triplet(rPerp, null, new double[]{1.0, 0.0}));
            bodyInfo.add(new int[]{-1});
            constraintInfo.add(new Triplet(rPerp, new double[]{0.0, 1.0}, vyRelChange));
            applicationInfo.add(new Triplet(rPerp, null, new double[]{0.0, 1.0}));
            bodyInfo.add(new int[]{-1});
        }
        if (mouseRelease) {
            newvX += flingX;
            newvY += flingY;
            mouseRelease = false;
        }
        countOfValidShifts += calculateJointEffects(constraintInfo, applicationInfo, bodyInfo);

        //MATRIX
        Matrix impulses = assembleImpulseMatrix(constraintInfo, applicationInfo, bodyInfo);
        //the margin here is 2^-32.
        if (impulses != null) {
            Matrix original = impulses.clone();
            impulses.rref(2.3283064365386963E-10);
            if (!isMatrixSolved(impulses)) {
                original = Matrix.add(original, Matrix.multiply(original.getIdentity(), Complex.valueOf(0.001)));
                original.rref(2.3283064365386963E-10);
                if (isMatrixSolved(original)) {
                    if (Simulation.showCreationInfoErrorMsgs) System.out.println("Diagonal constraint matrix stabilization successful.");
                    impulses = original;
                }
                else {
                    if (Simulation.showCreationInfoErrorMsgs) System.out.println("Soft diagonal addition stabilization ineffective to render constraint matrix solvable.");
                }
            }

            //now apply the impulses
            for (int i = 0; i < applicationInfo.size(); i++) {
                if (applicationInfo.get(i).getFirstDoubleArray() != null && applicationInfo.get(i).getThirdDoubleArray() != null) {
                    //begin linear impulse application
                    if (impulses.get(i, i).getReal() != 1.0) continue;
                    double[] rPerp = applicationInfo.get(i).getFirstDoubleArray();
                    double[] n = applicationInfo.get(i).getThirdDoubleArray();
                    double j = impulses.get(i, impulses.getNumColumns() - 1).getReal();
                    newvX += (isMovable() ? 1.0 / mass : 0.0) * n[0] * j;
                    newvY += (isMovable() ? 1.0 / mass : 0.0) * n[1] * j;
                    newangularV += (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * (n[0] * rPerp[0] + n[1] * rPerp[1]) * j;

                    //if a solid joint application, we have to manually add the impulse to the joint's parents' velocities
                    if (applicationInfo.get(i).getSecondDoubleArray() != null && applicationInfo.get(i).getSecondDoubleArray().length == 9) {
                        double t = applicationInfo.get(i).getSecondDoubleArray()[8];
                        Rigidbody joint1 = applicationInfo.get(i).getJoint1();
                        Rigidbody joint2 = applicationInfo.get(i).getJoint2();
                        if (joint1.isMovable()) {
                            joint1.newvX += -j * (1.0 - t) * applicationInfo.get(i).getSecondDoubleArray()[2] * n[0];
                            joint1.newvY += -j * (1.0 - t) * applicationInfo.get(i).getSecondDoubleArray()[2] * n[1];
                            if (!joint1.lockedRotation) {
                                joint1.newangularV += -j * (1.0 - t) * applicationInfo.get(i).getSecondDoubleArray()[2] * (applicationInfo.get(i).getSecondDoubleArray()[0] * n[0] + applicationInfo.get(i).getSecondDoubleArray()[1] * n[1]);
                            }
                        }
                        if (joint2.isMovable()) {
                            joint2.newvX += -j * t * applicationInfo.get(i).getSecondDoubleArray()[6] * n[0];
                            joint2.newvY += -j * t * applicationInfo.get(i).getSecondDoubleArray()[6] * n[1];
                            if (!joint2.lockedRotation) {
                                joint2.newangularV += -j * t * applicationInfo.get(i).getSecondDoubleArray()[7] * (applicationInfo.get(i).getSecondDoubleArray()[4] * n[0] + applicationInfo.get(i).getSecondDoubleArray()[5] * n[1]);
                            }
                        }
                    }
                }
                else {
                    //begin angular impulse application
                    double tau = impulses.get(i, impulses.getNumColumns() - 1).getReal();
                    newangularV += (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * tau;
                }
            }
            //now apply frictional impulses based on the new velocities (which are only changed due to constraints so far).
            //For static calculations purposes, friction assumes it is the only impulse acting on the other body (though this is only sometimes true).
            double changeVX = newvX - vX;
            double changeVY = newvY - vY;
            double changeAngularV = newangularV - angularV;
            for (Triplet friction : frictionInfo) {
                Triplet application = applicationInfo.get(friction.getFirstInt());
                double[] n = application.getThirdDoubleArray();
                double[] rPerp1 = application.getFirstDoubleArray();
                double temp = rPerp1[0] * -n[1] + rPerp1[1] * n[0];
                double meff = (isMovable() ? 1.0 / mass : 0.0) + (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * temp * temp;
                meff += findOtherEffectiveMassFriction(application);
                double staticFriction = -(friction.getThirdDouble() + (changeVX + changeAngularV * rPerp1[0]) * -n[1] + (changeVY + changeAngularV * rPerp1[1]) * n[0]) / meff;
                double dynamicFriction = Math.abs(friction.getSecondDouble() * impulses.get(friction.getFirstInt(), impulses.getNumColumns() - 1).getReal()) * Math.signum(staticFriction);
                double j = dynamicFriction;
                if (Math.abs(staticFriction) <= Math.abs(dynamicFriction)) {
                    j = staticFriction;
                }
                newvX += (isMovable() ? 1.0 / mass : 0.0) * -n[1] * j;
                newvY += (isMovable() ? 1.0 / mass : 0.0) * n[0] * j;
                newangularV += (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * (-n[1] * rPerp1[0] + n[0] * rPerp1[1]) * j;

                //solid joint collisions considered
                if (application.getSecondDoubleArray() != null && application.getSecondDoubleArray().length == 9) {
                    double t = application.getSecondDoubleArray()[8];
                    Rigidbody joint1 = application.getJoint1();
                    Rigidbody joint2 = application.getJoint2();
                    if (joint1.isMovable()) {
                        joint1.newvX += -j * (1.0 - t) * application.getSecondDoubleArray()[2] * -n[1];
                        joint1.newvY += -j * (1.0 - t) * application.getSecondDoubleArray()[2] * n[0];
                        if (!joint1.lockedRotation) {
                            joint1.newangularV += -j * (1.0 - t) * application.getSecondDoubleArray()[2] * (application.getSecondDoubleArray()[0] * -n[1] + application.getSecondDoubleArray()[1] * n[0]);
                        }
                    }
                    if (joint2.isMovable()) {
                        joint2.newvX += -j * t * application.getSecondDoubleArray()[6] * -n[1];
                        joint2.newvY += -j * t * application.getSecondDoubleArray()[6] * n[0];
                        if (!joint2.lockedRotation) {
                            joint2.newangularV += -j * t * application.getSecondDoubleArray()[7] * (application.getSecondDoubleArray()[4] * -n[1] + application.getSecondDoubleArray()[5] * n[0]);
                        }
                    }
                }
            }
        }

        if (sim.universalGravity) {
            double[] results = calculateGravity();
            newaX += results[0];
            newaY += results[1];
        }
        if (sim.airResistance) {
            double[] results = geometry.calculateAirResistance(sim.AIR_DENSITY, sim.DRAG_COEFFICIENT, sim.WIND_SPEED, dt);
            newaX += results[0] / mass;
            newaY += results[1] / mass;
            newangularA += results[2] / inertia;
        }

        calculateRepulsion();
        if (!controllers.isEmpty()) for (Controller controller : controllers) {
            double[] results = controller.respondToKeyJerk(sim.display.keysCache);
            if (compoundBody != null) {
                compoundBody.addControllerJerk(results);
            }
            else {
                newaX += results[0];
                newaY += results[1];
            }
        }

        if (countOfValidShifts > 0) {
            //this way, we equally weigh every MTV shift, not resulting in accumulating massive impulses
            if (compoundBody != null) compoundBody.changePosition((newposX - posX) / countOfValidShifts,  (newposY - posY) / countOfValidShifts);
            else {
                newposX = posX + (newposX - posX) / countOfValidShifts;
                newposY = posY + (newposY - posY) / countOfValidShifts;
            }
        }
    }

    private boolean isMatrixSolved(Matrix M) {
        int n = Math.min(M.getNumRows(), M.getNumColumns());
        boolean solved = true;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if ((i != j && !M.get(i, j).equals(Complex.valueOf(0.0))) || (i == j && !M.get(i, j).equals(Complex.valueOf(1.0)))) {
                    solved = false;
                    break;
                }
            }
        }
        return solved;
    }

    private Matrix assembleImpulseMatrix(ArrayList<Triplet> constraintInfo, ArrayList<Triplet> applicationInfo, ArrayList<int[]> bodyInfo) {
        Matrix result;
        if (!constraintInfo.isEmpty() && !applicationInfo.isEmpty()) result = new Matrix(constraintInfo.size(), applicationInfo.size() + 1);
        else return null;
        //the only application-constraint case we have so far is linear-linear
        for (int c = 0; c < constraintInfo.size(); c++) {
            Triplet constraint = constraintInfo.get(c);
            result.set(c, applicationInfo.size(), Complex.valueOf(constraint.getThirdDouble()));
            if (constraint.getFirstDoubleArray() != null && constraint.getSecondDoubleArray() != null) for (int a = 0; a < applicationInfo.size(); a++) {
                //the constraint is found to be linear
                Triplet application = applicationInfo.get(a);
                double coefficient = 0.0;
                if (application.getFirstDoubleArray() != null && application.getThirdDoubleArray() != null) {
                    //application is found to be linear (linear-linear case)
                    coefficient = (isMovable() ? 1.0 / mass : 0.0) * (application.getThirdDoubleArray()[0] * constraint.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[1] * constraint.getSecondDoubleArray()[1]);
                    coefficient += (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * (application.getThirdDoubleArray()[0] * application.getFirstDoubleArray()[0] + application.getThirdDoubleArray()[1] * application.getFirstDoubleArray()[1])
                            * (constraint.getSecondDoubleArray()[0] * constraint.getFirstDoubleArray()[0] + constraint.getSecondDoubleArray()[1] * constraint.getFirstDoubleArray()[1]);

                    //handle the three special cases when application is applied to a body that a constraint already acts on
                    if (bodyInfo.get(a)[0] != -1 && bodyInfo.get(c)[0] != -1 && Arrays.equals(bodyInfo.get(a), bodyInfo.get(c))) {
                        coefficient += findOtherEffectiveMassRegular(constraintInfo, c, applicationInfo, a);
                    }
                }
                else {
                    //application is found to be angular (angular-linear case)
                    coefficient = (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * (constraint.getSecondDoubleArray()[0] * constraint.getFirstDoubleArray()[0] + constraint.getSecondDoubleArray()[1] * constraint.getFirstDoubleArray()[1]);

                    //angular impulses only apply between bodies so far, so only the body special case needs to be considered
                    if (bodyInfo.get(a)[0] != -1 && bodyInfo.get(c)[0] != -1 && Arrays.equals(bodyInfo.get(a), bodyInfo.get(c))) {
                        coefficient += application.getSecondDoubleArray()[3] * (constraint.getSecondDoubleArray()[0] * applicationInfo.get(c).getSecondDoubleArray()[0] + constraint.getSecondDoubleArray()[1] * applicationInfo.get(c).getSecondDoubleArray()[1]);
                    }
                }
                result.set(c, a, Complex.valueOf(coefficient));
            }
            else for (int a = 0; a < applicationInfo.size(); a++) {
                //the constraint is found to be angular
                Triplet application = applicationInfo.get(a);
                double coefficient = 0.0;
                if (application.getFirstDoubleArray() != null && application.getThirdDoubleArray() != null) {
                    //application is found to be linear (linear-angular case)
                    coefficient = (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0) * (application.getThirdDoubleArray()[0] * application.getFirstDoubleArray()[0] + application.getThirdDoubleArray()[1] * application.getFirstDoubleArray()[1]);

                    //angular impulses only apply between bodies so far, so only the body special case needs to be considered
                    if (bodyInfo.get(a)[0] != -1 && bodyInfo.get(c)[0] != -1 && Arrays.equals(bodyInfo.get(a), bodyInfo.get(c))) {
                        coefficient += application.getSecondDoubleArray()[3] * (application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[1]);
                    }
                }
                else {
                    //application is found to be angular (angular-angular case)
                    coefficient = (isMovable() && !lockedRotation() ? 1.0 / inertia : 0.0);

                    //angular impulses only apply between bodies so far, so only the body special case needs to be considered
                    if (bodyInfo.get(a)[0] != -1 && bodyInfo.get(c)[0] != -1 && Arrays.equals(bodyInfo.get(a), bodyInfo.get(c))) {
                        coefficient += application.getSecondDoubleArray()[3];
                    }
                }
                result.set(c, a, Complex.valueOf(coefficient));
            }
        }

        return result;
    }
    private double findOtherEffectiveMassRegular(ArrayList<Triplet> constraintInfo, int c, ArrayList<Triplet> applicationInfo, int a) {
        double coefficient = 0.0;
        Triplet constraint = constraintInfo.get(c);
        Triplet application = applicationInfo.get(a);
        if (application.getSecondDoubleArray() != null && application.getSecondDoubleArray().length == 4) {
            //this is the most common body case
            coefficient += application.getSecondDoubleArray()[2] * (application.getThirdDoubleArray()[0] * constraint.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[1] * constraint.getSecondDoubleArray()[1]);
            coefficient += application.getSecondDoubleArray()[3] * (application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[1])
                    * (constraint.getSecondDoubleArray()[0] * applicationInfo.get(c).getSecondDoubleArray()[0] + constraint.getSecondDoubleArray()[1] * applicationInfo.get(c).getSecondDoubleArray()[1]);
        }
        else if (application.getSecondDoubleArray() != null && application.getSecondDoubleArray().length == 9) {
            //figure out the effect at point A and B and then use t to correctly interpolate between them.
            double A = application.getSecondDoubleArray()[2];
            A += application.getSecondDoubleArray()[3] * (application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[1])
                    * (constraint.getSecondDoubleArray()[0] * applicationInfo.get(c).getSecondDoubleArray()[0] + constraint.getSecondDoubleArray()[1] * applicationInfo.get(c).getSecondDoubleArray()[1]);
            double B = application.getSecondDoubleArray()[6];
            B += application.getSecondDoubleArray()[7] * (application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[4] + application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[5])
                    * (constraint.getSecondDoubleArray()[0] * applicationInfo.get(c).getSecondDoubleArray()[4] + constraint.getSecondDoubleArray()[1] * applicationInfo.get(c).getSecondDoubleArray()[5]);
            coefficient += (B - A) * application.getSecondDoubleArray()[8] + A;
        }
        return coefficient;
    }
    private double findOtherEffectiveMassFriction(Triplet application) {
        double coefficient = 0.0;
        if (application.getSecondDoubleArray() != null && application.getSecondDoubleArray().length == 4) {
            //this is the most common body case
            coefficient = (-application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[1]);
            coefficient = coefficient * coefficient * application.getSecondDoubleArray()[3] + application.getSecondDoubleArray()[2];
        }
        else if (application.getSecondDoubleArray() != null && application.getSecondDoubleArray().length == 9) {
            //figure out the effect at point A and B and then use t to correctly interpolate between them.
            double A = (-application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[0] + application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[1]);
            A *= A * application.getSecondDoubleArray()[3];
            A += application.getSecondDoubleArray()[2];
            double B = (-application.getThirdDoubleArray()[1] * application.getSecondDoubleArray()[4] + application.getThirdDoubleArray()[0] * application.getSecondDoubleArray()[5]);
            B *= B * application.getSecondDoubleArray()[7];
            B += application.getSecondDoubleArray()[6];
            coefficient += (B - A) * application.getSecondDoubleArray()[8] + A;
        }
        return coefficient;
    }

    private int calcImpulseWall(double rPerp1x, double rPerp1y, double nX, double nY, ArrayList<Triplet> constraintInfo, ArrayList<Triplet> applicationInfo, ArrayList<Triplet> frictionInfo, ArrayList<int[]> bodyInfo) {
        double vimprel = (vX + rPerp1x * angularV) * nX + (vY + rPerp1y * angularV) * nY;
        if (vimprel > 0.0) {
            return 0;
        }

        vimprel = vimprel * -(1.0 + getCOEFFICIENT_OF_RESTITUTION());
        double vtrel = (vX + rPerp1x * angularV) * -nY + (vY + rPerp1y * angularV) * nX;

        double mu = getCOEFFICIENT_OF_FRICTION();

        //MATRIX
        constraintInfo.add(new Triplet(new double[]{rPerp1x, rPerp1y}, new double[]{nX, nY}, vimprel));
        applicationInfo.add(new Triplet(new double[]{rPerp1x, rPerp1y}, null, new double[]{nX, nY}));
        if (mu != 0.0) {
            frictionInfo.add(new Triplet(applicationInfo.size() - 1, mu, vtrel));
        }
        bodyInfo.add(new int[]{-1});
        return 1;
    }
    private int calcImpulseRigidbody(double rPerp1x, double rPerp1y, double nX, double nY, int h, ArrayList<Triplet> constraintInfo, ArrayList<Triplet> applicationInfo, ArrayList<Triplet> frictionInfo, ArrayList<int[]> bodyInfo) {
        Rigidbody other = Rigidbody.get(collidingIDs.get(h)[0]);
        double rPerp2x = -(contactPoints.get(h)[3] - other.getPosY());
        double rPerp2y = contactPoints.get(h)[2] - other.getPosX();

        double vimprel = ((vX + rPerp1x * angularV) - (other.getVX() + rPerp2x * other.getAngularV())) * nX + ((vY + rPerp1y * angularV) - (other.getVY() + rPerp2y * other.getAngularV())) * nY;
        if (vimprel > 0.0) {
            return 0;
        }

        vimprel = vimprel * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(other));
        double vtrel = ((vX + rPerp1x * angularV) - (other.getVX() + rPerp2x * other.getAngularV())) * -nY + ((vY + rPerp1y * angularV) - (other.getVY() + rPerp2y * other.getAngularV())) * nX;

        double mu = getCOEFFICIENT_OF_FRICTION(other);
        double inverseMass = other.isMovable() ? 1.0 / other.getMass() : 0.0;
        double inverseInertia = other.isMovable() && !other.lockedRotation ? 1.0 / other.getInertia() : 0.0;

        //MATRIX
        constraintInfo.add(new Triplet(new double[]{rPerp1x, rPerp1y}, new double[]{nX, nY}, vimprel));
        applicationInfo.add(new Triplet(new double[]{rPerp1x, rPerp1y}, new double[]{rPerp2x, rPerp2y, inverseMass, inverseInertia}, new double[]{nX, nY}));
        if (mu != 0.0) {
            frictionInfo.add(new Triplet(applicationInfo.size() - 1, mu, vtrel));
        }
        bodyInfo.add(new int[]{collidingIDs.get(h)[0]});
        return 1;
    }
    private int calcImpulseSolidJoint(double rPerp1x, double rPerp1y, double nX, double nY, int h, ArrayList<Triplet> constraintInfo, ArrayList<Triplet> applicationInfo, ArrayList<Triplet> frictionInfo, ArrayList<int[]> bodyInfo) {
        Rigidbody joint1 = Rigidbody.get(-collidingIDs.get(h)[0] - 2);
        Rigidbody joint2 = Rigidbody.get(-collidingIDs.get(h)[1] - 2);

        //these go from the center of joint 1 (point A body) to point A and from joint 2 to point B
        //these values will be made perpendicular after the linear interpolation value t is calculated.
        double[] r2APerp = new double[]{0.0,0.0};
        double[] r2BPerp = new double[]{0.0,0.0};
        Joint attachment = joint1.attachments.get(collidingIDs.get(h)[2]);
        r2APerp[0] = attachment.offsetFromCMParent[0];
        r2APerp[1] = attachment.offsetFromCMParent[1];
        r2BPerp[0] = attachment.offsetFromCMOther[0];
        r2BPerp[1] = attachment.offsetFromCMOther[1];

        double temp1 = (contactPoints.get(h)[0] - joint1.getPosX() - r2APerp[0]) * (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0])
                + (contactPoints.get(h)[1] - joint1.getPosY() - r2APerp[1]) * (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]);
        double lineMagnitude = (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0]) * (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0])
                + (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]) * (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]);
        double t = temp1 / lineMagnitude;
        temp1 = r2APerp[0];
        r2APerp[0] = -r2APerp[1];
        r2APerp[1] = temp1;
        temp1 = r2BPerp[0];
        r2BPerp[0] = -r2BPerp[1];
        r2BPerp[1] = temp1;

        temp1 = (joint1.getVX() + joint1.getAngularV() * r2APerp[0]) * nX + (joint1.getVY() + joint1.getAngularV() * r2APerp[1]) * nY;
        double temp2 = (joint2.getVX() + joint2.getAngularV() * r2BPerp[0]) * nX + (joint2.getVY() + joint2.getAngularV() * r2BPerp[1]) * nY;
        //the velocity of the joint at the point of the collision is a linear interpolation between points A and B
        double vimprel = (vX + angularV * rPerp1x) * nX + (vY + angularV * rPerp1y) * nY - ((temp2 - temp1) * t + temp1);

        if (vimprel > 0.0) return 0;

        temp1 = (joint1.getVX() + joint1.getAngularV() * r2APerp[0]) * -nY + (joint1.getVY() + joint1.getAngularV() * r2APerp[1]) * nX;
        temp2 = (joint2.getVX() + joint2.getAngularV() * r2BPerp[0]) * -nY + (joint2.getVY() + joint2.getAngularV() * r2BPerp[1]) * nX;
        double vtrel = (vX + angularV * rPerp1x) * -nY + (vY + angularV * rPerp1y) * nX - ((temp2 - temp1) * t + temp1);

        //calculate inverses needed
        double inverseMassA = joint1.isMovable() ? 1.0 / joint1.getMass() : 0.0;
        double inverseInertiaA = joint1.isMovable() && !joint1.lockedRotation ? 1.0 / joint1.getInertia() : 0.0;
        double inverseMassB = joint2.isMovable() ? 1.0 / joint2.getMass() : 0.0;
        double inverseInertiaB = joint2.isMovable() && !joint2.lockedRotation ? 1.0 / joint2.getInertia() : 0.0;

        vimprel = vimprel * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(joint1, joint2));


        double mu = getCOEFFICIENT_OF_FRICTION(joint1, joint2);

        //MATRIX
        constraintInfo.add(new Triplet(new double[]{rPerp1x, rPerp1y}, new double[]{nX, nY}, vimprel));
        applicationInfo.add(new Triplet(new double[]{rPerp1x, rPerp1y}, new double[]{r2APerp[0], r2APerp[1], inverseMassA, inverseInertiaA, r2BPerp[0], r2BPerp[1], inverseMassB, inverseInertiaB, t}, new double[]{nX, nY}, joint1, joint2));
        if (mu != 0.0) {
            frictionInfo.add(new Triplet(applicationInfo.size() - 1, mu, vtrel));
        }
        bodyInfo.add(new int[]{collidingIDs.get(h)[0], collidingIDs.get(h)[1], collidingIDs.get(h)[2]});

        double magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
        if (joint1.isMovable() && joint2.isMovable()) {
            double multiplier = -((magnitude - sim.MTV_EPSILON) / magnitude) * (getCompoundMass() / (joint1.getCompoundMass() + joint2.getCompoundMass()));
            multiplier *= 1.0 + (sim.MTV_EPSILON / Math.abs(multiplier));
            joint1.newposX += MTVs.get(h)[0] * multiplier;
            joint2.newposX += MTVs.get(h)[0] * multiplier;
            joint1.newposY += MTVs.get(h)[1] * multiplier;
            joint2.newposY += MTVs.get(h)[1] * multiplier;
        }

        return 1;
    }
    private void calcImpulseSolidJointIfObstacle(double rPerp1x, double rPerp1y, double nX, double nY, int h) {
        Rigidbody joint1 = Rigidbody.get(-collidingIDs.get(h)[0] - 2);
        Rigidbody joint2 = Rigidbody.get(-collidingIDs.get(h)[1] - 2);

        //these go from the center of joint 1 (point A body) to point A and from joint 2 to point B
        //these values will be made perpendicular after the linear interpolation value t is calculated.
        double[] r2APerp = new double[]{0.0,0.0};
        double[] r2BPerp = new double[]{0.0,0.0};
        Joint attachment = joint1.attachments.get(collidingIDs.get(h)[2]);
        r2APerp[0] = attachment.offsetFromCMParent[0];
        r2APerp[1] = attachment.offsetFromCMParent[1];
        r2BPerp[0] = attachment.offsetFromCMOther[0];
        r2BPerp[1] = attachment.offsetFromCMOther[1];

        double temp1 = (contactPoints.get(h)[0] - joint1.getPosX() - r2APerp[0]) * (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0])
                + (contactPoints.get(h)[1] - joint1.getPosY() - r2APerp[1]) * (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]);
        double lineMagnitude = (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0]) * (joint2.getPosX() + r2BPerp[0] - joint1.getPosX() - r2APerp[0])
                + (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]) * (joint2.getPosY() + r2BPerp[1] - joint1.getPosY() - r2APerp[1]);
        double t = temp1 / lineMagnitude;
        temp1 = r2APerp[0];
        r2APerp[0] = -r2APerp[1];
        r2APerp[1] = temp1;
        temp1 = r2BPerp[0];
        r2BPerp[0] = -r2BPerp[1];
        r2BPerp[1] = temp1;

        temp1 = (joint1.getVX() + joint1.getAngularV() * r2APerp[0]) * nX + (joint1.getVY() + joint1.getAngularV() * r2APerp[1]) * nY;
        double temp2 = (joint2.getVX() + joint2.getAngularV() * r2BPerp[0]) * nX + (joint2.getVY() + joint2.getAngularV() * r2BPerp[1]) * nY;
        //the velocity of the joint at the point of the collision is a linear interpolation between points A and B
        double vimprel = (vX + angularV * rPerp1x) * nX + (vY + angularV * rPerp1y) * nY - ((temp2 - temp1) * t + temp1);

        if (vimprel > 0.0) return;

        temp1 = (joint1.getVX() + joint1.getAngularV() * r2APerp[0]) * -nY + (joint1.getVY() + joint1.getAngularV() * r2APerp[1]) * nX;
        temp2 = (joint2.getVX() + joint2.getAngularV() * r2BPerp[0]) * -nY + (joint2.getVY() + joint2.getAngularV() * r2BPerp[1]) * nX;
        double vtrel = (vX + angularV * rPerp1x) * -nY + (vY + angularV * rPerp1y) * nX - ((temp2 - temp1) * t + temp1);

        //calculate relevant dot products
        double r2APerpndot = r2APerp[0] * nX + r2APerp[1] * nY;
        double r2APerptdot = r2APerp[0] * -nY + r2APerp[1] * nX;
        double r2BPerpndot = r2BPerp[0] * nX + r2BPerp[1] * nY;
        double r2BPerptdot = r2BPerp[0] * -nY + r2BPerp[1] * nX;
        double ndotPerp1 = rPerp1x * nX + rPerp1y * nY;
        double tdotPerp1 = rPerp1x * -nY + rPerp1y * nX;

        //calculate inverses needed
        double inverseMassA = joint1.isMovable() ? 1.0 / joint1.getMass() : 0.0;
        double inverseInertiaA = joint1.isMovable() && !joint1.lockedRotation ? 1.0 / joint1.getInertia() : 0.0;
        double inverseMassB = joint2.isMovable() ? 1.0 / joint2.getMass() : 0.0;
        double inverseInertiaB = joint2.isMovable() && !joint2.lockedRotation ? 1.0 / joint2.getInertia() : 0.0;
        double inverseInertia = isMovable() && !lockedRotation ? 1.0 / inertia : 0.0;
        double inverseMass = isMovable() ? 1.0 / mass : 0.0;

        vimprel = vimprel * -(1.0 + getCOEFFICIENT_OF_RESTITUTION(joint1, joint2));
        vtrel = -vtrel;

        double cA = (1.0 - t) * (inverseMassA + inverseInertiaA * r2APerpndot * r2APerpndot);
        double cB = t * (inverseMassB + inverseInertiaB * r2BPerpndot * r2BPerpndot);
        double meff = inverseMass + inverseInertia * ndotPerp1 * ndotPerp1 + (cB - cA) * t + cA;

        //solve the linear equation
        double jr = vimprel / meff;

        meff = inverseMass + inverseInertia * tdotPerp1 * tdotPerp1;
        cA = inverseMassA + inverseInertiaA * r2APerptdot * r2APerptdot;
        cB = inverseMassB + inverseInertiaB * r2BPerptdot * r2BPerptdot;
        meff += (cB - cA) * t + cA;
        double jf = vtrel / meff;
        double dynamicFriction = Math.abs(getCOEFFICIENT_OF_FRICTION(joint1, joint2) * jr) * Math.signum(jf);
        if (Math.abs(dynamicFriction) <= Math.abs(jf)) {
            jf = dynamicFriction;
        }

        if (joint1.isMovable()) {
            joint1.newvX += -jr * (1.0 - t) * inverseMassA * nX + jf * (1.0 - t) * inverseMassA * nY;
            joint1.newvY += -jr * (1.0 - t) * inverseMassA * nY - jf * (1.0 - t) * inverseMassA * nX;
            if (!joint1.lockedRotation) {
                joint1.newangularV += -jr * (1.0 - t) * inverseInertiaA * r2APerpndot - jf * (1.0 - t) * inverseInertiaA * r2APerptdot;
            }
        }
        if (joint2.isMovable()) {
            joint2.newvX += -jr * t * inverseMassB * nX + jf * t * inverseMassB * nY;
            joint2.newvY += -jr * t * inverseMassB * nY - jf * t * inverseMassB * nX;
            if (!joint2.lockedRotation) {
                joint2.newangularV += -jr * t * inverseInertiaB * r2BPerpndot - jf * t * inverseInertiaB * r2BPerptdot;
            }
        }

        double magnitude = Math.sqrt(MTVs.get(h)[0] * MTVs.get(h)[0] + MTVs.get(h)[1] * MTVs.get(h)[1]);
        if (joint1.isMovable() && joint2.isMovable()) {
            double multiplier = -((magnitude - sim.MTV_EPSILON) / magnitude) * (getCompoundMass() / (joint1.getCompoundMass() + joint2.getCompoundMass()));
            multiplier *= 1.0 + (sim.MTV_EPSILON / Math.abs(multiplier));
            if (joint1.compoundBody != null) joint1.compoundBody.changePosition(MTVs.get(h)[0] * multiplier,  MTVs.get(h)[1] * multiplier);
            else {
                joint1.newposX += MTVs.get(h)[0] * multiplier;
                joint1.newposY += MTVs.get(h)[1] * multiplier;
            }
            if (joint2.compoundBody != null) joint2.compoundBody.changePosition(MTVs.get(h)[0] * multiplier,  MTVs.get(h)[1] * multiplier);
            else {
                joint2.newposX += MTVs.get(h)[0] * multiplier;
                joint2.newposY += MTVs.get(h)[1] * multiplier;
            }
        }
    }
    private double[] calculateGravity() {
        double sumaX = 0.0;
        double sumaY = 0.0;
        for (Rigidbody rigidbody : rigidbodies) {
            if (rigidbody != null && rigidbody.ID != ID && rigidbody.simID == simID && !rigidbody.isHitbox) {
                double rSquared = (posX - rigidbody.getPosX()) * (posX - rigidbody.getPosX()) + (posY - rigidbody.getPosY()) * (posY - rigidbody.getPosY());
                if (rSquared > 0.0) {
                    double r = Math.sqrt(rSquared);
                    double magnitude = (sim.GRAVITATIONAL_CONSTANT * rigidbody.getMass()) / (rSquared);
                    sumaX = sumaX + (magnitude / r) * (rigidbody.getPosX() - posX);
                    sumaY = sumaY + (magnitude / r) * (rigidbody.getPosY() - posY);
                }
            }
        }
        return(new double[]{sumaX, sumaY});
    }
    private void calculateRepulsion() {
        double REPULSION_STRENGTH = sim.REPULSION_STRENGTH;
        if (parentSoftbody != -1 && REPULSION_STRENGTH > 0.0) for (int i = 0; i < Softbody.num; i = i + 1) {
            if (Softbody.get(i) != null && Softbody.get(i).simID == simID && i != parentSoftbody) {
                double distanceToBodyX = posX - Softbody.get(i).cM[0];
                double distanceToBodyY = posY - Softbody.get(i).cM[1];
                double distanceToBody = Math.sqrt(distanceToBodyX * distanceToBodyX + distanceToBodyY * distanceToBodyY);
                if (distanceToBody <= Softbody.get(i).getRadius() + Softbody.get(i).getPointRadius() * sim.REPULSE_RADIUS_MULTIPLIER) {
                    for (int j = 0; j < Softbody.get(i).size(); j++) {
                        double dx = posX - Softbody.get(i).getMember(j).getPosX();
                        double dy = posY - Softbody.get(i).getMember(j).getPosY();
                        double distance = Math.sqrt(dx * dx + dy * dy);
                        if (distance > 0.0) {
                            double aMax = REPULSION_STRENGTH * (((Softbody.get(i).getMember(j).repulseRadius + repulseRadius) / distance) - 1.0);
                            double aMagnitude = Math.max(0.0, aMax);
                            newaX += aMagnitude * (dx / distance);
                            newaY += aMagnitude * (dy / distance);
                        }
                    }
                }
            }
        }
    }
    private int calculateJointEffects(ArrayList<Triplet> constraintInfo, ArrayList<Triplet> applicationInfo, ArrayList<int[]> bodyInfo) {
        double[] sumOfEffects = new double[6];
        int countOfShifts = 0;
        for (Joint joint : attachments) {
            double[] result = joint.calculateJointForceShift();
            joint.calculateJointImpulseConstraints(constraintInfo, applicationInfo, bodyInfo);
            for (int i = 0; i <= 4; i++) {
                sumOfEffects[i] += result[i];
            }
            if (joint.movesWithConnectedBody()) countOfShifts++;
        }

        newaX += sumOfEffects[0] / mass;
        newaY += sumOfEffects[1] / mass;
        newangularA += sumOfEffects[2] / inertia;

        newposX += sumOfEffects[3];
        newposY += sumOfEffects[4];
        return countOfShifts;
    }
    public static boolean moveByMouse(double x, double y, boolean reset, boolean mousePressed, double dt, int simID) {
        boolean output = false;
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (Rigidbody.get(i) != null && Rigidbody.get(i).simID == simID) {
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
        if (isMovable && geometry.pointInside(new double[]{currentMouseX, currentMouseY})) {
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
            double parentCos = Math.cos(angularV * dt);
            double parentSin = Math.sin(angularV * dt);
            for (Joint joint : attachments) {
                double x = joint.offsetFromCMParent[0];
                double y = joint.offsetFromCMParent[1];
                joint.offsetFromCMParent[0] = x * parentCos - y * parentSin;
                joint.offsetFromCMParent[1] = y * parentCos + x * parentSin;

                double otherCos = Math.cos(joint.connection.getAngularV() * dt);
                double otherSin = Math.sin(joint.connection.getAngularV() * dt);
                x = joint.offsetFromCMOther[0];
                y = joint.offsetFromCMOther[1];
                joint.offsetFromCMOther[0] = x * otherCos - y * otherSin;
                joint.offsetFromCMOther[1] = y * otherCos + x * otherSin;

                if (joint.type == JointType.Translational) {
                    x = joint.bounds[0];
                    y = joint.bounds[1];
                    if (joint.isTranslationalParent) {
                        joint.bounds[0] = x * parentCos - y * parentSin;
                        joint.bounds[1] = y * parentCos + x * parentSin;
                    }
                    else {
                        joint.bounds[0] = x * otherCos - y * otherSin;
                        joint.bounds[1] = y * otherCos + x * otherSin;
                    }
                }
            }
        }
    }

    //joint related methods
    public void refreshConnectedBody() {
        connectedBodies.removeIf(connectedBody -> connectedBody.members.contains(this));
        ConnectedBody connectedBody = new ConnectedBody(this);
        if (connectedBody.members.size() < 2) this.connectedBody = null;
        else connectedBodies.add(connectedBody);
    }
    public void refreshCompoundBody() {
        compoundBodies.removeIf(compoundBody -> compoundBody.members.contains(this));
        CompoundBody compoundBody = new CompoundBody(this);
        if (compoundBody.members.size() < 2) this.compoundBody = null;
        else compoundBodies.add(compoundBody);
    }

    public void springAttach(Rigidbody other, double SPRING_STRENGTH, double SPRING_DAMPING) {
        springAttach(other, SPRING_STRENGTH, SPRING_DAMPING, new double[]{0.0, 0.0}, new double[]{0.0, 0.0}, false);
    }
    public void springAttach(Rigidbody other, double SPRING_STRENGTH, double SPRING_DAMPING, double[] parentOffset, double[] otherOffset, boolean solid) {
        double distance = Math.sqrt((other.posX + otherOffset[0] - posX - parentOffset[0]) * (other.posX + otherOffset[0] - posX - parentOffset[0]) + (other.posY + otherOffset[1] - posY - parentOffset[1]) * (other.posY + otherOffset[1] - posY - parentOffset[1]));
        Joint joint1 = Joint.createSpring(this, other, otherOffset, parentOffset,
                distance, SPRING_STRENGTH, SPRING_DAMPING, -1, -1);
        Joint joint2 = Joint.createSpring(other, this, parentOffset, otherOffset,
                distance, SPRING_STRENGTH, SPRING_DAMPING, -1, -1);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
        if (solid) joint1.makeSolid();
    }
    public void distanceSpringAttach(Rigidbody other, double enforcement_strength, double leeway, double[] parentOffset, double[] otherOffset, boolean solid) {
        double distance = Math.sqrt((other.posX + otherOffset[0] - posX - parentOffset[0]) * (other.posX + otherOffset[0] - posX - parentOffset[0]) + (other.posY + otherOffset[1] - posY - parentOffset[1]) * (other.posY + otherOffset[1] - posY - parentOffset[1]));
        double effectiveMass = Math.min(mass, other.getMass());
        double SPRING_STRENGTH = enforcement_strength * effectiveMass;
        double SPRING_DAMPING = sim.SHAPE_DAMPING_COEFFICIENT_RELATOR * Math.sqrt(effectiveMass * SPRING_STRENGTH);
        if (leeway > 0.5 * distance) leeway = 0.5 * distance;
        leeway = leeway / distance;
        Joint joint1 = Joint.createSpring(this, other, otherOffset, parentOffset,
                distance, SPRING_STRENGTH, SPRING_DAMPING, 1.0 - leeway, 1.0 + leeway);
        Joint joint2 = Joint.createSpring(other, this, parentOffset, otherOffset,
                distance, SPRING_STRENGTH, SPRING_DAMPING,1.0 - leeway, 1.0 + leeway);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
        if (solid) joint1.makeSolid();
        refreshConnectedBody();
    }
    public void pinAttach(Rigidbody other, double[] parentOffset, double[] otherOffset) {
        Joint joint1 = Joint.createPin(this, other, otherOffset, parentOffset);
        Joint joint2 = Joint.createPin(other, this, parentOffset, otherOffset);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
        refreshConnectedBody();
    }
    public void revoluteAttach(Rigidbody other, double[] parentOffset, double[] otherOffset, double angleBound1, double angleBound2) {
        Joint joint1 = Joint.createRevolute(this, other, otherOffset, parentOffset, angleBound1, angleBound2);
        Joint joint2 = Joint.createRevolute(other, this, parentOffset, otherOffset, -angleBound2, -angleBound1);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
        refreshConnectedBody();
    }
    public void weldAttach(Rigidbody other, double[] parentOffset, double[] otherOffset) {
        Joint joint1 = Joint.createWeld(this, other, otherOffset, parentOffset);
        Joint joint2 = Joint.createWeld(other, this, parentOffset, otherOffset);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
        refreshCompoundBody();
        refreshConnectedBody();
    }
    public void translationalAttach(Rigidbody other, double[] parentOffset, double[] otherOffset, double[] direction, double[] bounds) {
        Joint joint1 = Joint.createTranslational(this, other, otherOffset, parentOffset, direction, bounds, true);
        Joint joint2 = Joint.createTranslational(other, this, parentOffset, otherOffset, direction, bounds, false);
        attachments.add(joint1);
        other.attachments.add(joint2);
        isAttached = true;
        other.isAttached = true;
        refreshConnectedBody();
    }

    public void setAllSpringJoints(double HOOKE_SPRING_CONSTANT, double SPRING_DAMPING_COEFFICIENT, double SPRING_MIN_DIST_MULT, double SPRING_MAX_DIST_MULT) {
        for (Joint joint : attachments) {
            joint.SPRING_CONSTANT = HOOKE_SPRING_CONSTANT;
            joint.SPRING_DAMPING = SPRING_DAMPING_COEFFICIENT;
            joint.minDistMultiplier = SPRING_MIN_DIST_MULT;
            joint.maxDistMultiplier = SPRING_MAX_DIST_MULT;
        }
    }

    public void springAttachSoftbodyConstruction(Rigidbody other) {
        boolean valid = true;
        for (Joint spring : attachments) {
            if (spring.type == JointType.Softbody && spring.connection.ID == other.ID) {
                valid = false;
                break;
            }
        }
        if (valid) {
            double distance = Math.sqrt((other.posX - posX) * (other.posX - posX) + (other.posY - posY) * (other.posY - posY));
            Joint joint1 = Joint.softbodyJointCreation(this, other, new double[]{0.0, 0.0}, new double[]{0.0, 0.0},
                    distance, 1.0, 1.0);
            Joint joint2 = Joint.softbodyJointCreation(other, this, new double[]{0.0, 0.0}, new double[]{0.0, 0.0},
                    distance, 1.0, 1.0);
            attachments.add(joint1);
            other.attachments.add(joint2);
            isAttached = true;
            other.isAttached = true;
        }
    }

    //joint methods related specifically to softbody construction and not used elsewhere
    public void setAllSoftbodyJoints(double HOOKE_SPRING_CONSTANT, double SPRING_DAMPING_COEFFICIENT, double SPRING_MIN_DIST_MULT, double SPRING_MAX_DIST_MULT) {
        for (Joint joint : attachments) {
            if (joint.type == JointType.Softbody){
                joint.SPRING_CONSTANT = HOOKE_SPRING_CONSTANT;
                joint.SPRING_DAMPING = SPRING_DAMPING_COEFFICIENT;
                joint.minDistMultiplier = SPRING_MIN_DIST_MULT;
                joint.maxDistMultiplier = SPRING_MAX_DIST_MULT;
            }
        }
    }
    //used in the construction of softbodies to find the number of attached rigidbodies within the softbody,
    public int getAttachmentNum() {
        if (parentSoftbody == -1) return(0);
        int count = 0;
        if (Softbody.get(parentSoftbody).members.contains(this)) {
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Softbody &&
                        Softbody.get(parentSoftbody).members.contains(attachment.connection)) count += 1;
            }
        }
        return(count);
    }
    //used in the construction of softbodies to find attached rigidbodies within the softbody.
    public Rigidbody getAttachment(int index) {
        if (index >= 0 && index < getAttachmentNum() && Softbody.get(parentSoftbody).members.contains(this)) {
            int count = 0;
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Softbody &&
                Softbody.get(parentSoftbody).members.contains(attachment.connection)) {
                    if (count == index) return(attachment.connection);
                    count += 1;
                }
            }
        }
        return(null);
    }
    //used in the construction of softbodies to find the rigidbodyID of an attached rigidbodies that is index-th softbody attachment.
    public int getAttachmentInt(int index) {
        Rigidbody rigidbody = getAttachment(index);
        if (rigidbody != null) return rigidbody.ID;
        return(-1);
    }
    //used in the construction of softbodies to find the number of boundary members attached
    public int getBoundaryAttachmentNum() {
        if (parentSoftbody == -1) return(0);
        int count = 0;
        if (Softbody.get(parentSoftbody).boundaryMembers.contains(ID)) {
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Softbody &&
                Softbody.get(parentSoftbody).boundaryMembers.contains(attachment.connection.ID)) count += 1;
            }
        }
        return(count);
    }
    public Joint getSoftbodyAttachmentJoint(int index) {
        if (index >= 0 && index < getAttachmentNum() && Softbody.get(parentSoftbody).members.contains(this)) {
            int count = 0;
            for (Joint attachment : attachments) {
                if (attachment.type == JointType.Softbody &&
                        Softbody.get(parentSoftbody).members.contains(attachment.connection)) {
                    if (count == index) return(attachment);
                    count += 1;
                }
            }
        }
        return(null);
    }

    //used in the construction of softbodies to generate lattices recursively.
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
                Simulation.get(simID).rigidbodyObjectsIDToGlobalID.put(Rigidbody.num - 1, Simulation.get(simID).physicsObjects.size() - 1);
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
                    }
                }
            }
        }
        x = newx;
        y = newy;

        //check if the point has already been created (temporary, later going to use local geometry to determine rather than distance check)
        boolean check = false;
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (Rigidbody.get(i) != null && i != ID && Rigidbody.get(i).parentSoftbody == parentSoftbody && Rigidbody.get(i).simID == simID) {
                double distance = (x - Rigidbody.get(i).posX) * (x - Rigidbody.get(i).posX) + (y - Rigidbody.get(i).posY) * (y - Rigidbody.get(i).posY);
                distance = Math.sqrt(distance);
                if (distance < Softbody.get(parentSoftbody).getPointRadius() || (onBoundary && distance < 2.0 * Softbody.get(parentSoftbody).getPointRadius())) {
                    inShape = false;
                    check = true;
                    break;
                }
            }
        }

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
    public boolean lockedRotation() {
        if (compoundBody != null) return compoundBody.lockedRotation;
        else return lockedRotation;
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
    public static CompoundBody getCompound(int index) {
        return compoundBodies.get(index);
    }
    public static int compoundNum() {
        return compoundBodies.size();
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
    public void setCompoundPos(double posX, double posY) {
        if (compoundBody != null) compoundBody.setPosition(posX, posY);
        else {
            setPosX(posX);
            setPosY(posY);
        }
    }
    public void setConnectedBodyPos(double posX, double posY) {
        if (connectedBody != null) connectedBody.changePosition(posX - this.posX, posY - this.posY);
        else {
            setPosX(posX);
            setPosY(posY);
        }
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
    public void setCompoundV(double vX, double vY) {
        if (compoundBody != null) compoundBody.setVelocity(vX, vY);
        else {
            setVX(vX);
            setVY(vY);
        }
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
    public void setCompoundA(double aX, double aY) {
        if (compoundBody != null) compoundBody.setAcceleration(aX, aY);
        else {
            setAX(aX);
            setAY(aY);
        }
    }
    public void setCompoundAInitialForce(double aX, double aY) {
        if (compoundBody != null) {
            for (Rigidbody member : compoundBody.members) {
                member.initialExternalForces[0] = aX;
                member.initialExternalForces[1] = aY;
            }
        }
        else {
            initialExternalForces[0] = aX;
            initialExternalForces[1] = aY;
        }
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
    public void setCompoundInitialAngularA(double angularA) {
        if (compoundBody != null) {
            compoundBody.initialExternalAngularA = angularA;
        }
        else {
            initialExternalTorque = angularA;
        }
    }
    public void setCompoundAngularV(double angularV) {
        if (compoundBody != null) compoundBody.setAngularV(angularV);
        else {
            setAngularV(angularV);
        }
    }

    public double getMass() {
        return(mass);
    }
    public double getCompoundMass() {
        if (compoundBody != null) return compoundBody.mass;
        else return mass;
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
        if (compoundBody == null) return(isMovable);
        else return compoundBody.movable;
    }
    public void setIsMovable(boolean a) {
        isMovable = a;
    }

    public static Material getMaterial(int index) {
        try {
            Material returnMaterial;
            if (index >= 0) returnMaterial = Simulation.get(Rigidbody.get(index).simID).getObject("Rigidbody", index).material;
            else if (index <= -2) {
                returnMaterial = Simulation.get(Rigidbody.get(-index - 2).simID).getObject("Rigidbody", -index - 2).material;
            }
            else returnMaterial = Simulation.defaultMaterial;
            if (returnMaterial != null) return (returnMaterial);
            else {
                if (Simulation.showCreationInfoErrorMsgs) System.out.println("Material of that object is unassigned, so the default was assumed.");
                return(Simulation.defaultMaterial);
            }
        }
        catch (Exception e) {
            System.out.println(e);
        }
        return(null);
    }
    public double getCOEFFICIENT_OF_RESTITUTION(Rigidbody other) {
        double a = other.COEFFICIENT_OF_RESTITUTION;
        double b = COEFFICIENT_OF_RESTITUTION;
        if (adoptOnlyOtherSurface) return(a);
        else return(Math.sqrt((a * a + b * b) * 0.5));
    }
    public double getCOEFFICIENT_OF_RESTITUTION() {
        double a = sim.COEFFICIENT_OF_RESTITUTION;
        double b = COEFFICIENT_OF_RESTITUTION;
        if (adoptOnlyOtherSurface) return(a);
        else return(Math.sqrt((a * a + b * b) * 0.5));
    }
    public double getCOEFFICIENT_OF_RESTITUTION(Rigidbody joint1, Rigidbody joint2) {
        double a = joint1.COEFFICIENT_OF_RESTITUTION;
        double b = joint2.COEFFICIENT_OF_RESTITUTION;
        double c = (a * a + b * b) * 0.5;
        double d = COEFFICIENT_OF_RESTITUTION;
        if (adoptOnlyOtherSurface) return(Math.sqrt(c));
        else return(Math.sqrt((c + d * d) * 0.5));
    }
    public double getCOEFFICIENT_OF_FRICTION(Rigidbody other) {
        double a = other.COEFFICIENT_OF_FRICTION;
        double b = COEFFICIENT_OF_FRICTION;
        if (adoptOnlyOtherSurface) return(a);
        else return((a + b) * 0.5);
    }
    public double getCOEFFICIENT_OF_FRICTION() {
        double a = sim.COEFFICIENT_OF_FRICTION;
        double b = COEFFICIENT_OF_FRICTION;
        if (adoptOnlyOtherSurface) return(a);
        else return((a + b) * 0.5);
    }
    public double getCOEFFICIENT_OF_FRICTION(Rigidbody joint1, Rigidbody joint2) {
        double a = joint1.COEFFICIENT_OF_FRICTION;
        double b = joint2.COEFFICIENT_OF_FRICTION;
        double c = COEFFICIENT_OF_FRICTION;
        if (adoptOnlyOtherSurface) return((a + b) * 0.5);
        else return((a + b + c) / 3.0);
    }
}

