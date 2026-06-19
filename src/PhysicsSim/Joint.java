package PhysicsSim;

import java.util.ArrayList;

class Joint {
    final Rigidbody parent;
    final Rigidbody connection;
    final double[] offsetFromCMParent;
    final double[] offsetFromCMOther;
    double SPRING_CONSTANT;
    double SPRING_DAMPING;
    private final double idealDistance;
    private boolean isSolid = false;
    boolean isTranslationalParent = false;

    double maxDistMultiplier; //this variable is used in both distance constraint and translation joints
    double minDistMultiplier; //same with this one
    double[] bounds = new double[2]; //this variable is used for angle bounds and translational joint DIRECTION vector
    final JointType type;
    //joint force calculations come in a double[] of length 8: Fx, Fy, torque, mvx, mvy, mAngularV, xShift, yShift

    //spring
    private Joint(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset,
                 double idealDistance, double SPRING_CONSTANT, double SPRING_DAMPING, double minDistMultiplier, double maxDistMultiplier, JointType type) {
        this.parent = parent;
        this.connection = connection;
        offsetFromCMParent = new double[]{thisOffset[0], thisOffset[1]};
        offsetFromCMOther = new double[]{otherOffset[0], otherOffset[1]};
        this.idealDistance = idealDistance;
        this.SPRING_CONSTANT = SPRING_CONSTANT;
        this.SPRING_DAMPING = SPRING_DAMPING;
        this.minDistMultiplier = minDistMultiplier;
        this.maxDistMultiplier = maxDistMultiplier;
        this.type = type;
    }
    //default / internal
    private Joint(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset, JointType type) {
        this.parent = parent;
        this.connection = connection;
        offsetFromCMParent = new double[]{thisOffset[0], thisOffset[1]};
        offsetFromCMOther = new double[]{otherOffset[0], otherOffset[1]};
        this.SPRING_CONSTANT = 0.0;
        this.SPRING_DAMPING = 0.0;
        this.idealDistance = 0.0;
        this.maxDistMultiplier = -1.0;
        this.minDistMultiplier = -1.0;
        this.type = type;
    }
    //revolute
    private Joint(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset, double[] angleBounds, JointType type) {
        this.parent = parent;
        this.connection = connection;
        offsetFromCMParent = new double[]{thisOffset[0], thisOffset[1]};
        offsetFromCMOther = new double[]{otherOffset[0], otherOffset[1]};
        this.SPRING_CONSTANT = 0.0;
        this.SPRING_DAMPING = 0.0;
        this.idealDistance = 0.0;
        this.maxDistMultiplier = -1.0;
        this.minDistMultiplier = -1.0;
        this.bounds = new double[]{angleBounds[0], angleBounds[1]};
        this.type = type;
    }
    //translation
    private Joint(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset,
                  double[] direction, double[] bounds, boolean isTranslationalParent, JointType type) {
        this.parent = parent;
        this.connection = connection;
        offsetFromCMParent = new double[]{thisOffset[0], thisOffset[1]};
        offsetFromCMOther = new double[]{otherOffset[0], otherOffset[1]};
        this.SPRING_CONSTANT = 0.0;
        this.SPRING_DAMPING = 0.0;
        this.idealDistance = 0.0;
        this.maxDistMultiplier = Math.max(bounds[0], bounds[1]);
        this.minDistMultiplier = Math.min(bounds[0], bounds[1]);
        this.bounds = new double[]{direction[0], direction[1]};
        this.isTranslationalParent = isTranslationalParent;
        this.type = type;

    }
    public static boolean checkIfSameJoint(int parentID1, int parentID2, Joint joint1, Joint joint2) {
        if (parentID1 == parentID2) return true;
        if (joint1 != null) {
            if (joint1.parent.ID == parentID2 || joint1.connection.ID == parentID2) return true;
            //determine if we should ignore it because they belong to same softbody
            if (parentID2 != -1 && Rigidbody.get(parentID2).parentSoftbody != -1 && (joint1.parent.parentSoftbody == Rigidbody.get(parentID2).parentSoftbody || joint1.connection.parentSoftbody == Rigidbody.get(parentID2).parentSoftbody)
                    && !Softbody.get(joint1.parent.parentSoftbody).selfCollides()) return true;
        }
        if (joint2 != null) {
            if (joint2.parent.ID == parentID1 || joint2.connection.ID == parentID1) return true;
            if (parentID1 != -1 && Rigidbody.get(parentID1).parentSoftbody != -1 && (joint2.parent.parentSoftbody == Rigidbody.get(parentID1).parentSoftbody || joint2.connection.parentSoftbody == Rigidbody.get(parentID1).parentSoftbody)
                    && !Softbody.get(joint2.parent.parentSoftbody).selfCollides()) return true;
        }
        return false;
    }


    public static Joint createSpring(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset,
                                     double idealDistance, double SPRING_CONSTANT, double SPRING_DAMPING, double minDistMultiplier, double maxDistMultiplier) {
        return new Joint(parent, connection, otherOffset, thisOffset, idealDistance, SPRING_CONSTANT, SPRING_DAMPING, minDistMultiplier, maxDistMultiplier, JointType.Spring);
    }
    public static Joint softbodyJointCreation(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset,
                                      double idealDistance, double SPRING_CONSTANT, double SPRING_DAMPING) {
        return new Joint(parent, connection, otherOffset, thisOffset, idealDistance, SPRING_CONSTANT, SPRING_DAMPING, -1, -1, JointType.Softbody);
    }
    public static Joint createPin(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset) {
        return new Joint(parent, connection, otherOffset, thisOffset, JointType.Pin);
    }
    public static Joint createRevolute(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset, double angleBound1, double angleBound2) {
        double[] angleBounds = new double[]{angleMod(angleBound1), angleMod(angleBound2)};
        return new Joint(parent, connection, otherOffset, thisOffset, angleBounds, JointType.Revolute);
    }
    private static double angleMod(double theta) {
        theta += Math.PI;
        if (theta >= 0.0) theta = theta % Math.TAU;
        else {
            theta = (-theta) % Math.TAU;
            theta = Math.TAU - theta;
            if (theta == Math.TAU) theta = 0.0;
        }
        theta -= Math.PI;
        return theta;
    }
    public static Joint createWeld(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset) {
        return new Joint(parent, connection, otherOffset, thisOffset, JointType.Weld);
    }
    public static Joint createTranslational(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset,
                                            double[] direction, double[] bounds, boolean isTranslationalParent) {
        return new Joint(parent, connection, otherOffset, thisOffset, direction, bounds, isTranslationalParent, JointType.Translational);
    }
    public double[] calculateJointForceShift() {
        if (type == JointType.Weld) return new double[5];
        double dx = (connection.getPosX() + offsetFromCMOther[0]) - (parent.getPosX() + offsetFromCMParent[0]);
        double dy = (connection.getPosY() + offsetFromCMOther[1]) - (parent.getPosY() + offsetFromCMParent[1]);
        double distance = Math.sqrt(dx * dx + dy * dy);
        double nX = dx / distance;
        double nY = dy / distance;
        if (Double.isNaN(nX) || Double.isNaN(nY)) {
            nX = 0.0;
            nY = 0.0;
        }
        //0 is x force, 1 is y force, 2 is torque, 3 is x shift, 4 is y shift
        double[] returnArray = new double[5];

        //spring effects
        //these are the soft constraints (distance-constrain joints are spring joints under the hood)
        if (type == JointType.Spring || type == JointType.Softbody) {
            double[] fS = new double[2];
            fS[0] = SPRING_CONSTANT * (dx - idealDistance * nX);
            fS[1] = SPRING_CONSTANT * (dy - idealDistance * nY);
            double vdotN = (parent.getVX() - offsetFromCMParent[1] * parent.getAngularV() - connection.getVX() + offsetFromCMParent[1] * connection.getAngularV()) * nX + (parent.getVY() + offsetFromCMParent[0] * parent.getAngularV() - connection.getVY() - offsetFromCMOther[0] * connection.getAngularV()) * nY;
            double multiplier = vdotN * SPRING_DAMPING;
            double[] fD = new double[]{-nX * multiplier, -nY * multiplier};
            if (fD[0] * fD[0] + fD[1] * fD[1] < fS[0] * fS[0] + fS[1] * fS[1]) {
                double crossProduct = -offsetFromCMParent[1] * (fS[0] + fD[0]) + offsetFromCMParent[0] * (fS[1] + fD[1]);
                returnArray[0] += fS[0] + fD[0];
                returnArray[1] += fS[1] + fD[1];
                returnArray[2] += crossProduct;
            }

            //distance constraint impulses
            if (maxDistMultiplier > 0.0 && distance > idealDistance * maxDistMultiplier) {
                multiplier = (distance - idealDistance * maxDistMultiplier) * (connection.isMovable() ? (connection.getCompoundMass() / (connection.getCompoundMass() + parent.getCompoundMass())) : 1.0);
                returnArray[3] = nX * multiplier;
                returnArray[4] = nY * multiplier;
            }
            else if (minDistMultiplier > 0.0 && distance < idealDistance * minDistMultiplier) {
                multiplier = (distance - idealDistance * minDistMultiplier) * (connection.isMovable() ? (connection.getCompoundMass() / (connection.getCompoundMass() + parent.getCompoundMass())) : 1.0);
                returnArray[3] = nX * multiplier;
                returnArray[4] = nY * multiplier;
            }
        }

        //pin effects
        //these are the hard constraints
        if (type == JointType.Pin || type == JointType.Revolute) {
            double multiplier = distance * (connection.isMovable() ? (connection.getCompoundMass() / (connection.getCompoundMass() + parent.getCompoundMass())) : 1.0);
            returnArray[3] = nX * multiplier;
            returnArray[4] = nY * multiplier;
        }

        if (type == JointType.Translational) {
            if (!isTranslationalParent) {
                dx = -dx;
                dy = -dy;
            }
            double boundsDistance = dx * bounds[0] + dy * bounds[1];

            //if moving out, take pin approach
            if (boundsDistance > maxDistMultiplier || boundsDistance < minDistMultiplier) {
                if (boundsDistance > maxDistMultiplier) {
                    dx -= bounds[0] * maxDistMultiplier;
                    dy -= bounds[1] * maxDistMultiplier;
                }
                else {
                    dx -= bounds[0] * minDistMultiplier;
                    dy -= bounds[1] * minDistMultiplier;
                }
                distance = Math.sqrt(dx * dx + dy * dy);
                nX = dx / distance;
                nY = dy / distance;
                double multiplier = distance * (connection.isMovable() ? (connection.getCompoundMass() / (connection.getCompoundMass() + parent.getCompoundMass())) : 1.0);
                returnArray[3] = nX * multiplier;
                returnArray[4] = nY * multiplier;
                if (!isTranslationalParent) {
                    returnArray[3] = -returnArray[3];
                    returnArray[4]= -returnArray[4];
                }
            }
            else {
                distance = dx * -bounds[1] + dy * bounds[0];
                double multiplier = distance * (connection.isMovable() ? (connection.getCompoundMass() / (connection.getCompoundMass() + parent.getCompoundMass())) : 1.0);
                returnArray[3] = -bounds[1] * multiplier;
                returnArray[4] = bounds[0] * multiplier;
                if (!isTranslationalParent) {
                    returnArray[3] = -returnArray[3];
                    returnArray[4] = -returnArray[4];
                }
            }
        }
        return returnArray;
    }
    public void calculateJointImpulseConstraints(ArrayList<Triplet> constraintInfo, ArrayList<Triplet> applicationInfo, ArrayList<int[]> bodyInfo) {
        double dx = (connection.getPosX() + offsetFromCMOther[0]) - (parent.getPosX() + offsetFromCMParent[0]);
        double dy = (connection.getPosY() + offsetFromCMOther[1]) - (parent.getPosY() + offsetFromCMParent[1]);
        double distance = Math.sqrt(dx * dx + dy * dy);
        double[] n = new double[]{dx / distance, dy / distance};
        if (Double.isNaN(n[0]) || Double.isNaN(n[1])) {
            n[0] = 0.0;
            n[1] = 0.0;
        }
        double[] rPerp1 = new double[]{-offsetFromCMParent[1], offsetFromCMParent[0]};
        double[] rPerp2 = new double[]{-offsetFromCMOther[1], offsetFromCMOther[0], (connection.isMovable() ? 1.0 / connection.getMass() : 0.0), (connection.isMovable() && !connection.lockedRotation() ? 1.0 / connection.getInertia() : 0.0)};
        //0 is x force, 1 is y force, 2 is torque, 3 is x impulse, 4 is y impulse, 5 is angular momentum, 6 is x shift, 7 is y shift

        //spring effects
        //these are the soft constraints (distance-constraint joints are spring joints under the hood)
        if (type == JointType.Spring || type == JointType.Softbody) {
            double vdotN = (parent.getVX() - offsetFromCMParent[1] * parent.getAngularV() - connection.getVX() + offsetFromCMOther[1] * connection.getAngularV()) * n[0]
                    + (parent.getVY() + offsetFromCMParent[0] * parent.getAngularV() - connection.getVY() - offsetFromCMOther[0] * connection.getAngularV()) * n[1];
            //distance constraint impulses
            if ((maxDistMultiplier > 0.0 && distance > idealDistance * maxDistMultiplier && vdotN < 0.0) || (minDistMultiplier > 0.0 && distance < idealDistance * minDistMultiplier && vdotN > 0.0)) {
                constraintInfo.add(new Triplet(rPerp1, n, -vdotN));
                applicationInfo.add(new Triplet(rPerp1, rPerp2, n));
                bodyInfo.add(new int[]{connection.ID});
            }
        }

        //pin effects
        //these are the hard constraints
        if (type == JointType.Pin || type == JointType.Revolute) {
            double vxRelChange = -(parent.getVX() + rPerp1[0] * parent.getAngularV() - connection.getVX() - rPerp2[0] * connection.getAngularV());
            double vyRelChange = -(parent.getVY() + rPerp1[1] * parent.getAngularV() - connection.getVY() - rPerp2[1] * connection.getAngularV());

            constraintInfo.add(new Triplet(rPerp1, new double[]{1.0, 0.0}, vxRelChange));
            applicationInfo.add(new Triplet(rPerp1, rPerp2, new double[]{1.0, 0.0}));
            bodyInfo.add(new int[]{connection.ID});
            constraintInfo.add(new Triplet(rPerp1, new double[]{0.0, 1.0}, vyRelChange));
            applicationInfo.add(new Triplet(rPerp1, rPerp2, new double[]{0.0, 1.0}));
            bodyInfo.add(new int[]{connection.ID});

            if (type == JointType.Revolute) {
                double angle = Math.atan2(offsetFromCMParent[1] * offsetFromCMOther[0] - offsetFromCMParent[0] * offsetFromCMOther[1],
                        -offsetFromCMParent[0] * offsetFromCMOther[0] - offsetFromCMParent[1] * offsetFromCMOther[1]);
                double angularVRelChange = -parent.getAngularV() + connection.getAngularV();
                double angleViolation = 0.0;
                double angleAverage = 0.5 * (bounds[0] + bounds[1]);
                if (angle > bounds[1] && (bounds[0] < bounds[1] || angle < angleAverage)) {
                    angleViolation = angle - bounds[1];
                }
                if (angle < bounds[0] && (bounds[0] < bounds[1] || angle > angleAverage)) {
                    angleViolation = angle - bounds[0];
                }

                if (angleViolation * angularVRelChange > 0.0) {
                    double[] nullArr = null;
                    constraintInfo.add(new Triplet(nullArr, nullArr, angularVRelChange));
                    //the first two values of rPerp2 are not used, but the two inverse values are.
                    applicationInfo.add(new Triplet(nullArr, rPerp2, nullArr));
                    bodyInfo.add(new int[]{connection.ID});
                }
            }
        }

        if (type == JointType.Translational) {
            if (!isTranslationalParent) {
                dx = -dx;
                dy = -dy;
            }
            double boundsDistance = dx * bounds[0] + dy * bounds[1];


            //if moving out, take pin approach
            //the multipliers here are used on bound limits
            if (boundsDistance > maxDistMultiplier || boundsDistance < minDistMultiplier) {
                if (boundsDistance > maxDistMultiplier) {
                    if (isTranslationalParent) {
                        rPerp1[0] = rPerp1[0] - bounds[1] * maxDistMultiplier;
                        rPerp1[1] = rPerp1[1] + bounds[0] * maxDistMultiplier;
                    }
                    else {
                        rPerp2[0] = rPerp2[0] - bounds[1] * maxDistMultiplier;
                        rPerp2[1] = rPerp2[1] + bounds[0] * maxDistMultiplier;
                    }
                }
                else {
                    if (isTranslationalParent) {
                        rPerp1[0] = rPerp1[0] - bounds[1] * minDistMultiplier;
                        rPerp1[1] = rPerp1[1] + bounds[0] * minDistMultiplier;
                    }
                    else {
                        rPerp2[0] = rPerp2[0] - bounds[1] * minDistMultiplier;
                        rPerp2[1] = rPerp2[1] + bounds[0] * minDistMultiplier;
                    }
                }
            }
            else {
                distance = dx * bounds[0] + dy * bounds[1];
                dx = distance * bounds[0];
                dy = distance * bounds[1];
                if (isTranslationalParent) {
                    rPerp1[0] = rPerp1[0] - dy;
                    rPerp1[1] = rPerp1[1] + dx;
                }
                else {
                    rPerp2[0] = rPerp2[0] - dy;
                    rPerp2[1] = rPerp2[1] + dx;
                }
            }
            n[0] = -bounds[1];
            n[1] = bounds[0];

            double vnRelChange = ((parent.getVX() + parent.getAngularV() * rPerp1[0]) - (connection.getVX() + connection.getAngularV() * rPerp2[0])) * n[0];
            vnRelChange += ((parent.getVY() + parent.getAngularV() * rPerp1[1]) - (connection.getVY() + connection.getAngularV() * rPerp2[1])) * n[1];
            vnRelChange = -vnRelChange;

            constraintInfo.add(new Triplet(rPerp1, n, vnRelChange));
            applicationInfo.add(new Triplet(rPerp1, rPerp2, n));
            bodyInfo.add(new int[]{connection.ID});

            double vtRel = (parent.getVX() + rPerp1[0] * parent.getAngularV() - connection.getVX() - rPerp2[0] * connection.getAngularV()) * -n[1];
            vtRel += (parent.getVY() + rPerp1[1] * parent.getAngularV() - connection.getVY() - rPerp2[1] * connection.getAngularV()) * n[0];
            vtRel *= isTranslationalParent ? -1.0 : 1.0;
            if ((boundsDistance > maxDistMultiplier && vtRel < 0.0) || (boundsDistance < minDistMultiplier && vtRel > 0.0)) {
                vtRel *= isTranslationalParent ? -1.0 : 1.0;
                constraintInfo.add(new Triplet(rPerp1, new double[]{-n[1], n[0]}, -vtRel));
                applicationInfo.add(new Triplet(rPerp1, rPerp2, new double[]{-n[1], n[0]}));
                bodyInfo.add(new int[]{connection.ID});
            }
        }
    }
    public void makeSolid() {
        if (!isSolid && canBeMadeSolid()) {
            boolean valid = true;
            for (Joint joint : connection.attachments) {
                if (joint.isSolid && joint.connection.ID == parent.ID) {
                    valid = false;
                    break;
                }
            }
            if (valid) {
                isSolid = true;
                Simulation sim = Simulation.get(parent.simID);
                sim.getSAPCell(0, 0).addBox(this);
                sim.BVHtrees.get(0).addBox(this);
            }
        }
    }
    private boolean canBeMadeSolid() {
        return type == JointType.Softbody || type == JointType.Spring;
    }
    public boolean collidesWithSelfConnections() {
        return !(type == JointType.Pin || type == JointType.Weld || type == JointType.Revolute);
    }
    public boolean movesWithConnectedBody() {
        return (type == JointType.Spring && minDistMultiplier != -1 && maxDistMultiplier != -1) || type == JointType.Pin || type == JointType.Revolute || type == JointType.Weld || type == JointType.Translational;
    }
    public boolean isSolid() {
        return isSolid;
    }

    @Override
    public String toString() {
        StringBuilder result = new StringBuilder();
        result.append("Parent Rigidbody: ").append(parent.ID).append(" Connection Rigidbody: ").append(connection.ID);
        result.append(" ").append(type);
        return result.toString();
    }
}
