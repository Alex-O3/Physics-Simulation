package PhysicsSim;

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
    public double[] calculateJointForceImpulseShift() {
        double dx = (connection.getPosX() + offsetFromCMOther[0]) - (parent.getPosX() + offsetFromCMParent[0]);
        double dy = (connection.getPosY() + offsetFromCMOther[1]) - (parent.getPosY() + offsetFromCMParent[1]);
        double distance = Math.sqrt(dx * dx + dy * dy);
        double nX = dx / distance;
        double nY = dy / distance;
        if (Double.isNaN(nX) || Double.isNaN(nY)) {
            nX = 0.0;
            nY = 0.0;
        }
        //0 is x force, 1 is y force, 2 is torque, 3 is x impulse, 4 is y impulse, 5 is angular momentum, 6 is x shift, 7 is y shift
        double[] returnArray = new double[8];

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
                multiplier = (distance - idealDistance * maxDistMultiplier) * (connection.isMovable() ? (connection.getMass() / (connection.getMass() + parent.getMass())) : 1.0);
                returnArray[6] = nX * multiplier;
                returnArray[7] = nY * multiplier;
                if (vdotN < 0.0) {
                    double rPerp1nDot = -offsetFromCMParent[1] * nX + offsetFromCMParent[0] * nY;
                    double rPerp2nDot = -offsetFromCMOther[1] * nX + offsetFromCMOther[0] * nY;
                    double j = (1.0 / parent.getMass()) + (connection.isMovable() ? (1.0 / connection.getInertia()) : 0.0) + (1.0 / parent.getInertia()) * rPerp1nDot * rPerp1nDot +
                            ((connection.isMovable() && !connection.lockedRotation()) ? (1.0 / connection.getInertia()) : 0.0) * rPerp2nDot * rPerp2nDot;
                    j = -vdotN / j;
                    returnArray[3] += j * nX;
                    returnArray[4] += j * nY;
                    returnArray[5] += rPerp1nDot * j;
                }
            }
            else if (minDistMultiplier > 0.0 && distance < idealDistance * minDistMultiplier) {
                multiplier = (distance - idealDistance * minDistMultiplier) * (connection.isMovable() ? (connection.getMass() / (connection.getMass() + parent.getMass())) : 1.0);
                returnArray[6] = nX * multiplier;
                returnArray[7] = nY * multiplier;
                if (vdotN > 0.0) {
                    double rPerp1nDot = -offsetFromCMParent[1] * nX + offsetFromCMParent[0] * nY;
                    double rPerp2nDot = -offsetFromCMOther[1] * nX + offsetFromCMOther[0] * nY;
                    double j = (1.0 / parent.getMass()) + (connection.isMovable() ? (1.0 / connection.getInertia()) : 0.0) + (1.0 / parent.getInertia()) * rPerp1nDot * rPerp1nDot +
                            ((connection.isMovable() && !connection.lockedRotation()) ? (1.0 / connection.getInertia()) : 0.0) * rPerp2nDot * rPerp2nDot;
                    j = -vdotN / j;
                    returnArray[3] += j * nX;
                    returnArray[4] += j * nY;
                    returnArray[5] += rPerp1nDot * j;
                }
            }
        }

        //pin effects
        //these are the hard constraints
        if (type == JointType.Pin || type == JointType.Revolute || type == JointType.Weld) {
            double multiplier = distance * (connection.isMovable() ? (connection.getMass() / (connection.getMass() + parent.getMass())) : 1.0);
            returnArray[6] = nX * multiplier;
            returnArray[7] = nY * multiplier;
            boolean pinOnly = true;
            if (type == JointType.Revolute) {
                //this is the simple approach that might change to effectively couple the revolute impulse, but for now
                //as astute "viewers" may have noticed, some of the comments in my code are outdated, even deprecated. But, this is how this
                //part actually works: the torque and impulse factors are coupled only when the angle constraint is violated.
                double angle = Math.atan2(offsetFromCMParent[1] * offsetFromCMOther[0] - offsetFromCMParent[0] * offsetFromCMOther[1],
                        -offsetFromCMParent[0] * offsetFromCMOther[0] - offsetFromCMParent[1] * offsetFromCMOther[1]);
                boolean inBounds = (bounds[0] < bounds[1] && angle < bounds[1] && angle > bounds[0]);
                inBounds = inBounds || (bounds[0] > bounds[1] && !(angle < bounds[0] && angle > bounds[1]));;
                if (!inBounds) pinOnly = false;
                double c12 = -parent.getAngularV() + connection.getAngularV();
                double angleViolation = 0.0;
                double angleAverage = 0.5 * (bounds[0] + bounds[1]);
                if (angle > bounds[1] && (bounds[0] < bounds[1] || angle < angleAverage)) {
                    angleViolation = angle - bounds[1];
                }
                if (angle < bounds[0] && (bounds[0] < bounds[1] || angle > angleAverage)) {
                    angleViolation = angle - bounds[0];
                }

                if (angleViolation * c12 > 0.0) {
                    pinOnly = false;
                    double inverseM1 = 1.0 / parent.getMass();
                    double inverseI1 = !parent.lockedRotation() ? 1.0 / parent.getInertia() : 0.0;
                    double inverseM2 = connection.isMovable() ? 1.0 / connection.getMass() : 0.0;
                    double inverseI2 = (connection.isMovable() && !connection.lockedRotation()) ? 1.0 / connection.getInertia() : 0.0;
                    double r1px = -offsetFromCMParent[1];
                    double r1py = offsetFromCMParent[0];
                    double r2px = -offsetFromCMOther[1];
                    double r2py = offsetFromCMOther[0];

                    double c1 = inverseI1 * r1px + inverseI2 * r2px;
                    double c2 = inverseM1 + inverseM2 + inverseI1 * r1px * r1px + inverseI2 * r2px * r2px;
                    double c3 = inverseI1 * r1px * r1py + inverseI2 * r2px * r2py;
                    double c4 = -(parent.getVX() + parent.getAngularV() * r1px) + (connection.getVX() + connection.getAngularV() * r2px);
                    double c5 = inverseI1 * r1py + inverseI2 * r2py;
                    double c6 = c3;
                    double c7 = inverseM1 + inverseM2 + inverseI1 * r1py * r1py + inverseI2 * r2py * r2py;
                    double c8 = -(parent.getVY() + parent.getAngularV() * r1py) + (connection.getVY() + connection.getAngularV() * r2py);
                    double c9 = inverseI1 + inverseI2;
                    double c10 = c1;
                    double c11 = c5;

                    double D = c6 * c11 * c1 - c10 * c7 * c1 - c5 * c11 * c2 + c9 * c7 * c2 + c5 * c10 * c3 - c6 * c9 * c3;
                    double tau = c6 * c11 * c4 - c10 * c7 * c4 - c8 * c11 * c2 + c12 * c7 * c2 + c8 * c10 * c3 - c6 * c12 * c3;
                    tau /= D;
                    double jcX = c8 * c11 * c1 - c12 * c7 * c1 - c5 * c11 * c4 + c9 * c7 * c4 + c5 * c12 * c3 - c8 * c9 * c3;
                    jcX /= D;
                    double jcY = c6 * c12 * c1 - c10 * c8 * c1 - c5 * c12 * c2 + c9 * c8 * c2 + c5 * c10 * c4 - c6 * c9 * c4;
                    jcY /= D;

                    returnArray[3] += jcX;
                    returnArray[4] += jcY;
                    returnArray[5] += jcX * r1px + jcY * r1py + tau;
                }
            }
            if (pinOnly) {
                double[] rPerp1 = new double[]{-offsetFromCMParent[1], offsetFromCMParent[0]};
                double[] rPerp2 = new double[]{-offsetFromCMOther[1], offsetFromCMOther[0]};
                double vxRelChange = -(parent.getVX() + rPerp1[0] * parent.getAngularV() - connection.getVX() - rPerp2[0] * connection.getAngularV());
                double vyRelChange = -(parent.getVY() + rPerp1[1] * parent.getAngularV() - connection.getVY() - rPerp2[1] * connection.getAngularV());
                double myInverseMass = 1.0 / parent.getMass();
                double myInverseInertia = !parent.lockedRotation() ? 1.0 / parent.getInertia() : 0.0;
                double otherInverseMass = (connection.isMovable() ? (1.0 / connection.getMass()) : 0.0);
                double otherInverseInertia = ((connection.isMovable() && !connection.lockedRotation()) ? (1.0 / connection.getInertia()) : 0.0);
                double c1 = myInverseMass + myInverseInertia * rPerp1[0] * rPerp1[0]
                        + otherInverseMass + otherInverseInertia * rPerp2[0] * rPerp2[0];
                double c2 = myInverseInertia * rPerp1[0] * rPerp1[1] + otherInverseInertia * rPerp2[0] * rPerp2[1];
                double c4 = myInverseMass + myInverseInertia * rPerp1[1] * rPerp1[1] +
                        otherInverseMass + otherInverseInertia * rPerp2[1] * rPerp2[1];

                double jrX = (c2 * vyRelChange - c4 * vxRelChange) / (c2 * c2 - c1 * c4);
                double jrY = (c2 * vxRelChange - c1 * vyRelChange) / (c2 * c2 - c1 * c4);
                returnArray[3] += jrX;
                returnArray[4] += jrY;
                returnArray[5] += jrX * rPerp1[0] + jrY * rPerp1[1];
            }
        }

        if (type == JointType.Translational) {
            double boundsDistance = dx * bounds[0] + dy * bounds[1];

            double myInverseMass = 1.0 / parent.getMass();
            double myInverseInertia = !parent.lockedRotation() ? 1.0 / parent.getInertia() : 0.0;
            double otherInverseMass = (connection.isMovable() ? (1.0 / connection.getMass()) : 0.0);
            double otherInverseInertia = ((connection.isMovable() && !connection.lockedRotation()) ? (1.0 / connection.getInertia()) : 0.0);
            double[] rPerp1 = new double[]{-offsetFromCMParent[1], offsetFromCMParent[0]};
            double[] rPerp2 = new double[]{-offsetFromCMOther[1], offsetFromCMOther[0]};

            //if moving out, take pin approach
            if (boundsDistance > maxDistMultiplier || boundsDistance < minDistMultiplier) {
                if (boundsDistance > maxDistMultiplier) {
                    dx -= bounds[0] * maxDistMultiplier;
                    dy -= bounds[1] * maxDistMultiplier;
                }
                else if (boundsDistance < minDistMultiplier) {
                    dx -= bounds[0] * minDistMultiplier;
                    dy -= bounds[1] * minDistMultiplier;
                }
                distance = Math.sqrt(dx * dx + dy * dy);
                nX = dx / distance;
                nY = dy / distance;
                double multiplier = distance * (connection.isMovable() ? (connection.getMass() / (connection.getMass() + parent.getMass())) : 1.0);
                returnArray[6] = nX * multiplier;
                returnArray[7] = nY * multiplier;

                double vxRelChange = -(parent.getVX() + rPerp1[0] * parent.getAngularV() - connection.getVX() - rPerp2[0] * connection.getAngularV());
                double vyRelChange = -(parent.getVY() + rPerp1[1] * parent.getAngularV() - connection.getVY() - rPerp2[1] * connection.getAngularV());
                double c1 = myInverseMass + myInverseInertia * rPerp1[0] * rPerp1[0]
                        + otherInverseMass + otherInverseInertia * rPerp2[0] * rPerp2[0];
                double c2 = myInverseInertia * rPerp1[0] * rPerp1[1] + otherInverseInertia * rPerp2[0] * rPerp2[1];
                double c4 = myInverseMass + myInverseInertia * rPerp1[1] * rPerp1[1] +
                        otherInverseMass + otherInverseInertia * rPerp2[1] * rPerp2[1];

                double jrX = (c2 * vyRelChange - c4 * vxRelChange) / (c2 * c2 - c1 * c4);
                double jrY = (c2 * vxRelChange - c1 * vyRelChange) / (c2 * c2 - c1 * c4);
                returnArray[3] += jrX;
                returnArray[4] += jrY;
                returnArray[5] += jrX * rPerp1[0] + jrY * rPerp1[1];
            }
            else {
                nX = -bounds[1];
                nY = bounds[0];
                distance = dx * nX + dy * nY;
                double multiplier = distance * (connection.isMovable() ? (connection.getMass() / (connection.getMass() + parent.getMass())) : 1.0);
                returnArray[6] = nX * multiplier;
                returnArray[7] = nY * multiplier;

                double ndotPerp1 = rPerp1[0] * nX + rPerp1[1] * nY;
                double ndotPerp2 = rPerp2[0] * nX + rPerp2[1] * nY;
                double vnRelChange = ((parent.getVX() + parent.getAngularV() * rPerp1[0]) - (connection.getVX() + connection.getAngularV() * rPerp2[0])) * nX;
                vnRelChange += ((parent.getVY() + parent.getAngularV() * rPerp1[1]) - (connection.getVY() + connection.getAngularV() * rPerp2[1])) * nY;
                vnRelChange = -vnRelChange;
                double c1 = (myInverseMass + otherInverseMass + myInverseInertia * ndotPerp1 * ndotPerp1 + otherInverseInertia * ndotPerp2 * ndotPerp2);
                double jr = vnRelChange / c1;

                returnArray[3] += jr * nX;
                returnArray[4] += jr * nY;
                returnArray[5] += jr * ndotPerp1;
            }
        }

        return returnArray;
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
