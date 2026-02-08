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

    double maxDistMultiplier;
    double minDistMultiplier;
    double[] angleBounds = new double[2];
    final JointType type;
    //joint force calculations come in a double[] of length 8: Fx, Fy, torque, mvx, mvy, mAngularV, xShift, yShift
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
        this.angleBounds = angleBounds;
        this.type = type;
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
        if (type == JointType.Pin || type == JointType.Revolute || type == JointType.Weld) {
            double multiplier = distance * (connection.isMovable() ? (connection.getMass() / (connection.getMass() + parent.getMass())) : 1.0);
            returnArray[6] = nX * multiplier;
            returnArray[7] = nY * multiplier;
            boolean pinOnly = true;
            if (type == JointType.Revolute) {
                //this is the simple approach that might change to effectively couple the revolute impulse, but for now
                //the revolute impulse and pin impulse are calculated based on the same data and interfere. In the next cycle, the pin impulse will correct
                //this coupling as a result. Currently, the revolute impulse is modeled as a basic "imaginary" collision when the angle is too big.
                double angle = Math.atan2(offsetFromCMParent[1] * offsetFromCMOther[0] - offsetFromCMParent[0] * offsetFromCMOther[1],
                        -offsetFromCMParent[0] * offsetFromCMOther[0] - offsetFromCMParent[1] * offsetFromCMOther[1]);
                boolean inBounds = (angleBounds[0] < angleBounds[1] && angle < angleBounds[1] && angle > angleBounds[0]);
                inBounds = inBounds || (angleBounds[0] > angleBounds[1] && !(angle < angleBounds[0] && angle > angleBounds[1]));;
                if (!inBounds) pinOnly = false;
                double c12 = -parent.getAngularV() + connection.getAngularV();
                double angleViolation = 0.0;
                double angleAverage = 0.5 * (angleBounds[0] + angleBounds[1]);
                if (angle > angleBounds[1] && (angleBounds[0] < angleBounds[1] || angle < angleAverage)) {
                    angleViolation = angle - angleBounds[1];
                }
                if (angle < angleBounds[0] && (angleBounds[0] < angleBounds[1] || angle > angleAverage)) {
                    angleViolation = angle - angleBounds[0];
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

                /*if (!inBounds && false) {
                    //the normal is a line cast out at the angle bounds. For now, we assume this is perpendicular to the radius from the connection point, as this allows
                    //better correction for errors past the angle bounds.

                    double magnitude1 = Math.sqrt(offsetFromCMOther[0] * offsetFromCMOther[0] + offsetFromCMOther[1] * offsetFromCMOther[1]);
                    double n2X = (-offsetFromCMOther[1] * Math.signum(angle)) / magnitude1;
                    double n2Y = (offsetFromCMOther[0] * Math.signum(angle)) / magnitude1;
                    double magnitude2 = Math.sqrt(offsetFromCMParent[0] * offsetFromCMParent[0] + offsetFromCMParent[1] * offsetFromCMParent[1]);
                    double n1X = (-offsetFromCMParent[1] * -Math.signum(angle)) / magnitude2;
                    double n1Y = (offsetFromCMParent[0] * -Math.signum(angle)) / magnitude2;
                    if (angleBounds[0] > angleBounds[1]) {
                        n1X = -n1X;
                        n1Y = -n1Y;
                        n2X = -n2X;
                        n2Y = -n2Y;
                    }

                    //calculate the coefficients used to determine the impulses
                    double inverseM1 = 1.0 / parent.getMass();
                    double inverseI1 = !parent.lockedRotation() ? 1.0 / parent.getInertia() : 0.0;
                    double inverseM2 = connection.isMovable() ? 1.0 / connection.getMass() : 0.0;
                    double inverseI2 = (connection.isMovable() && !connection.lockedRotation()) ? 1.0 / connection.getInertia() : 0.0;
                    double r1px = -offsetFromCMParent[1];
                    double r1py = offsetFromCMParent[0];
                    double r2px = -offsetFromCMOther[1];
                    double r2py = offsetFromCMOther[0];
                    double rdotN1 = r1px * n1X + r1py * n1Y;
                    double rdotN2 = r2px * n2X + r2py * n2Y;

                    double c1 = inverseM1 * n1X - inverseM2 * n2X;
                    double c2 = inverseM1 + inverseM2 + inverseI1 * r1px * r1px + inverseI2 * r2px * r2px;
                    double c3 = inverseI1 * r1px * r1py + inverseI2 * r2px * r2py;
                    double c4 = -(parent.getVX() + parent.getAngularV() * r1px) + (connection.getVX() + connection.getAngularV() * r2px);

                    double c5 = inverseM1 * n1Y - inverseM2 * n2Y;
                    double c6 = c3;
                    double c7 = inverseM1 + inverseM2 + inverseI1 * r1py * r1py + inverseI2 * r2py * r2py;
                    double c8 = -(parent.getVY() + parent.getAngularV() * r1py) + (connection.getVY() + connection.getAngularV() * r2py);
                    double c9 = inverseI1 * (1.0 / magnitude1) * rdotN1 * r1px + inverseI2 * (1.0 / magnitude2) * rdotN2 * r2px;
                    double c10 = inverseI1 * (1.0 / magnitude1) * rdotN1 * r1py + inverseI2 * (1.0 / magnitude2) * rdotN2 * r2py;
                    double c11 = -(1.0 / magnitude1) * rdotN1 * parent.getAngularV() - (1.0 / magnitude2) * rdotN2 * connection.getAngularV();

                    //solve the system of linear equations
                    double D = c6 * c10 * c1 - c9 * c7 * c1 - c5 * c10 * c2 + c5 * c9 * c3;
                    double j = c6 * c10 * c4 - c9 * c7 * c4 - c8 * c10 * c2 + c11 * c7 * c2 + c8 * c9 * c3 - c6 * c11 * c3;
                    j /= D;
                    double jcX = c8 * c10 * c1 - c11 * c7 * c1 - c5 * c10 * c4 + c5 * c11 * c3;
                    jcX /= D;
                    double jcY = c6 * c11 * c1 - c9 * c8 * c1 - c5 * c11 * c2 + c5 * c9 * c4;
                    jcY /= D;

                    System.out.println("j: " + j + ", jcX: " + jcX + ", jcY: " + jcY);

                    //validate
                    double result1 = c1 * j + c2 * jcX + c3 * jcY;
                    result1 -= c4;
                    double result2 = c5 * j + c6 * jcX + c7 * jcY - c8;
                    double result3 = c9 * jcX + c10 * jcY - c11;

                    System.out.println("Errors: " + result1 + ", " + result2 + ", " + result3);
                    System.out.println();

                    if (c11 < 0.0) {
                        returnArray[3] += j * n1X + jcX;
                        returnArray[4] += j * n1Y + jcY;
                        returnArray[5] += jcX * r1px + jcY * r1py;
                    }


                }*/
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

        return returnArray;
    }
    public void makeSolid() {
        if (!isSolid) {
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
    public boolean isSolid() {
        return isSolid;
    }
}
