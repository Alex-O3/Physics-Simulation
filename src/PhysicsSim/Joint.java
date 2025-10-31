package PhysicsSim;

class Joint {
    final Rigidbody parent;
    final Rigidbody connection;
    final double[] offsetFromCMParent;
    final double[] offsetFromCMOther;
    double SPRING_CONSTANT;
    double SPRING_DAMPING;
    private final double idealDistance;

    double maxDistMultiplier;
    double minDistMultiplier;
    final JointType type;
    //joint force calculations come in a double[] of length 3: Fx, Fy, and torque
    public Joint(Rigidbody parent, Rigidbody connection, double[] otherOffset, double[] thisOffset, double idealDistance, double SPRING_CONSTANT, double SPRING_DAMPING, double maxDistMultiplier) {
        this.parent = parent;
        this.connection = connection;
        offsetFromCMParent = thisOffset;
        offsetFromCMOther = otherOffset;
        this.idealDistance = idealDistance;
        this.SPRING_CONSTANT = SPRING_CONSTANT;
        this.SPRING_DAMPING = SPRING_DAMPING;
        this.maxDistMultiplier = maxDistMultiplier;
        type = JointType.Spring;
    }
    public double[] calculateSpringForce() {
        double dx = (connection.getPosX() + offsetFromCMOther[0]) - (parent.getPosX() + offsetFromCMParent[0]);
        double dy = (connection.getPosY() + offsetFromCMOther[1]) - (parent.getPosY() + offsetFromCMParent[1]);
        double distance = Math.sqrt(dx * dx + dy * dy);
        double nX = dx / distance;
        double nY = dy / distance;
        if (Double.isNaN(nX) || Double.isNaN(nY)) {
            nX = 0.0;
            nY = 0.0;
        }
        double[] fS = new double[2];
        fS[0] = SPRING_CONSTANT * (dx - idealDistance * nX);
        fS[1] = SPRING_CONSTANT * (dy - idealDistance * nY);
        double multiplier = SPRING_CONSTANT * maxDistMultiplier * idealDistance;
        double[] maxF = new double[]{multiplier * nX, multiplier * nY};
        fS[0] = Math.min(Math.abs(fS[0]), Math.abs(maxF[0])) * Math.signum(fS[0]);
        fS[1] = Math.min(Math.abs(fS[1]), Math.abs(maxF[1])) * Math.signum(fS[1]);
        multiplier = ((parent.getVX() - connection.getVX()) * nX + (parent.getVY() - connection.getVY()) * nY) * SPRING_DAMPING;
        double[] fD = new double[]{-nX * multiplier, -nY * multiplier};
        if (fD[0] * fD[0] + fD[1] * fD[1] < fS[0] * fS[0] + fS[1] * fS[1]) {
            double crossProduct = -offsetFromCMParent[1] * (fS[0] + fD[0]) + offsetFromCMParent[0] * (fS[1] + fD[1]);
            return(new double[]{fS[0] + fD[0], fS[1] + fD[1], crossProduct});
        }
        return(new double[]{0.0, 0.0, 0.0});

    }
}
