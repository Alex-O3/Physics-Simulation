package PhysicsSim;

import java.util.ArrayList;

class CompoundBody {
    final ArrayList<Rigidbody> members = new ArrayList<>();
    double inertia;
    double mass;
    double[] cM;
    private final double[] memberOffsets;
    boolean movable = true;
    boolean lockedRotation = false;
    private double[] controllerJerk = new double[2];
    double initialExternalAngularA = 0.0;

    public CompoundBody(Rigidbody firstMember) {
        ArrayList<double[]> memberOffsetConstruction = new ArrayList<>();
        memberOffsetConstruction.add(new double[]{firstMember.getPosX(), firstMember.getPosY()});
        expand(firstMember, memberOffsetConstruction, 0);
        for (int i = 0; i < members.size(); i++) {
            members.get(i).newposX = memberOffsetConstruction.get(i)[0];
            members.get(i).setPosX(memberOffsetConstruction.get(i)[0]);
            members.get(i).newposY = memberOffsetConstruction.get(i)[1];
            members.get(i).setPosY(memberOffsetConstruction.get(i)[1]);
        }
        memberOffsets = new double[members.size() * 2];
        calculateProperties();
        for (int i = 0; i < memberOffsetConstruction.size(); i++) {
            memberOffsets[2 * i] = memberOffsetConstruction.get(i)[0] - cM[0];
            memberOffsets[2 * i + 1] = memberOffsetConstruction.get(i)[1] - cM[1];
        }
    }
    private void expand(Rigidbody nextMember, ArrayList<double[]> memberOffsetConstruction, int parentMemberID) {
        members.add(nextMember);
        nextMember.compoundBody = this;
        for (Joint weld : nextMember.attachments) {
            if (weld.type == JointType.Weld && !members.contains(weld.connection)) {
                memberOffsetConstruction.add(new double[]{memberOffsetConstruction.get(parentMemberID)[0] + weld.offsetFromCMParent[0] - weld.offsetFromCMOther[0], memberOffsetConstruction.get(parentMemberID)[1] + weld.offsetFromCMParent[1] - weld.offsetFromCMOther[1]});
                expand(weld.connection, memberOffsetConstruction, members.size());
            }
        }
    }

    public void calculateProperties() {
        cM = new double[2];
        inertia = 0.0;
        mass = 0.0;
        movable = true;
        lockedRotation = false;
        for (Rigidbody member : members) {
            mass += member.getMass();
            cM[0] += member.newposX * member.getMass();
            cM[1] += member.newposY * member.getMass();
            if (!member.isMovable) movable = false;
            if (member.lockedRotation) lockedRotation = true;
        }
        cM[0] /= mass;
        cM[1] /= mass;

        inertia = 0.0;
        for (Rigidbody member : members) {
            double squaredDistance = ((cM[0] - member.newposX) * (cM[0] - member.newposX) + (cM[1] - member.newposY) * (cM[1] - member.newposY));
            inertia += member.getMass() * squaredDistance + member.getInertia();
        }
    }

    public void equalizeProperties() {
        double totalAngularVelocity = 0.0;
        double totalAngularAcceleration = 0.0;
        double[] totalLinearVelocity = new double[2];
        double[] totalLinearAcceleration = new double[2];
        for (Rigidbody member : members) {
            totalLinearVelocity[0] += member.getMass() * member.newvX;
            totalLinearVelocity[1] += member.getMass() * member.newvY;

            totalLinearAcceleration[0] += member.getMass() * member.newaX;
            totalLinearAcceleration[1] += member.getMass() * member.newaY;
        }
        totalLinearVelocity[0] /= mass;
        totalLinearVelocity[1] /= mass;
        totalLinearAcceleration[0] /= mass;
        totalLinearAcceleration[1] /= mass;
        for (Rigidbody member : members) {
            totalAngularVelocity += member.getMass() * (-(member.newposY - cM[1]) * (member.newvX - totalLinearVelocity[0]) + (member.newposX - cM[0]) * (member.newvY - totalLinearVelocity[1]));
            totalAngularVelocity += member.getInertia() * member.newangularV;

            totalAngularAcceleration += member.getMass() * (-(member.newposY - cM[1]) * (member.newaX - totalLinearAcceleration[0]) + (member.newposX - cM[0]) * (member.newaY - totalLinearAcceleration[1]));
            totalAngularAcceleration += member.getInertia() * member.newangularA;
        }
        if (!lockedRotation) {
            totalAngularVelocity /= inertia;
            totalAngularAcceleration /= inertia;
        }
        else {
            totalAngularVelocity = 0.0;
            totalAngularAcceleration = 0.0;
        }
        totalAngularAcceleration += initialExternalAngularA;

        for (Rigidbody member : members) {
            member.newangularV = totalAngularVelocity;
            member.newangularA = totalAngularAcceleration;
            double[] v = new double[]{totalAngularVelocity * -(member.newposY - cM[1]), totalAngularVelocity * (member.newposX - cM[0])};
            double[] a = new double[]{totalAngularAcceleration * -(member.newposY - cM[1]), totalAngularAcceleration * (member.newposX - cM[0])};

            member.newvX = totalLinearVelocity[0] + v[0];
            member.newvY = totalLinearVelocity[1] + v[1];
            member.newaX = totalLinearAcceleration[0] + a[0] + controllerJerk[0];
            member.newaY = totalLinearAcceleration[1] + a[1] + controllerJerk[1];
        }
        controllerJerk[0] = 0.0;
        controllerJerk[1] = 0.0;
    }
    public void spaceProperly() {
        for (int i = 0; i < members.size(); i++) {
            Rigidbody member = members.get(i);
            member.newposX = cM[0] + memberOffsets[2 * i];
            member.newposY = cM[1] + memberOffsets[2 * i + 1];
        }
    }
    public void rotateAroundCenter(double theta) {
        if (!lockedRotation) {
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            for (int i = 0; i < members.size(); i++) {
                double x = memberOffsets[2 * i];
                double y = memberOffsets[2 * i + 1];
                memberOffsets[2 * i] = x * cos - y * sin;
                memberOffsets[2 * i + 1] = x * sin + y * cos;
            }
        }
    }

    public void setPosition(double x, double y) {
        double[] newCM = new double[]{x, y};
        for (Rigidbody member : members) {
            double offsetX = member.getPosX() - cM[0];
            double offsetY = member.getPosY() - cM[1];
            member.setPosX(newCM[0] + offsetX);
            member.setPosY(newCM[1] + offsetY);
            member.newposX = member.getPosX();
            member.newposY = member.getPosY();
        }
        cM[0] = newCM[0];
        cM[1] = newCM[1];
    }
    public void changePosition(double x, double y) {
        setPosition(cM[0] + x, cM[1] + y);
    }
    public void setVelocity(double vx, double vy) {
        for (Rigidbody member : members) {
            member.setVX(vx);
            member.setVY(vy);
        }
    }
    public double getVX() {
        Rigidbody firstMember = members.getFirst();
        return firstMember.getVX() + firstMember.getAngularV() * -(cM[1] - firstMember.getPosY());
    }
    public double getVY() {
        Rigidbody firstMember = members.getFirst();
        return firstMember.getVY() + firstMember.getAngularV() * (cM[0] - firstMember.getPosX());
    }
    public void setAcceleration(double ax, double ay) {
        for (Rigidbody member : members) {
            member.setAX(ax);
            member.setAY(ay);
        }
    }
    public double getAX() {
        Rigidbody firstMember = members.getFirst();
        return firstMember.getAX() + firstMember.getAngularA() * -(cM[1] - firstMember.getPosY());
    }
    public double getAY() {
        Rigidbody firstMember = members.getFirst();
        return firstMember.getAY() + firstMember.getAngularA() * (cM[0] - firstMember.getPosX());
    }
    public double getAngularV() {
        Rigidbody firstMember = members.getFirst();
        return firstMember.getAngularV();
    }
    public void setAngularV(double angularV) {
        double[] totalLinearVelocity = new double[]{getVX(), getVY()};
        for (Rigidbody member : members) {
            member.newangularV = angularV;
            double[] v = new double[]{angularV * -(member.getPosY() - cM[1]), angularV * (member.getPosX() - cM[0])};

            member.newvX = totalLinearVelocity[0] + v[0];
            member.newvY = totalLinearVelocity[1] + v[1];
        }
    }
    public double getAngularA() {
        Rigidbody firstMember = members.getFirst();
        return firstMember.getAngularA();
    }
    public void setAngularA(double angularA) {
        double[] totalLinearAcceleration = new double[]{getAX(), getAY()};
        for (Rigidbody member : members) {
            member.newangularA = angularA;
            double[] a = new double[]{angularA * -(member.getPosY() - cM[1]), angularA * (member.getPosX() - cM[0])};

            member.newaX = totalLinearAcceleration[0] + a[0];
            member.newaY = totalLinearAcceleration[1] + a[1];
        }
    }
    public void addControllerJerk(double[] jerk) {
        controllerJerk[0] += jerk[0];
        controllerJerk[1] += jerk[1];
    }
}
