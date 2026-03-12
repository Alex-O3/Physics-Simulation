package PhysicsSim;

import java.util.ArrayList;

class CompoundBody {
    final ArrayList<Rigidbody> members = new ArrayList<>();
    double inertia;
    double mass;
    double[] cM;
    boolean movable = true;
    boolean lockedRotation = false;
    public CompoundBody(Rigidbody firstMember) {
        expand(firstMember);
    }
    private void expand(Rigidbody nextMember) {
        if (!members.contains(nextMember)) {
            members.add(nextMember);
            nextMember.compoundBody = this;
            for (Joint weld : nextMember.attachments) {
                if (weld.type == JointType.Weld && !members.contains(weld.connection)) {
                    expand(weld.connection);
                }
            }
        }
    }

    private void calculateProperties() {
        cM = new double[2];
        inertia = 0.0;
        mass = 0.0;
        movable = true;
        lockedRotation = false;
        for (Rigidbody member : members) {
            mass += member.getMass();
            cM[0] += member.getPosX() * member.getMass();
            cM[1] += member.getPosY() * member.getMass();
            if (!member.isMovable()) movable = false;
            if (member.lockedRotation()) lockedRotation = true;
        }
        cM[0] /= mass;
        cM[1] /= mass;

        inertia = 0.0;
        for (Rigidbody member : members) {
            double squaredDistance = ((cM[0] - member.getPosX()) * (cM[0] - member.getPosX()) + (cM[1] - member.getPosY()) * (cM[1] - member.getPosY()));
            inertia += member.getMass() * squaredDistance + member.getInertia();
        }
    }

    public void equalizeProperties() {
        calculateProperties();

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

        for (Rigidbody member : members) {
            member.newangularV = totalAngularVelocity;
            member.newangularA = totalAngularAcceleration;
            double[] v = new double[]{totalAngularVelocity * -(member.getPosY() - cM[1]), totalAngularVelocity * (member.getPosX() - cM[0])};
            double[] a = new double[]{totalAngularAcceleration * -(member.getPosY() - cM[1]), totalAngularAcceleration * (member.getPosX() - cM[0])};

            member.newvX = totalLinearVelocity[0] + v[0];
            member.newvY = totalLinearVelocity[1] + v[1];
            member.newaX = totalLinearAcceleration[0] + a[0];
            member.newaY = totalLinearAcceleration[1] + a[1];
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
}
