package PhysicsSim;

import java.util.ArrayList;

class CompoundBody {
    final ArrayList<Rigidbody> members = new ArrayList<>();
    double inertia;
    double mass;
    double[] cM;
    boolean movable = true;
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
    public boolean isCompoundAttachment(Joint joint) {
        return joint.type == JointType.Weld || joint.type == JointType.Pin || joint.type == JointType.Revolute;
    }

    private void calculateProperties() {
        cM = new double[2];
        inertia = 0.0;
        mass = 0.0;
        for (Rigidbody member : members) {
            mass += member.getMass();
            cM[0] += member.getPosX() * member.getMass();
            cM[1] += member.getPosY() * member.getMass();
            if (!member.isMovable()) movable = false;
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
        totalAngularVelocity /= inertia;
        totalAngularAcceleration /= inertia;

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
}
