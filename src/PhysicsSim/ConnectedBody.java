package PhysicsSim;

import java.util.ArrayList;

class ConnectedBody {
    final ArrayList<Rigidbody> members = new ArrayList<>();
    public ConnectedBody(Rigidbody firstMember) {
        expand(firstMember);
    }
    private void expand(Rigidbody nextMember) {
        if (!members.contains(nextMember)) {
            members.add(nextMember);
            nextMember.connectedBody = this;
            for (Joint joint : nextMember.attachments) {
                if (joint.movesWithConnectedBody() && !members.contains(joint.connection)) {
                    expand(joint.connection);
                }
            }
        }
    }

    public void changePosition(double x, double y) {
        for (Rigidbody member : members) {
            member.setPosX(member.getPosX() + x);
            member.setPosY(member.getPosY() + y);
        }
    }
}
