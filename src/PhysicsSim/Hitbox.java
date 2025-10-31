package PhysicsSim;

import java.awt.*;
import java.util.ArrayList;

public class Hitbox {
    private final Triplet whatAmI;
    protected final Rigidbody rigidbody;
    public final String name;

    protected Hitbox(Rigidbody input, String name) {
        whatAmI = new Triplet("Rigidbody", input.ID, input.simID);
        rigidbody = input;
        this.name = name;
    }

    public String getType() {
        return(whatAmI.getFirstString());
    }
    public int getIDinType() {
        return(whatAmI.getSecondInt());
    }
    public int getIDinSim() {
        return(Simulation.get(whatAmI.getThirdInt()).hitboxes.indexOf(this));
    }
    public void sleep() {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.simID = -1;
        }
    }
    public void wake() {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.simID = whatAmI.getThirdInt();
        }
    }
    public void setVisible(boolean visible) {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.draw = visible;
        }
    }

    public double[] getAcceleration() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getAX(), rigidbody.getAY()});
        }
        throw new Exception("Hitbox does not store acceleration.");
    }
    public double[] getVelocity() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getVX(), rigidbody.getVY()});
        }
        throw new Exception("Hitbox does not store velocity.");
    }
    public double[] getPosition() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getPosX(), rigidbody.getPosY()});
        }
        throw new Exception("Hitbox does not store position.");
    }
    public double[] getAngularMovement() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getAngularV(), rigidbody.getAngularA()});
        }
        throw new Exception("Hitbox does not store angular movement.");
    }
    public void setAcceleration(double[] acceleration) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setAX(acceleration[0]);
            rigidbody.setAY(acceleration[1]);
        } else {
            throw new Exception("Hitbox does not store acceleration.");
        }
    }
    public void setVelocity(double[] velocity) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setVX(velocity[0]);
            rigidbody.setVY(velocity[1]);
        } else {
            throw new Exception("Hitbox does not store velocity.");
        }
    }
    public void setPosition(double[] position) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setPosX(position[0]);
            rigidbody.setPosY(position[1]);
        } else {
            throw new Exception("Hitbox does not store position.");
        }
    }
    public void setAngularMovement(double[] angularMovement) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setAngularV(angularMovement[0]);
            rigidbody.setAngularA(angularMovement[1]);
        } else {
            throw new Exception("Hitbox does not store position.");
        }
    }
    public double[] getPositionsX() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            Triplet drawInformation = rigidbody.geometry.getDrawDoubles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            return switch (drawInformation.getFirstRigidbodyGeometries()) {
                case Polygon -> (drawInformation.getSecondDoubleArray());
                case Circle -> (new double[]{drawInformation.getSecondDoubleArray()[0]});
            };
        }
        throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getPositionsX()' is not applicable.");
    }
    public double[] getPositionsY() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            Triplet drawInformation = rigidbody.geometry.getDrawDoubles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            return switch (drawInformation.getFirstRigidbodyGeometries()) {
                case Polygon -> (drawInformation.getThirdDoubleArray());
                case Circle -> (new double[]{drawInformation.getSecondDoubleArray()[1]});
            };
        }
        throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getPositionsY()' is not applicable.");
    }
    public double getRadius() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) return(Math.sqrt(rigidbody.geometry.getLargestDistanceSquared()));
        throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getRadius()' is not applicable.");
    }
    public ArrayList<PhysicsObject> getObjectCollisions() throws Exception {
        ArrayList<PhysicsObject> returnArray = new ArrayList<>();
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                for (int i = 0; i < rigidbody.collidingIDs.size(); i = i + 1) {
                    if (rigidbody.collidingIDs.get(i) >= 0) {
                        if (Rigidbody.get(rigidbody.collidingIDs.get(i)).isHitbox) continue;
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Rigidbody", rigidbody.collidingIDs.get(i)));
                    }
                    else if (rigidbody.collidingIDs.get(i) <= -2) {
                        int index = -rigidbody.collidingIDs.get(i) - 2;
                        PhysicsObject parentSoftbody = Simulation.get(whatAmI.getThirdInt()).getObject("Softbody", index);
                        if (!returnArray.contains(parentSoftbody)) returnArray.add(parentSoftbody);
                    }
                }
                break;
            }
        }
        return(returnArray);
    }
    public ArrayList<Hitbox> getHitboxIntersections() throws Exception {
        ArrayList<Hitbox> returnArray = new ArrayList<>();
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            for (int i = 0; i < rigidbody.collidingIDs.size(); i = i + 1) {
                if (rigidbody.collidingIDs.get(i) >= 0) {
                    if (!Rigidbody.get(rigidbody.collidingIDs.get(i)).isHitbox) continue;
                    returnArray.add(Simulation.get(whatAmI.getThirdInt()).getHitbox("Rigidbody", rigidbody.collidingIDs.get(i)));
                }
            }
        }
        return(returnArray);
    }
}
