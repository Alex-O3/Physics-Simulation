package PhysicsSim;

import java.util.ArrayList;

public class Hitbox {
    private final Triplet whatAmI;
    protected final Rigidbody rigidbody;
    protected final Point point;
    protected final Softbody softbody;
    public final String name;

    protected Hitbox(Rigidbody input, String name) {
        whatAmI = new Triplet("Rigidbody", input.ID, input.simID);
        rigidbody = input;
        point = null;
        softbody = null;
        this.name = name;
    }
    protected Hitbox(Point input, String name) {
        whatAmI = new Triplet("Point", input.ID, input.simID);
        rigidbody = null;
        point = input;
        softbody = null;
        this.name = name;
    }

    public String getType() {
        return(whatAmI.getFirstString());
    }
    public int getIDinType() {
        return(whatAmI.getSecondInt());
    }
    public int getIDinSim() {
        return(Simulation.get(whatAmI.getThirdInt()).physicsObjects.indexOf(this));
    }
    public void sleep() {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.simID = -1;
                break;
            }
            case("Point"): {
                point.simID = -1;
                break;
            }
        }
    }
    public void wake() {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.simID = whatAmI.getThirdInt();
                break;
            }
            case("Point"): {
                point.simID = whatAmI.getThirdInt();
                break;
            }
        }
    }
    public void setVisible(boolean visible) {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.draw = visible;
                break;
            }
            case("Point"): {
                point.draw = visible;
                break;
            }
            case("Softbody"): {
                for (int i = 0; i < softbody.size(); i = i + 1) {
                    softbody.getMember(i).draw = visible;
                }
                break;
            }
        }
    }

    public double[] getAcceleration() throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): return(new double[]{rigidbody.getAX(), rigidbody.getAY()});
            case("Point"): return(new double[]{point.getAX(), point.getAY()});
            default:
                throw new Exception("Physics object (softbody likely) does not store acceleration.");
        }
    }
    public double[] getVelocity() throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): return(new double[]{rigidbody.getVX(), rigidbody.getVY()});
            case("Point"): return(new double[]{point.getVX(), point.getVY()});
            default:
                throw new Exception("Physics object (softbody likely) does not store velocity.");
        }
    }
    public double[] getPosition() throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): return(new double[]{rigidbody.getPosX(), rigidbody.getPosY()});
            case("Point"): return(new double[]{point.getX(), point.getY()});
            default:
                throw new Exception("Physics object (softbody likely) does not store position.");
        }
    }
    public double[] getAngularMovement() throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): return(new double[]{rigidbody.getAngularV(), rigidbody.getAngularA()});
            default:
                throw new Exception("Physics object (softbody or point likely) does not store angular movement.");
        }
    }
    public void setAcceleration(double[] acceleration) throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.aX = acceleration[0];
                rigidbody.aY = acceleration[1];
                break;
            }
            case("Point"): {
                point.aX = acceleration[0];
                point.aY = acceleration[1];
                break;
            }
            default:
                throw new Exception("Physics object (softbody likely) does not store acceleration.");
        }
    }
    public void setVelocity(double[] velocity) throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.vX = velocity[0];
                rigidbody.vY = velocity[1];
                break;
            }
            case("Point"): {
                point.vX = velocity[0];
                point.vY = velocity[1];
                break;
            }
            default:
                throw new Exception("Physics object (softbody likely) does not store velocity.");
        }
    }
    public void setPosition(double[] position) throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.posX = position[0];
                rigidbody.posY = position[1];
                break;
            }
            case("Point"): {
                point.posX = position[0];
                point.posY = position[1];
                break;
            }
            default:
                throw new Exception("Physics object (softbody likely) does not store position.");
        }
    }
    public void setAngularMovement(double[] angularMovement) throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.angularV = angularMovement[0];
                rigidbody.angularA = angularMovement[1];
                break;
            }
            default:
                throw new Exception("Physics object (softbody likely) does not store position.");
        }
    }
    public ArrayList<Double> getXPoints() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) return(rigidbody.getXPoints());
        throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getXPoints()' is not applicable.");
    }
    public ArrayList<Double> getYPoints() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) return(rigidbody.getYPoints());
        throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getXPoints()' is not applicable.");
    }
    public double getRadius() throws Exception {
        if (whatAmI.getFirstString().equals("Point")) return(point.getRadius());
        throw new Exception("Accessed physicsObject is not a 'Point', so 'getRadius()' is not applicable.");
    }
    public ArrayList<Integer> getSpringAttachments() throws Exception {
        if (whatAmI.getFirstString().equals("Point")) return(point.getAttachments());
        throw new Exception("Accessed physicsObject is not a 'Point', so 'getSpringAttachments()' is not applicable. Attached indices are local in type of static Point.");
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
                        int parent = Point.get(index).parentSoftbody;
                        if (Point.get(index).isHitbox) continue;
                        if (parent != -1) {
                            PhysicsObject parentSoftbody = Simulation.get(whatAmI.getThirdInt()).getObject("Softbody", parent);
                            if (!returnArray.contains(parentSoftbody)) returnArray.add(parentSoftbody);
                        }
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Point", index)); }
                }
                break;
            }
            case("Point"): {
                for (int i = 0; i < point.collidingIDs.size(); i = i + 1) {
                    if (point.collidingIDs.get(i) >= 0) {
                        if (Rigidbody.get(point.collidingIDs.get(i)).isHitbox) continue;
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Rigidbody", point.collidingIDs.get(i)));
                    }
                    else if (point.collidingIDs.get(i) <= -2) {
                        int index = -point.collidingIDs.get(i) - 2;
                        int parent = Point.get(index).parentSoftbody;
                        if (Point.get(index).isHitbox) continue;
                        if (parent != -1) {
                            PhysicsObject parentSoftbody = Simulation.get(whatAmI.getThirdInt()).getObject("Softbody", parent);
                            if (!returnArray.contains(parentSoftbody)) returnArray.add(parentSoftbody);
                        }
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Point", index));}
                }
                break;
            }
        }
        return(returnArray);
    }
    public ArrayList<Hitbox> getHitboxIntersections() throws Exception {
        ArrayList<Hitbox> returnArray = new ArrayList<>();
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                for (int i = 0; i < rigidbody.collidingIDs.size(); i = i + 1) {
                    if (rigidbody.collidingIDs.get(i) >= 0) {
                        if (!Rigidbody.get(rigidbody.collidingIDs.get(i)).isHitbox) continue;
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getHitbox("Rigidbody", rigidbody.collidingIDs.get(i)));
                    }
                    else if (rigidbody.collidingIDs.get(i) <= -2) {
                        if (!Point.get(-rigidbody.collidingIDs.get(i) - 2).isHitbox) continue;
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getHitbox("Point", -rigidbody.collidingIDs.get(i) - 2));
                    }
                }
                break;
            }
            case("Point"): {
                for (int i = 0; i < point.collidingIDs.size(); i = i + 1) {
                    if (point.collidingIDs.get(i) >= 0) {
                        if (!Rigidbody.get(point.collidingIDs.get(i)).isHitbox) continue;
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getHitbox("Rigidbody", point.collidingIDs.get(i)));
                    }
                    else if (point.collidingIDs.get(i) <= -2) {
                        if (!Point.get(-point.collidingIDs.get(i) - 2).isHitbox) continue;
                        returnArray.add(Simulation.get(whatAmI.getThirdInt()).getHitbox("Point", -point.collidingIDs.get(i) - 2));
                    }
                }
                break;
            }
        }
        return(returnArray);
    }
}
