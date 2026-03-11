package PhysicsSim;

import java.awt.*;
import java.util.ArrayList;

/**
 * Hitbox acts as a wrapper facade class for rigidbodies set to not collide with but still store all collision information booleans. While
 * values such as velocity and position may be accessed, others like boundary color or mass may not as this class does not store those values. For ease of access, hitboxes
 * may be assigned names for reference, though duplicate names invalidate all previous identical name references.
 */
public class Hitbox {
    private final Triplet whatAmI;
    protected final Rigidbody rigidbody;
    public final String name;


    protected Hitbox(Rigidbody input, String name) {
        whatAmI = new Triplet("Rigidbody", input.ID, input.simID);
        rigidbody = input;
        this.name = name;
    }

    /**
     *
     * @return String describing the hitbox stored by this instance. Currently, this only returns "Rigidbody".
     */
    public String getType() {
        return(whatAmI.getFirstString());
    }
    /**
     *
     * @return String describing the geometry represented by this hitbox.
     */
    public String getGeometryType() {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.geometry instanceof Polygon) return "Polygon";
            if (rigidbody.geometry instanceof Circle) return "Circle";
            else return "Unknown";
        }
        else if (whatAmI.getFirstString().equals("Softbody")) {
            return "Softbody";
        }
        return "Unknown";
    }
    /**
     * Sets the simulation ID of this hitbox to -1 such that it cannot interact with any hitboxes or objects in any simulations.
     */
    public void sleep() {
        rigidbody.simID = -1;
    }
    /**
     * Sets the simulation ID of this hitbox to what it was instantiated as having.
     */
    public void wake() {
        rigidbody.simID = whatAmI.getThirdInt();
    }
    /**
     *
     * @boolean visible, whether this object can be drawn in the native JFrame display.
     */
    public void setVisible(boolean visible) {
        rigidbody.draw = visible;
    }

    /**
     *
     * @return double[2] acceleration vector for this hitbox's center of mass.
     * @throws Exception if this object cannot store acceleration.
     */
    public double[] getAcceleration() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getAX(), rigidbody.getAY()});
        }
        throw new Exception("Hitbox does not store acceleration.");
    }
    /**
     *
     * @return double[2] velocity vector for this hitbox's center of mass.
     * @throws Exception if this object cannot store velocity.
     */
    public double[] getVelocity() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getVX(), rigidbody.getVY()});
        }
        throw new Exception("Hitbox does not store velocity.");
    }
    /**
     *
     * @return double[2] position vector from origin for this hitbox's center of mass.
     * @throws Exception if this object cannot store position.
     */
    public double[] getPosition() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getPosX(), rigidbody.getPosY()});
        }
        throw new Exception("Hitbox does not store position.");
    }
    /**
     *
     * @return double[2]{angularVelocity, angularAcceleration} of this rigidbody.
     * @throws Exception if this hitbox does not store angular movement values.
     */
    public double[] getAngularMovement() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getAngularV(), rigidbody.getAngularA()});
        }
        throw new Exception("Hitbox does not store angular movement.");
    }
    /**
     *
     * @param acceleration double[2] vector to set this hitbox's initial acceleration vector at the center of mass. This does not apply a momentary acceleration, but rather a constant acceleration until changed again.
     * @throws Exception if this hitbox does not store initial acceleration.
     */
    public void setAcceleration(double[] acceleration) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setCompoundAInitialForce(acceleration[0], acceleration[1]);
        }
        else {
            throw new Exception("Hitbox does not store acceleration.");
        }
    }
    /**
     *
     * @param velocity double[2] vector to set this hitbox's velocity vector at the center of mass.
     * @throws Exception if this hitbox does not store velocity.
     */
    public void setVelocity(double[] velocity) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setVX(velocity[0]);
            rigidbody.setVY(velocity[1]);
        } else {
            throw new Exception("Hitbox does not store velocity.");
        }
    }
    /**
     *
     * @param position double[2] vector to set this hitbox's position vector from the origin at the center of mass.
     * @throws Exception if this hitbox does not store position.
     */
    public void setPosition(double[] position) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setPosX(position[0]);
            rigidbody.setPosY(position[1]);
        } else {
            throw new Exception("Hitbox does not store position.");
        }
    }
    /**
     *
     * @param angularVelocity double angular velocity of this hitbox around its center of mass.
     * @throws Exception if this hitbox does not store angular velocity.
     */
    public void setAngularMovement(double angularVelocity) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setAngularV(angularVelocity);
        } else {
            throw new Exception("Hitbox does not store angular movement.");
        }
    }
    /**
     * Retrieves data regarding positional information necessary to draw this object.
     * @return double[n] of x positions of polygon points (if polygon) or double[1] of the circle's x center.
     * @throws Exception if this object is not a rigidbody with a geometry.
     */
    public double[] getPositionsX() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            Triplet drawInformation = rigidbody.geometry.getDrawDoubles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            return switch (drawInformation.getFirstRigidbodyGeometries()) {
                case Polygon -> (drawInformation.getSecondDoubleArray());
                case Circle -> (new double[]{drawInformation.getSecondDoubleArray()[0]});
            };
        }
        throw new Exception("Accessed hitbox is not a 'Rigidbody', so 'getPositionsX()' is not applicable.");
    }
    /**
     * Retrieves data regarding positional information necessary to draw this object.
     * @return double[n] of y positions of polygon points (if polygon) or double[1] of the circle's y center.
     * @throws Exception if this object is not a rigidbody with a geometry.
     */
    public double[] getPositionsY() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            Triplet drawInformation = rigidbody.geometry.getDrawDoubles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            return switch (drawInformation.getFirstRigidbodyGeometries()) {
                case Polygon -> (drawInformation.getThirdDoubleArray());
                case Circle -> (new double[]{drawInformation.getSecondDoubleArray()[1]});
            };
        }
        throw new Exception("Accessed hitbox is not a 'Rigidbody', so 'getPositionsY()' is not applicable.");
    }
    /**
     * Retrieves the radius of this hitbox, defined as the longest distance between its center of mass and any member position.
     * @return the largest radius.
     * @throws Exception if this hitbox does not store radius.
     */
    public double getRadius() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) return(rigidbody.geometry.getLargestDistance());
        throw new Exception("Accessed hitbox is not a 'Rigidbody', so 'getRadius()' is not applicable.");
    }
    /**
     * Retrieves a list of all physicsObjects this object is presently colliding with. This list retains an entry for every collision point, allowing duplicate physics objects to be in the list. Collisions with solid joints are not considered.
     * @return ArrayList, element type PhysicsObject, of all object collisions.
     * @throws Exception if a hitbox improperly stores collisions.
     */
    public ArrayList<PhysicsObject> getObjectCollisions() throws Exception {
        ArrayList<PhysicsObject> returnArray = new ArrayList<>();
        for (int i = 0; i < rigidbody.collidingIDs.size(); i = i + 1) {
            if (rigidbody.collidingIDs.get(i) >= 0) {
                if (Rigidbody.get(rigidbody.collidingIDs.get(i)).isHitbox) continue;
                returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Rigidbody", rigidbody.collidingIDs.get(i)));
            }
        }
        return(returnArray);
    }
    /**
     * Retrieves a list of all hitboxes this hitbox is presently intersecting without any duplicates.
     * @return ArrayList, element type Hitbox, of all hitbox intersection.
     * @throws Exception if a hitbox improperly stores collisions.
     */
    public ArrayList<Hitbox> getHitboxIntersections() throws Exception {
        ArrayList<Hitbox> returnArray = new ArrayList<>();
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            for (int i = 0; i < rigidbody.collidingIDs.size(); i = i + 1) {
                if (rigidbody.collidingIDs.get(i) >= 0) {
                    if (!Rigidbody.get(rigidbody.collidingIDs.get(i)).isHitbox) continue;
                    Hitbox entry = Simulation.get(whatAmI.getThirdInt()).getHitbox("Rigidbody", rigidbody.collidingIDs.get(i));
                    if (returnArray.contains(entry)) continue;
                    returnArray.add(entry);
                }
            }
        }
        return(returnArray);
    }

    /**
     * Converts this hitbox into a description in the form of a String.
     * @return String
     */
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder();
        result.append(getGeometryType()).append(" ").append(rigidbody.ID).append(" ").append(name).append("\n");
        switch (getGeometryType()) {
            case "Polygon": {
                Polygon polygon = (Polygon) rigidbody.geometry;
                result.append("Points: ");
                double x = rigidbody.getPosX();
                double y = rigidbody.getPosY();
                for (int j = 0; j < polygon.getNumPoints(); j++) {
                    result.append("{").append(polygon.getXPoints(j) + x).append(", ").append(polygon.getYPoints(j) + y).append("}");
                    if (j < polygon.getNumPoints() - 1) result.append(",  ");
                }
                break;
            }
            case "Circle": {
                result.append("Center: {").append(rigidbody.getPosX()).append(", ").append(rigidbody.getPosY()).append("}, Radius: ").append(rigidbody.geometry.getLargestDistance());
                break;
            }
            default: {
                result.append("Unknown");
                break;
            }
        }
        return result.toString();
    }
}
