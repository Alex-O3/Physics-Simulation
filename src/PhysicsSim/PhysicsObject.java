package PhysicsSim;

import java.awt.*;
import java.util.ArrayList;

/**
 * PhysicsObject acts as a wrapper facade class for rigidbodies or softbodies. Through this class, values such as the color, velocity, acceleration, and position
 * can be set and retrieved, in addition to the bodies this PhysicsObject is colliding with. Additionally, through methods available to this class,
 * the joints connecting rigidibodies and the material- controlling density, friction, and restitution- of this object can be set. PhysicsObjects can be members
 * of softbodies, of compound bodies, and of connected bodies. Though softbodies can be accessed with get methods, compound bodies and connected bodies cannot be accessed,
 * except implicit access by changing position and velocity. Compound bodies contain all rigidbodies connected by weld joints while connected bodies
 * contain all bodies connected by constraint-based joints.<br>
 */
public class PhysicsObject {
     private final Triplet whatAmI;
     protected final Rigidbody rigidbody;
     protected final Softbody softbody;
     protected Material material;

     protected PhysicsObject(Rigidbody input) {
        whatAmI = new Triplet("Rigidbody", input.ID, input.simID);
        rigidbody = input;
        softbody = null;
     }
     protected PhysicsObject(Softbody input) {
         whatAmI = new Triplet("Softbody", input.ID, input.simID);
         rigidbody = null;
         softbody = input;
     }

    /**
     *
     * @return String describing the physicsObject stored by this instance.
     */
     public String getType() {
         return(whatAmI.getFirstString());
     }

    /**
     *
     * @return String describing the geometry represented by this physicsObject.
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
     * Sets the simulation ID of this physicsObject to -1 such that it cannot interact with any objects in any simulations.
     */
    public void sleep() {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 rigidbody.simID = -1;
                 break;
             }
             case("Softbody"): {
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     softbody.getMember(i).simID = -1;
                 }
                 softbody.simID = -1;
                 break;
             }
         }
     }

    /**
     * Sets the simulation ID of this physicsObject to what it was instantiated as having.
     */
    public void wake() {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 rigidbody.simID = whatAmI.getThirdInt();
                 break;
             }
             case("Softbody"): {
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     softbody.getMember(i).simID = whatAmI.getThirdInt();
                 }
                 softbody.simID = whatAmI.getThirdInt();
                 break;
             }
         }
     }

    /**
     *
     * @boolean visible, whether this object can be drawn in the native JFrame display.
     */
     public void setVisible(boolean visible) {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.draw = visible;
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

    /**
     *
     * @return double[2] acceleration vector for this object. If it is part of a compound body, this method returns the acceleration of the compound body's center of mass.
     * @throws Exception if this object cannot store acceleration.
     */
    public double[] getAcceleration() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return new double[]{rigidbody.compoundBody.getAX(), rigidbody.compoundBody.getAY()};
            else return (new double[]{rigidbody.getAX(), rigidbody.getAY()});
        }
        throw new Exception("Physics object (softbody likely) does not store acceleration.");
    }

    /**
     *
     * @return double[2] velocity vector for this object. If it is part of a compound body, this method returns the velocity of the compound body's center of mass.
     * @throws Exception if this object cannot store a single velocity.
     */
    public double[] getVelocity() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return new double[]{rigidbody.compoundBody.getVX(), rigidbody.compoundBody.getVY()};
            else return (new double[]{rigidbody.getVX(), rigidbody.getVY()});
        }
        throw new Exception("Physics object (softbody likely) does not store velocity.");
    }

    /**
     *
     * @return double[2] position vector from the origin for this object. If it is part of a compound body, this method returns the position of the compound body's center of mass.
     * @throws Exception if this object cannot store a single position.
     */
    public double[] getPosition() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return new double[]{rigidbody.compoundBody.cM[0], rigidbody.compoundBody.cM[1]};
            else return (new double[]{rigidbody.getPosX(), rigidbody.getPosY()});
        }
        throw new Exception("Physics object (softbody likely) does not store position.");
    }

    /**
     *
     * @return double mass of this object, or its compound body's mass if applicable.
     * Obstacles have finite mass, but they are treated as infinite mass in calculations, excepting universal gravity.
     * @throws Exception if this object cannot store mass.
     */
    public double getMass() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return rigidbody.getCompoundMass();
            else return (rigidbody.getMass());
        }
        else if (whatAmI.getFirstString().equals("Softbody")) {
            double mass = 0.0;
            for (int i = 0; i < softbody.size(); i++) {
                mass += softbody.members.get(i).getCompoundMass();
            }
            return(mass);
        }
        throw new Exception("Physics object does not store mass.");
    }

    /**
     *
     * @return double moment of inertia, or its compound body's moment of inertia if applicable. The result received is in reference to the given body's center of mass.
     * Obstacles have finite moments of inertia, but they are treated as infinite moments in calculations.
     * @throws Exception if this object cannot store inertia (i.e. softbody).
     */
    public double getInertia() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return rigidbody.compoundBody.inertia;
            else return (rigidbody.getInertia());
        }
        throw new Exception("Physics object (softbody likely) does not store moment of inertia.");
    }

    /**
     *
     * @return boolean whether it is an obstacle body.
     * @throws Exception if this object cannot be categorized as an obstacle or not as an obstacle.
     */
    public boolean isObstacle() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return !rigidbody.compoundBody.movable;
            else return !rigidbody.isMovable();
        }
        else if (whatAmI.getFirstString().equals("Softbody")) {
            return false;
        }
        throw new Exception("Physics object cannot be an obstacle, nor can it not be an obstacle.");
    }


    /**
     *
     * @return double[2]{angularVelocity, angularAcceleration} of this rigidbody, or its compound body if applicable.
     * @throws Exception if this object does not store angular movement values (i.e. softbody).
     */
    public double[] getAngularMovement() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            if (rigidbody.compoundBody != null) return new double[]{rigidbody.compoundBody.getAngularV(), rigidbody.compoundBody.getAngularA()};
            else return (new double[]{rigidbody.getAngularV(), rigidbody.getAngularA()});
        }
        throw new Exception("Physics object (softbody likely) does not store angular movement.");
    }

    /**
     *
     * @param acceleration double[2] vector to set this object's, or its compound body's if applicable, initial acceleration vector. This does not apply a momentary acceleration, but rather a constant acceleration until changed again.
     * @throws Exception if this object does not store initial acceleration.
     */
    public void setAcceleration(double[] acceleration) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setCompoundAInitialForce(acceleration[0], acceleration[1]);
        }
        else if (whatAmI.getFirstString().equals("Softbody")) {
            for (Rigidbody member : softbody.members) {
                member.setCompoundAInitialForce(acceleration[0], acceleration[1]);
            }
        }
        else {
            throw new Exception("Physics object does not store acceleration.");
        }
    }

    /**
     *
     * @param velocity double[2] vector to set this object's, or its compound body's if applicable, velocity vector.
     * @throws Exception if this object (i.e. softbody) does not store velocity.
     */
    public void setVelocity(double[] velocity) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setCompoundV(velocity[0], velocity[1]);
        } else {
            throw new Exception("Physics object (softbody likely) does not store velocity.");
        }
    }

    /**
     *
     * @param position double[2] position vector from origin to set this object's, or its connected body's, position. Note that a connected body is any body connected by position-based constraint joints (like pin or weld joints). Connected bodies include compound bodies if applicable. Applies to softbodies as well.
     * @throws Exception if this object does not store any form of position.
     */
    public void setPosition(double[] position) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setConnectedBodyPos(position[0], position[1]);
        }
        else if (whatAmI.getFirstString().equals("Softbody")) {
            double offsetX = position[0] - softbody.cM[0];
            double offsetY = position[1] - softbody.cM[1];
            for (Rigidbody member : softbody.members) {
                member.setConnectedBodyPos(member.getPosX() + offsetX, member.getPosY() + offsetY);
            }
        }
        else {
            throw new Exception("Physics object does not store position.");
        }
    }

    /**
     *
     * @param angularV double angular velocity of this object around its, or its compound body's if applicable, center of mass.
     * @throws Exception if this object does not store angular velocity (i.e. softbody).
     */
    public void setAngularVelocity(double angularV) throws Exception {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 rigidbody.setCompoundAngularV(angularV);
                 break;
             }
             default:
                 throw new Exception("Physics object (softbody likely) does not store angular movement.");
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
         throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getPositionsX()' is not applicable.");
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
        throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getPositionsY()' is not applicable.");
    }

    /**
     * Retrieves the radius of this object, defined as the longest distance between its center of mass and any member position.
     * @return the largest radius.
     * @throws Exception if this object does not store radius.
     */
    public double getRadius() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) return(rigidbody.geometry.getLargestDistance());
        else if (whatAmI.getFirstString().equals("Softbody")) return (softbody.getRadius());
         throw new Exception("Accessed physicsObject is not applicable to 'getRadius()'.");
    }

    /**
     * Sets the color of boundary members of this softbody. If this object is not a softbody, nothing occurs.
     * @param color java.awt Color instance.
     */
    public void setBoundaryColor(Color color) {
         if (whatAmI.getFirstString().equals("Softbody")) softbody.setBoundaryColor(color);
    }

    /**
     * Retrieves a list of all physicsObjects this object is presently colliding with. If it is itself a rigidbody, this list retains an entry for every collision point, allowing duplicate physics objects to be in the list. If it is itself a softbody, duplicates do not occur. Collisions with solid joints are not considered.
     * @return ArrayList, element type PhysicsObject, of all object collisions.
     * @throws Exception if an object improperly stores collisions.
     */
    public ArrayList<PhysicsObject> getObjectCollisions() throws Exception {
         ArrayList<PhysicsObject> returnArray = new ArrayList<>();
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 for (int i = 0; i < rigidbody.collidingIDs.size(); i = i + 1) {
                     if (rigidbody.collidingIDs.get(i) >= 0) {
                         if (Rigidbody.get(rigidbody.collidingIDs.get(i)).isHitbox) continue;
                         PhysicsObject other = Simulation.get(whatAmI.getThirdInt()).getObject("Rigidbody", rigidbody.collidingIDs.get(i));
                         returnArray.add(other);
                         if (other.rigidbody.parentSoftbody != -1) returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Softbody", other.rigidbody.parentSoftbody));
                     }
                 }
                 break;
             }
             case("Softbody"): {
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     ArrayList<PhysicsObject> collisions = Simulation.get(whatAmI.getThirdInt()).getObject("Rigidbody", softbody.getMember(i).ID).getObjectCollisions();
                     for (PhysicsObject collision : collisions) {
                         if (!returnArray.contains(collision)) returnArray.add(collision);
                     }
                 }
                 break;
             }
        }
        return(returnArray);
    }

    /**
     * Prints a String to System.out describing all joints of this physicsObject.
     */
    public void describeAllJoints() {
        if (whatAmI.getFirstString().equals("Rigidbody")) for (int i = 0; i < rigidbody.attachments.size(); i++) {
            System.out.print("Joint " + i + " | " + rigidbody.attachments.get(i));
        }
    }

    /**
     * Gets the physicsObject connected by the index-th joint attached to this object. The indices of joints can be found with describeAllJoints().
     * @param index of the joint attachment
     * @return PhysicsObject, unless this object is not a rigidbody or the index is out of bounds, in which case it returns null.
     */
    public PhysicsObject getAttachment(int index) {
        if (whatAmI.getFirstString().equals("Rigidbody") && index >= 0 && index < rigidbody.attachments.size()) {
            int ID = Simulation.get(whatAmI.getThirdInt()).rigidbodyObjectsIDToGlobalID.get(rigidbody.attachments.get(index).connection.ID);
            return Simulation.get(whatAmI.getThirdInt()).getObject(ID);
        }
        return null;
    }

    /**
     * Removes the attachment of this index, which can be found through describeAllJoints. Removing a joint shifts joint indices.
     * Removing attachments recalculates compound and connected body groups. If a joint cannot be removed, it is not, but no error occurs.
     * Joints cannot be removed if part of a softbody.
     * @param index of the removed attachment.
     */
    public void removeAttachment(int index) {
        if (whatAmI.getFirstString().equals("Rigidbody") && index >= 0 && index < rigidbody.attachments.size() && rigidbody.attachments.get(index).type != JointType.Softbody) {
            Rigidbody other = null;
            for (int i = 0; i < rigidbody.attachments.get(index).connection.attachments.size(); i++) {
                other = rigidbody.attachments.get(index).connection;
                if (other.attachments.get(i).connection.ID == rigidbody.ID) {
                    other.attachments.remove(i);
                    break;
                }
            }
            boolean refreshConnectedBodies = rigidbody.attachments.get(index).movesWithConnectedBody();
            boolean refreshCompoundBodies = rigidbody.attachments.get(index).type == JointType.Weld;
            rigidbody.attachments.remove(index);
            if (refreshConnectedBodies && other != null) {
                rigidbody.refreshConnectedBody();
                other.refreshConnectedBody();
            }
            if (refreshCompoundBodies && other != null) {
                rigidbody.refreshCompoundBody();
                other.refreshCompoundBody();
            }
        }
    }

    /**
     * Converts this physicsObject into a description in the form of a String.
     * @return String
     */
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder();
        if (getType().equals("Rigidbody") && rigidbody.parentSoftbody == -1) {
            if (rigidbody.isMovable()) result.append("Dynamic");
            else result.append("Obstacle");
            result.append(" 'Rigidbody' ").append(getGeometryType()).append(" ");
            if (getGeometryType().equals("Polygon")) {
                boolean face = false;
                for (ConvexPolygon convexPolygon : ((Polygon) rigidbody.geometry).convexDecomposition) {
                    if (convexPolygon.faceEdgeIndex != -1) {
                        face = true;
                        break;
                    }
                }
                if (face) result.append("(Face) ");
            }
            if (rigidbody.isAttached) result.append(" (Has Joints) ");
            result.append(rigidbody.ID);
            result.append("\nRadius: ").append(rigidbody.geometry.getLargestDistance());
            result.append(", Area: ").append(rigidbody.getArea()).append(", Moment of Inertia: ").append(rigidbody.getInertia());
            result.append(", Mass: ").append(rigidbody.getMass()).append(", Difference against air density: ").append((rigidbody.getMass() / rigidbody.getArea()) - Simulation.get(whatAmI.getThirdInt()).AIR_DENSITY);
            if (getGeometryType().equals("Polygon")) {
                result.append("\n");
                result.append(((Polygon) rigidbody.geometry).getTriangulationCategorization());
                result.append(((Polygon) rigidbody.geometry).getConvexCategorization());
            }
        }
        if (getType().equals("Softbody")) {
            result.append("'Softbody' ").append(softbody.ID);
            result.append("\n");
            result.append("Size: ").append(softbody.size()).append(" members and ");
            result.append(softbody.boundarySize()).append(" boundary members with ").append(softbody.mass / softbody.size()).append(" mass per member.");
            result.append("\n");
            switch (softbody.getType()) {
                case SoftbodyType.PressureSpring: {
                    result.append("Initial Pressure: ").append(softbody.getInitialPressure()).append(" | Type: PressureSpring.");
                    break;
                }
                case SoftbodyType.ShapeMatchHollow: {

                }
                case SoftbodyType.ShapeMatchSolid: {
                    result.append("Shape-Matching -   Spring Strength: ").append(softbody.SHAPE_MATCH_STRENGTH).append(", Spring Damping: ").append(softbody.SHAPE_MATCH_DAMPING);
                }
                case SoftbodyType.SpringMass: {
                    //all springs have the same stiffness, so we can just pick one. All springs are not required to have the same stiffness, but
                    //do by the fact that the user cannot change individual rigidbody member properties in a softbody.
                    result.append("Spring Strength: ").append(softbody.getMember(0).getSoftbodyAttachmentJoint(0).SPRING_CONSTANT);
                    result.append(", Spring Damping: ").append(softbody.getMember(0).getSoftbodyAttachmentJoint(0).SPRING_DAMPING).append(" | Spring-Mass");
                    break;
                }

            }
        }
        return result.toString();
    }

    /**
     * Sets surface coefficients ("RESTITUTION" or "FRICTION") for this object.
     * @param coefficientType String: "RESTITUTION" or "FRICTION"
     * @param value double for the coefficient value.
     */
    public void setSurfaceCoefficient(String coefficientType, double value) {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 switch(coefficientType) {
                     case("RESTITUTION"): {
                         rigidbody.COEFFICIENT_OF_RESTITUTION = value;
                         break;
                     }
                     case("FRICTION"): {
                         rigidbody.COEFFICIENT_OF_FRICTION = value;
                         break;
                     }
                 }
                 break;
             }
             case("Softbody"): {
                 switch (coefficientType) {
                     case ("RESTITUTION"): {
                         for (int i = 0; i < softbody.size(); i = i + 1) {
                             softbody.getMember(i).COEFFICIENT_OF_RESTITUTION = value;
                         }
                         break;
                     }
                     case ("FRICTION"): {
                         for (int i = 0; i < softbody.size(); i = i + 1) {
                             softbody.getMember(i).COEFFICIENT_OF_FRICTION = value;
                         }
                         break;
                     }
                 }
                 break;
             }
         }
    }

    /**
     * Sets this object's mass, assuming the previous mass was non-zero for moment of inertia adjustments. This method does not alter the compound body's mass.
     * @param mass double describing the new mass for this object. Must be greater than 0.
     */
    public void setMass(double mass) {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 rigidbody.inertia = rigidbody.inertia * (mass / rigidbody.mass);
                 rigidbody.mass = mass;
                 break;
             }
             case("Softbody"): {
                 mass = mass / softbody.size();
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     softbody.getMember(i).inertia = softbody.getMember(i).inertia * (mass / softbody.getMember(i).mass);
                     softbody.getMember(i).mass = mass;
                 }
                 break;
             }
         }
    }

    /**
     * Sets the color of this object.
     * @param color java.awt Color instance.
     */
    public void setColor(Color color) {
         switch (whatAmI.getFirstString()) {
             case "Rigidbody": {
                 rigidbody.geometry.setColor(color);
                 break;
            }
             case "Softbody": {
                 for (int i = 0; i < softbody.size(); i++) {
                     softbody.getMember(i).geometry.setColor(color);
                 }
             }
         }
    }

    /**
     * Sets the density of this object in terms of mass per 2500 unit^2 area.
     * @param density double describing the new density. Must be greater than 0 for moment of inertia adjustments.
     */
    public void setDensity(double density) {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                double mass = (density * rigidbody.getArea()) / 2500.0;
                rigidbody.inertia = rigidbody.inertia * (mass / rigidbody.mass);
                rigidbody.mass = mass;
                break;
            }
            case("Softbody"): {
                for (int i = 0; i < softbody.size(); i = i + 1) {
                    double mass = (density * softbody.getMember(i).getArea()) / 2500.0;
                    softbody.getMember(i).inertia = softbody.getMember(i).inertia * (mass / softbody.getMember(i).mass);
                    softbody.getMember(i).mass = mass;
                }
                break;
            }
        }
    }

    /**
     * Sets the material of this object, which carries a name, density, and surface coefficients.
     * @param name the name of the material being passed in. Must have a density greater than 0 for density changes to occur.
     */
    public void setMaterial(String name) {
        for (int i = 0; i < Simulation.materials.size(); i = i + 1) {
             if (Simulation.materials.get(i).name.equals(name)) {
                 material = Simulation.materials.get(i);
                 try {
                     if (whatAmI.getFirstString().equals("Softbody")) {
                         for (int j = 0; j < softbody.size(); j = j + 1) {
                             Simulation.get(whatAmI.getThirdInt()).getObject("Rigidbody", softbody.getMember(j).ID).setMaterial(name);
                         }
                         break;
                     }
                     setSurfaceCoefficient("RESTITUTION", material.restitution);
                     setSurfaceCoefficient("FRICTION", material.dynamic_friction);
                     if (!hasZeroMass()) setDensity(material.density);
                 }
                 catch (Exception e) {
                     System.out.println(e);
                 }
                 break;
             }
         }
    }

    /**
     *
     * @return the name of the material of this object, or nothing if no material is assigned.
     */
    public String getMaterialName() {
         try {
             return(material.name);
         }
         catch (Exception e) {
             return "";
         }
    }
    private boolean hasZeroMass() {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 if (rigidbody.mass == 0.0) return(true);
                 break;
             }
             case("Softbody"): {
                 if (softbody.mass == 0.0) return(true);
                 break;
             }
         }
         return(false);
    }

    /**
     * Determines whether this object will adopt only the other object's surface coefficients in any collision (true), or whether it will average them (false).
     * @param a
     */
    public void makeAdoptOtherSurfaceOnly(boolean a) {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.makeAdoptOtherSurfaceOnly(a);
                break;
            }
            case("Softbody"): {
                for (int i = 0; i < softbody.size(); i = i + 1) {
                    softbody.getMember(i).makeAdoptOtherSurfaceOnly(a);
                }
                break;
            }

        }
    }

    /**
     * Determines whether this object will freely rotate (false) or not (true).
     * @param a
     */
    public void lockRotation(boolean a) {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.lockRotation(a);
                break;
            }
            case("Softbody"): {
                for (int i = 0; i < softbody.size(); i = i + 1) {
                    softbody.getMember(i).lockRotation(a);
                }
                break;
            }

        }
    }

    /**
     * Adds a controller to the object (if a rigidbody) when a key is pressed while the native JFrame display is in focus. It does not apply to softbodies, but applies it to the object's compound body if applicable. Velocities given are relative to the object's determined ground velocity.
     * @param key the character of the key to trigger the controller
     * @param initialVelocity the velocity along the direction of the controller that the object's center of mass is set to when the key is first pressed.
     * @param releaseVelocity the velocity along the direction of the controller that the object's center of mass is set to when the key is released.
     * @param sustainedForce the acceleration along the direction of the controller that the object's center of mass is set to while the key is pressed.
     * @param maxSpeed the max speed along the direction of the controller that the object's center of mass can reach while the key is pressed.
     * @param direction the direction vector for this controller.
     * @param excludeIfKey if this key is pressed down, this controller will not activate.
     */
    public void addController(char key, double initialVelocity, double releaseVelocity, double sustainedForce, double maxSpeed, double[] direction, char excludeIfKey) {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 if (!Simulation.get(whatAmI.getThirdInt()).display.hasController) {
                     Simulation.get(whatAmI.getThirdInt()).display.hasController = true;
                     Simulation.get(whatAmI.getThirdInt()).display.addKeyListener(Simulation.get(whatAmI.getThirdInt()).display);
                 }
                 new Controller(key, initialVelocity, releaseVelocity, sustainedForce, maxSpeed, direction, rigidbody, excludeIfKey);
                 break;
             }
             default: {
                 System.out.println("Invalid type to add Controller to.");
                 break;
             }
         }
    }

    /**
     * Adds the developer's tested "standard" bundle of controllers set to WASD.
     */
    public void addStandardControllerBundle() {
        addController('d', 20.0, 15.0 ,300.0, 100.0, new double[]{1.0, 0.0}, 'a');
        addController('a', 20.0,15.0,  300.0, 100.0, new double[]{-1.0, 0.0}, 'd');
        addController('w', 100.0, 0.0, 0.0, 0.0, new double[]{0.0, -1.0}, '\u0000');
        addController('s', 0.0, 0.0, 180.0, 0.0, new double[]{0.0, 1.0}, '\u0000');
        makeAdoptOtherSurfaceOnly(true);
        lockRotation(true);
        Simulation.createMaterial("Player", 100.0, 0.1, 0.5);
        setMaterial("Player");
     }

    /**
     * Sets some parameters of the controllers applied to this object.
     * @param stepsUntilOffGround the number of simulation steps until the controller considers the object off the ground.
     * @param velocityReframingRate the rate at which the controller adjusts the ground velocity to the zero vector when no longer touching the ground.
     */
    public void setControllerParameters(int stepsUntilOffGround, double velocityReframingRate) {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            for (int i = 0; i < rigidbody.controllers.size(); i = i + 1) {
                rigidbody.controllers.get(i).stepsUntilNotTouchingGround = stepsUntilOffGround;
                rigidbody.controllers.get(i).velocityDecrementWhenInAir = velocityReframingRate;
            }
        }
    }

    //joint attachment access methods
    private boolean physicsObjectIsJointAttachable(PhysicsObject other) {
         return (whatAmI.getFirstString().equals("Rigidbody") && other.whatAmI.getFirstString().equals("Rigidbody"));
    }

    /**
     * Attaches one object to another via a spring of given strength and damping at the center of mass.
     * @param other rigidbody PhysicsObject
     * @param SPRING_STRENGTH the constant 'k' in Hooke's Law.
     * @param SPRING_DAMPING a constant that combats the acceleration due to spring strength proportional to the magnitude of velocity by this constant.
     * @throws Exception if one object is not a rigidbody.
     */
    public void springAttach(PhysicsObject other, double SPRING_STRENGTH, double SPRING_DAMPING) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.springAttach(other.rigidbody, SPRING_STRENGTH, SPRING_DAMPING);
    }

    /**
     * Attaches one object to another via a spring of given strength and damping at the given offsets from each object's center of mass.
     * @param other rigidbody PhysicsObject.
     * @param SPRING_STRENGTH the constant 'k' in Hooke's Law.
     * @param SPRING_DAMPING a constant that combats the acceleration due to spring strength proportional to the magnitude of velocity by this constant.
     * @param parentOffset offset vector from this object's center of mass to where the connection point on this object is.
     * @param otherOffset offset vector from the other object's center of mass to where the connection point on the other object is.
     * @param solid whether the joint is solid for collision purposes.
     */
    public void springAttach(PhysicsObject other, double SPRING_STRENGTH, double SPRING_DAMPING, double[] parentOffset, double[] otherOffset, boolean solid) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.springAttach(other.rigidbody, SPRING_STRENGTH, SPRING_DAMPING, parentOffset, otherOffset, solid);
    }

    /**
     * Attaches one object to another via a distance constraint of given strength and leeway at the given offsets from each object's center of mass.
     * @param other rigidbody PhysicsObject
     * @param enforcement_strength determines the soft correction applied via a spring. The spring strength equals minimum of both objects' masses multiplied by the enforcement strength. Correspondingly, the damping equals the damping coefficient relator times the sqrt(effective mass * spring strength).
     * @param leeway the amount of deviation in distance from the ideal distance constraint allowed.
     * @param parentOffset offset vector from this object's center of mass to where the connection point on this object is.
     * @param otherOffset offset vector from the other object's center of mass to where the connection point on the other object is.
     * @param solid whether the joint is solid for collision purposes.
     */
    public void distanceSpringAttach(PhysicsObject other, double enforcement_strength, double leeway, double[] parentOffset, double[] otherOffset, boolean solid) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.distanceSpringAttach(other.rigidbody, enforcement_strength, leeway, parentOffset, otherOffset, solid);
    }

    /**
     * Attaches one object to another at an offset from each object with the constraint that those points on each object exist in the same world space.
     * @param other rigidbody PhysicsObject
     * @param parentOffset offset vector from this object's center of mass to where the connection point on this object is.
     * @param otherOffset offset vector from the other object's center of mass to where the connection point on the other object is.
     */
    public void pinAttach(PhysicsObject other, double[] parentOffset, double[] otherOffset) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.pinAttach(other.rigidbody, parentOffset, otherOffset);
    }

    /**
     * Attaches one object to another at an offset from each object with the constraint that those points on each object must exist in the same world space.
     * The additional constraint here when compared to pin joints is that the angle between the offset vectors must be between two angle offset bounds. This angle is the angle between the vector from the parent object to the connection point and the vector between the connection point and the other object. The bounds are in terms of an offset between the first vector and the second.
     * The acceptable range of angle lies between angleBound1 and angleBound2.
     * @param other rigidbody PhysicsObject
     * @param parentOffset offset vector from this object's center of mass to where the connection point on this object is.
     * @param otherOffset offset vector from the other object's center of mass to where the connection point on the other object is.
     * @param angleBound1
     * @param angleBound2
     */
    public void revoluteAttach(PhysicsObject other, double[] parentOffset, double[] otherOffset, double angleBound1, double angleBound2) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.revoluteAttach(other.rigidbody, parentOffset, otherOffset, angleBound1, angleBound2);
    }

    /**
     * Attaches two objects to each other in a compound body where they cannot freely rotate about one another and must remain connected at the given points.
     * @param other rigidbody PhysicsObject
     * @param parentOffset offset vector from this object's center of mass to where the connection point on this object is.
     * @param otherOffset offset vector from the other object's center of mass to where the connection point on the other object is.
     */
    public void weldAttach(PhysicsObject other, double[] parentOffset, double[] otherOffset) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.weldAttach(other.rigidbody, parentOffset, otherOffset);
    }

    /**
     * Attaches this object to another such that the connection point must remain with the bounds along the direction vector.
     * @param other rigidbody PhysicsObject
     * @param parentOffset offset vector from this object's center of mass to where the connection point on this object is.
     * @param otherOffset offset vector from the other object's center of mass to where the connection point on the other object is.
     * @param direction the direction vector along which the connection point may move.
     * @param bounds the upper and lower bounds (order does not matter) along that direction vector along which the connection point may move.
     */
    public void translationalAttach(PhysicsObject other, double[] parentOffset, double[] otherOffset, double[] direction, double[] bounds) {
        if (!physicsObjectIsJointAttachable(other)) throw new IllegalArgumentException("Physics joints may not attach between non-rigidbody objects");
        rigidbody.translationalAttach(other.rigidbody, parentOffset, otherOffset, direction, bounds);
    }

}
