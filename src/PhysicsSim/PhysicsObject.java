package PhysicsSim;

import java.awt.*;
import java.util.ArrayList;

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
             case("Softbody"): {
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     softbody.getMember(i).simID = -1;
                 }
                 softbody.simID = -1;
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
             case("Softbody"): {
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     softbody.getMember(i).simID = whatAmI.getThirdInt();
                 }
                 softbody.simID = whatAmI.getThirdInt();
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
            case("Softbody"): {
                for (int i = 0; i < softbody.size(); i = i + 1) {
                    softbody.getMember(i).draw = visible;
                }
                break;
            }
        }
    }

    public double[] getAcceleration() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getAX(), rigidbody.getAY()});
        }
        throw new Exception("Physics object (softbody likely) does not store acceleration.");
    }
    public double[] getVelocity() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getVX(), rigidbody.getVY()});
        }
        throw new Exception("Physics object (softbody likely) does not store velocity.");
    }
    public double[] getPosition() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getPosX(), rigidbody.getPosY()});
        }
        throw new Exception("Physics object (softbody likely) does not store position.");
    }
    public double[] getAngularMovement() throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            return (new double[]{rigidbody.getAngularV(), rigidbody.getAngularA()});
        }
        throw new Exception("Physics object (softbody likely) does not store angular movement.");
    }
    public void setAcceleration(double[] acceleration) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setAX(acceleration[0]);
            rigidbody.setAY(acceleration[1]);
        } else {
            throw new Exception("Physics object (softbody likely) does not store acceleration.");
        }
    }
    public void setVelocity(double[] velocity) throws Exception {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            rigidbody.setVX(velocity[0]);
            rigidbody.setVY(velocity[1]);
        } else {
            throw new Exception("Physics object (softbody likely) does not store velocity.");
        }
    }
    public void setPosition(double[] position) throws Exception {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                rigidbody.setPosX(position[0]);
                rigidbody.setPosY(position[1]);
                break;
            }
            default:
                throw new Exception("Physics object (softbody likely) does not store position.");
        }
    }
    public void setAngularMovement(double[] angularMovement) throws Exception {
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 rigidbody.setAngularV(angularMovement[0]);
                 rigidbody.setAngularA(angularMovement[1]);
                 break;
             }
             default:
                 throw new Exception("Physics object (softbody likely) does not store position.");
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
         if (whatAmI.getFirstString().equals("Rigidbody")) return(rigidbody.geometry.getLargestDistance());
         throw new Exception("Accessed physicsObject is not a 'Rigidbody', so 'getRadius()' is not applicable.");
    }
    public void setBoundaryColor(Color color) {
         if (whatAmI.getFirstString().equals("Softbody")) softbody.setBoundaryColor(color);
         else System.out.println("'setBoundaryColor(Color color)' may not be applied to any physicsObject but a softbody.");
    }
    public void enableBoundary(boolean boundaryEnabled) {
        if (whatAmI.getFirstString().equals("Softbody")) softbody.boundaryCollision = boundaryEnabled;
        else System.out.println("'enableBoundary(boolean boundaryEnabled)' may not be applied to any physicsObject but a softbody.");
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

    public void setSurfaceCoefficient(String coefficientType, double value) throws Exception{
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
                 switch(coefficientType) {
                     case("RESTITUTION"): {
                         for (int i = 0; i < softbody.size(); i = i + 1) {
                             softbody.getMember(i).COEFFICIENT_OF_RESTITUTION = value;
                         }
                         break;
                     }
                     case("FRICTION"): {
                         for (int i = 0; i < softbody.size(); i = i + 1) {
                             softbody.getMember(i).COEFFICIENT_OF_FRICTION = value;
                         }
                         break;
                     }
                 }
                 break;
             }
             default:
                 throw new Exception("The 'coefficientType' must be either 'RESTITUTION' or 'FRICTION'.");
         }
    }
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
                     else System.out.println("Cannot change the density of a zero-mass object.");
                 }
                 catch (Exception e) {
                     System.out.println(e);
                 }
                 break;
             }
         }
    }
    public String getMaterialName() {
         return(material.name);
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
    public void setControllerParameters(double stepsUntilOffGround, double velocityReframingRate) {
        if (whatAmI.getFirstString().equals("Rigidbody")) {
            for (int i = 0; i < rigidbody.controllers.size(); i = i + 1) {
                rigidbody.controllers.get(i).stepsUntilNotTouchingGround = stepsUntilOffGround;
                rigidbody.controllers.get(i).velocityDecrementWhenInAir = velocityReframingRate;
            }
        }
    }
 }
