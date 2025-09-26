package PhysicsSim;

import java.awt.*;
import java.util.ArrayList;

public class PhysicsObject {
     private final Triplet whatAmI;
     protected final Rigidbody rigidbody;
     protected final Point point;
     protected final Softbody softbody;
     protected Material material;

     protected PhysicsObject(Rigidbody input) {
        whatAmI = new Triplet("Rigidbody", input.ID, input.simID);
        rigidbody = input;
        point = null;
        softbody = null;
     }
     protected PhysicsObject(Point input) {
         whatAmI = new Triplet("Point", input.ID, input.simID);
         rigidbody = null;
         point = input;
         softbody = null;
     }
     protected PhysicsObject(Softbody input) {
         whatAmI = new Triplet("Softbody", input.ID, input.simID);
         rigidbody = null;
         point = null;
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
             case("Point"): {
                 point.simID = -1;
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
             case("Point"): {
                 point.simID = whatAmI.getThirdInt();
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
            case("Point"): return(new double[]{point.getAngularV(), point.getAngularA()});
            default:
                throw new Exception("Physics object (softbody likely) does not store angular movement.");
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
             case("Point"): {
                 point.angularV = angularMovement[0];
                 point.angularA = angularMovement[1];
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
                         int parent = Point.get(index).parentSoftbody;
                         if (Point.get(index).isHitbox) continue;
                         if (parent != -1) {
                             PhysicsObject parentSoftbody = Simulation.get(whatAmI.getThirdInt()).getObject("Softbody", parent);
                             if (!returnArray.contains(parentSoftbody)) returnArray.add(parentSoftbody);
                         }
                         returnArray.add(Simulation.get(whatAmI.getThirdInt()).getObject("Point", index));
                     }
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
             case("Softbody"): {
                 for (int i = 0; i < softbody.size(); i = i + 1) {
                     ArrayList<PhysicsObject> collisions = Simulation.get(whatAmI.getThirdInt()).getObject("Point", softbody.getMember(i).ID).getObjectCollisions();
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
             case("Point"): {
                 switch(coefficientType) {
                     case("RESTITUTION"): {
                         point.COEFFICIENT_OF_RESTITUTION = value;
                         break;
                     }
                     case("FRICTION"): {
                         point.COEFFICIENT_OF_FRICTION = value;
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
             case("Point"): {
                 point.inertia = point.inertia * (mass / point.mass);
                 point.mass = mass;
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
    public void setDensity(double density) {
        switch(whatAmI.getFirstString()) {
            case("Rigidbody"): {
                double mass = (density * rigidbody.getArea()) / 2500.0;
                rigidbody.inertia = rigidbody.inertia * (mass / rigidbody.mass);
                rigidbody.mass = mass;
                break;
            }
            case("Point"): {
                double mass = (density * point.getArea()) / 2500.0;
                point.inertia = point.inertia * (mass / point.mass);
                point.mass = mass;
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
                             Simulation.get(whatAmI.getThirdInt()).getObject("Point", softbody.getMember(j).ID).setMaterial(name);
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
             case("Point"): {
                 if (point.mass == 0.0) return(true);
                 break;
             }
             case("Softbody"): {
                 if (softbody.mass == 0.0) return(true);
                 break;
             }
             default: {
                 return(false);
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
            case("Point"): {
                point.makeAdoptOtherSurfaceOnly(a);
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
            case("Point"): {
                point.lockRotation(a);
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
             case("Point"): {
                 if (!Simulation.get(whatAmI.getThirdInt()).display.hasController) {
                     Simulation.get(whatAmI.getThirdInt()).display.hasController = true;
                     Simulation.get(whatAmI.getThirdInt()).display.addKeyListener(Simulation.get(whatAmI.getThirdInt()).display);
                 }
                 new Controller(key, initialVelocity, releaseVelocity, sustainedForce, maxSpeed, direction, point, excludeIfKey);
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
         switch(whatAmI.getFirstString()) {
             case("Rigidbody"): {
                 for (int i = 0; i < rigidbody.controllers.size(); i = i + 1) {
                     rigidbody.controllers.get(i).stepsUntilNotTouchingGround = stepsUntilOffGround;
                     rigidbody.controllers.get(i).velocityDecrementWhenInAir = velocityReframingRate;
                 }
                 break;
             }
             case("Point"): {
                 for (int i = 0; i < point.controllers.size(); i = i + 1) {
                     point.controllers.get(i).stepsUntilNotTouchingGround = stepsUntilOffGround;
                     point.controllers.get(i).velocityDecrementWhenInAir = velocityReframingRate;
                 }
                 break;
             }
         }
    }
 }
