package PhysicsSim;

class Circle extends GeometricType {
    private final double radius;
    public double[] angularPosTest = new double[]{50.0, 0.0};
    public Circle(double radius) {
        super(Rigidbody.num);
        this.radius = radius;
        largestDistanceSquared = radius * radius;
        largestDistance = radius;
        masslessInertia = 0.5 * largestDistanceSquared;
        area = Math.PI * largestDistanceSquared;
        bottomBoundBox = radius;
        topBoundBox = -radius;
        leftBoundBox = -radius;
        rightBoundBox = radius;
    }

    public double getRadius() {
        return(radius);
    }

    @Override
    boolean findCollisions(GeometricType otherGeometry) {
        Rigidbody otherObject = Rigidbody.get(otherGeometry.getParentRigidbodyID());
        Rigidbody myObject = Rigidbody.get(getParentRigidbodyID());
        double MTV_EPSILON = Simulation.get(myObject.simID).MTV_EPSILON;
        if (otherGeometry instanceof Polygon) {
            Polygon other = (Polygon) otherGeometry;
            boolean intersecting = false;
            for (int i = 0; i < other.getNumPoints(); i = i + 1) {
                if (other.triangles.get(i).doesExist()) {
                    Triplet results = other.triangles.get(i).checkCollisions(this);
                    if (results.getFirstBoolean()) {
                        intersecting = true;
                        myObject.contactPoints.add(results.getThirdDoubleArrayReference());
                        Double[] MTV = results.getSecondDoubleArrayReference();
                        if (otherObject.isMovable()){
                            double magnitude1 = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
                            double magnitude2 = -(magnitude1 - MTV_EPSILON) * (otherObject.getCompoundMass() / myObject.getCompoundMass()) - MTV_EPSILON;
                            MTV[0] = MTV[0] * (magnitude2 / magnitude1);
                            MTV[1] = MTV[1] * (magnitude2 / magnitude1);
                            myObject.MTVs.add(MTV);
                            myObject.collidingIDs.add(otherObject.getID());
                        }
                        else {
                            myObject.MTVs.add(MTV);
                            myObject.collidingIDs.add(otherObject.getID());
                        }

                    }
                }
            }
            return(intersecting);
        }
        else if (otherGeometry instanceof Circle) {
            Circle other = (Circle) otherGeometry;
            boolean intersecting = false;
            double dx = (myObject.getPosX() - otherObject.getPosX());
            double dy = (myObject.getPosY() - otherObject.getPosY());
            double distance = Math.sqrt(dx * dx + dy * dy);
            double overlap = distance - (other.getRadius() + radius);
            if (overlap <= 0.0) {
                intersecting = true;
                Double[] MTV = new Double[2];
                Double[] pointOfContact = new Double[2];
                double temp = overlap;
                if (otherObject.isMovable()){
                    temp = (overlap / distance) * (otherObject.getCompoundMass() / (otherObject.getCompoundMass() + myObject.getCompoundMass()));
                }
                else {
                    temp = (overlap / distance);
                }
                MTV[0] = (otherObject.getPosX() - myObject.getPosX()) * temp;
                MTV[0] = MTV[0] + MTV_EPSILON * Math.signum(MTV[0]);
                MTV[1] = (otherObject.getPosY() - myObject.getPosY()) * temp;
                MTV[1] = MTV[1] + MTV_EPSILON * Math.signum(MTV[1]);
                pointOfContact[0] = (otherObject.getPosX() - myObject.getPosX()) * (radius / distance) + MTV[0] + myObject.getPosX();
                pointOfContact[1] = (otherObject.getPosY() - myObject.getPosY()) * (radius / distance) + MTV[1] + myObject.getPosY();
                myObject.MTVs.add(MTV);
                myObject.contactPoints.add(pointOfContact);
                myObject.collidingIDs.add(otherObject.ID);
            }
            return(intersecting);
        }
        else {
            System.out.println("Geometric type collision that this object is unfamiliar with.");
            return(false);
        }
    }

    @Override
    boolean findCollisions(Joint solidJoint) {
        double lineThickness = Simulation.get(solidJoint.parent.simID).solidJointCollisionLineThickness;
        lineThickness = Math.min(Math.min(lineThickness, solidJoint.parent.geometry.getLargestDistance()), solidJoint.connection.geometry.getLargestDistance());
        double x1 = solidJoint.parent.getPosX() + solidJoint.offsetFromCMParent[0];
        double x2 = solidJoint.connection.getPosX() + solidJoint.offsetFromCMOther[0];
        double y1 = solidJoint.parent.getPosY() + solidJoint.offsetFromCMParent[1];
        double y2 = solidJoint.connection.getPosY() + solidJoint.offsetFromCMOther[1];
        double posX = Rigidbody.get(getParentRigidbodyID()).getPosX();
        double posY = Rigidbody.get(getParentRigidbodyID()).getPosY();
        double magnitude = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        double nX = -(y2 - y1) / magnitude;
        double nY = (x2 - x1) / magnitude;
        double MTV_EPSILON = Rigidbody.get(getParentRigidbodyID()).sim.MTV_EPSILON;
        double dotProd = (posX - x1) * nX + (posY - y1) * nY;
        double orthoDotProd = (posX - x1) * nY + (posY - y1) * -nX;
        if (Math.abs(dotProd) - radius <= lineThickness && orthoDotProd >= 0.0 && orthoDotProd <= magnitude) {
            Rigidbody rigidbody = Rigidbody.get(getParentRigidbodyID());
            double inverseMass = 0.0;
            if (solidJoint.connection.isMovable() && solidJoint.parent.isMovable()) {
                inverseMass = 1.0 / (solidJoint.parent.getMass() + solidJoint.connection.getMass() + rigidbody.getMass());
            }
            MTV_EPSILON *= (solidJoint.parent.getCompoundMass() + solidJoint.connection.getCompoundMass()) * inverseMass;
            double multiplier = (lineThickness - Math.abs(dotProd) + radius + MTV_EPSILON) * Math.signum(dotProd);
            Double[] MTV = new Double[]{multiplier * nX, multiplier * nY};
            rigidbody.MTVs.add(MTV);
            rigidbody.MTVs.add(MTV);
            magnitude = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
            Double[] contactPoint = new Double[]{radius * -(MTV[0] / magnitude) + posX + MTV[0], radius * -(MTV[1] / magnitude) + posY + MTV[1]};
            rigidbody.contactPoints.add(contactPoint);
            //this is the way that the second contact point is encoded
            rigidbody.contactPoints.add(new Double[]{Double.NaN, 0.0});
            rigidbody.collidingIDs.add(-solidJoint.parent.ID - 2);
            rigidbody.collidingIDs.add(-solidJoint.connection.ID - 2);
            return true;
        }
        return false;
    }

    @Override
    boolean checkForCollisionsWall() {
        boolean intersecting = false;
        Rigidbody myObject = Rigidbody.get(getParentRigidbodyID());
        Simulation sim = Simulation.get(myObject.simID);
        double posX = myObject.getPosX();
        double posY = myObject.getPosY();
        double worldBottomBound = sim.worldBottomBound;
        double worldTopBound = sim.worldTopBound;
        double worldLeftBound = sim.worldLeftBound;
        double worldRightBound = sim.worldRightBound;
        if (posY + radius >= worldBottomBound) {
            myObject.MTVs.add(new Double[]{0.0, worldBottomBound - (posY + radius) - sim.MTV_EPSILON});
            myObject.contactPoints.add(new Double[]{posX, worldBottomBound});
            myObject.collidingIDs.add(-1);
            intersecting = true;
        }
        if (posY - radius <= worldTopBound) {
            myObject.MTVs.add(new Double[]{0.0, worldTopBound - (posY - radius) + sim.MTV_EPSILON});
            myObject.contactPoints.add(new Double[]{posX, worldTopBound});
            myObject.collidingIDs.add(-1);
            intersecting = true;
        }
        if (posX - radius <= worldLeftBound) {
            myObject.MTVs.add(new Double[]{worldLeftBound - (posX - radius) + sim.MTV_EPSILON, 0.0});
            myObject.contactPoints.add(new Double[]{worldLeftBound, posY});
            myObject.collidingIDs.add(-1);
            intersecting = true;
        }
        if (posX + radius >= worldRightBound) {
            myObject.MTVs.add(new Double[]{worldRightBound - (posX + radius) - sim.MTV_EPSILON, 0.0});
            myObject.contactPoints.add(new Double[]{worldRightBound, posY});
            myObject.collidingIDs.add(-1);
            intersecting = true;
        }
        return(intersecting);
    }

    @Override
    void rotateAroundCenter(double theta) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double x = angularPosTest[0];
        double y = angularPosTest[1];
        angularPosTest[0] = x * cos - y * sin;
        angularPosTest[1] = x * sin + y * cos;
    }

    @Override
    boolean pointInside(double[] point) {
        double dx = point[0] - Rigidbody.get(getParentRigidbodyID()).getPosX();
        double dy = point[1] - Rigidbody.get(getParentRigidbodyID()).getPosY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        return(distance <= radius);
    }

    @Override
    double[] calculateAirResistance(double AIR_DENSITY, double DRAG_COEFFICIENT, double[] v) {
        double crossArea = 2.0 * radius;
        double magnitude = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
        double forceMagnitude = 0.5 * AIR_DENSITY * crossArea * DRAG_COEFFICIENT * magnitude * magnitude;
        double uX = v[0] / magnitude;
        double uY = v[1] / magnitude;
        //the 10.0 here is arbitrary, and is a stand-in to prevent quick changes in velocity (impulses) from causing
        //the simplified drag force from "over-correcting"
        forceMagnitude = Math.min(Rigidbody.get(getParentRigidbodyID()).getMass() * magnitude * 10.0, forceMagnitude);
        double[] fD = new double[]{-uX * forceMagnitude, -uY * forceMagnitude};
        if (Double.isNaN(fD[0])) fD[0] = 0.0;
        if (Double.isNaN(fD[1])) fD[1] = 0.0;

        return(new double[]{fD[0], fD[1], 0.0});
    }

    @Override
    Triplet getDrawInt(double shiftX, double resolutionCenterX, double pixelShiftX, double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling) {
        double posX = Rigidbody.get(getParentRigidbodyID()).getPosX();
        double posY = Rigidbody.get(getParentRigidbodyID()).getPosY();
        int x = (int) Math.round(((posX - shiftX - resolutionCenterX) / resolutionScaling) + resolutionCenterX + pixelShiftX);
        int y = (int) Math.round(((posY - shiftY - resolutionCenterY) / resolutionScaling) + resolutionCenterY + pixelShiftY);
        return new Triplet(RigidbodyGeometries.Circle, new int[]{x, y}, radius / resolutionScaling);
    }

    @Override
    Triplet getDrawDoubles(double shiftX, double resolutionCenterX, double pixelShiftX, double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling) {
        double posX = Rigidbody.get(getParentRigidbodyID()).getPosX();
        double posY = Rigidbody.get(getParentRigidbodyID()).getPosY();
        double x = ((posX - shiftX - resolutionCenterX) / resolutionScaling) + resolutionCenterX + pixelShiftX;
        double y = ((posY - shiftY - resolutionCenterY) / resolutionScaling) + resolutionCenterY + pixelShiftY;
        return new Triplet(RigidbodyGeometries.Circle, new double[]{x, y}, radius / resolutionScaling);
    }
}
