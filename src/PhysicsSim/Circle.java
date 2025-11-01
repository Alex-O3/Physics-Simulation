package PhysicsSim;

class Circle extends GeometricType {
    private final double radius;
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
                            double magnitude2 = -(magnitude1 - MTV_EPSILON) * (otherObject.getMass() / myObject.getMass()) - MTV_EPSILON;
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
                    temp = (overlap / distance) * (otherObject.getMass() / (otherObject.getMass() + myObject.getMass()));
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
    boolean findCollisions(Softbody softbody) {
        boolean intersecting = false;
        Rigidbody rigidbody = Rigidbody.get(getParentRigidbodyID());
        //returns (intersecting, MTV, edgeIndex (maps n -> -n - 2
        Triplet results = softbody.resolvePointInside(new double[]{rigidbody.getPosX(), rigidbody.getPosY()}, radius);
        if (results.getFirstBoolean()) {
            Double[] MTV = new Double[2];
            double multiplier = (Rigidbody.getMass(results.getThirdInt()) / (Rigidbody.getMass(results.getThirdInt()) + rigidbody.getMass()));
            MTV[0] = results.getSecondDoubleArray()[0] * multiplier;
            MTV[1] = results.getSecondDoubleArray()[1] * multiplier;
            double magnitude = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
            double nX = MTV[0] / magnitude;
            double nY = MTV[1] / magnitude;
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            Double[] contactPoint = new Double[]{rigidbody.getPosX() + MTV[0] - radius * nX, rigidbody.getPosY() + MTV[1] - radius * nY};
            rigidbody.contactPoints.add(contactPoint);
            rigidbody.MTVs.add(MTV);
            rigidbody.collidingIDs.add(results.getThirdInt());
            intersecting = true;
        }
        return(intersecting);
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
