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
        if (otherObject.simID != myObject.simID) return false;
        if (otherGeometry instanceof Polygon) {
            Polygon other = (Polygon) otherGeometry;
            boolean intersecting = false;
            for (ConvexPolygon otherConvex : other.convexDecomposition) {
                Triplet results = otherConvex.checkCollisions(this);
                if (results.getFirstBoolean()) {
                    intersecting = true;
                    myObject.contactPoints.add(results.getThirdDoubleArray());
                    double[] MTV = results.getSecondDoubleArray();
                    if (otherObject.isMovable()){
                        double magnitude1 = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
                        double magnitude2 = -(magnitude1) * (otherObject.getCompoundMass() / myObject.getCompoundMass());
                        MTV[0] = MTV[0] * (magnitude2 / magnitude1);
                        MTV[1] = MTV[1] * (magnitude2 / magnitude1);
                        myObject.MTVs.add(MTV);
                        myObject.collidingIDs.add(new int[]{otherObject.ID});
                    }
                    else {
                        double multiplier = -(otherObject.getCompoundMass() + myObject.getCompoundMass()) / myObject.getCompoundMass();
                        MTV[0] *= multiplier;
                        MTV[1] *= multiplier;
                        myObject.MTVs.add(MTV);
                        myObject.collidingIDs.add(new int[]{otherObject.ID});
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
                double[] MTV = new double[2];
                double[] pointOfContact = new double[4];
                double temp = overlap;
                if (otherObject.isMovable()){
                    temp = (overlap / distance) * (otherObject.getCompoundMass() / (otherObject.getCompoundMass() + myObject.getCompoundMass()));
                }
                else {
                    temp = (overlap / distance);
                }
                MTV[0] = (otherObject.getPosX() - myObject.getPosX()) * temp;
                MTV[1] = (otherObject.getPosY() - myObject.getPosY()) * temp;
                temp = 1.0 + MTV_EPSILON / Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
                MTV[0] *= temp;
                MTV[1] *= temp;
                pointOfContact[0] = (otherObject.getPosX() - myObject.getPosX()) * (radius / distance) + myObject.getPosX();
                pointOfContact[1] = (otherObject.getPosY() - myObject.getPosY()) * (radius / distance) + myObject.getPosY();
                pointOfContact[2] = (myObject.getPosX() - otherObject.getPosX()) * (other.getRadius() / distance) + otherObject.getPosX();
                pointOfContact[3] = (myObject.getPosY() - otherObject.getPosY()) * (other.getRadius() / distance) + otherObject.getPosY();
                myObject.MTVs.add(MTV);
                myObject.contactPoints.add(pointOfContact);
                myObject.collidingIDs.add(new int[]{otherObject.ID});
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
            double[] MTV = new double[]{multiplier * nX, multiplier * nY};
            rigidbody.MTVs.add(MTV);
            magnitude = Math.sqrt(MTV[0] * MTV[0] + MTV[1] * MTV[1]);
            double[] contactPoint = new double[]{radius * -(MTV[0] / magnitude) + posX + MTV[0], radius * -(MTV[1] / magnitude) + posY + MTV[1]};
            rigidbody.contactPoints.add(contactPoint);
            int solidJointAttachmentIDParent = -1;
            for (int i = 0; i < solidJoint.parent.attachments.size(); i++) {
                if (solidJoint.parent.attachments.get(i) == solidJoint) {
                    solidJointAttachmentIDParent = i;
                    break;
                }
            }
            rigidbody.collidingIDs.add(new int[]{-solidJoint.parent.ID - 2, -solidJoint.connection.ID - 2, solidJointAttachmentIDParent});
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
            myObject.MTVs.add(new double[]{0.0, worldBottomBound - (posY + radius) - sim.MTV_EPSILON});
            myObject.contactPoints.add(new double[]{posX, posY + radius});
            myObject.collidingIDs.add(new int[]{-1});
            intersecting = true;
        }
        if (posY - radius <= worldTopBound) {
            myObject.MTVs.add(new double[]{0.0, worldTopBound - (posY - radius) + sim.MTV_EPSILON});
            myObject.contactPoints.add(new double[]{posX, posY - radius});
            myObject.collidingIDs.add(new int[]{-1});
            intersecting = true;
        }
        if (posX - radius <= worldLeftBound) {
            myObject.MTVs.add(new double[]{worldLeftBound - (posX - radius) + sim.MTV_EPSILON, 0.0});
            myObject.contactPoints.add(new double[]{posX - radius, posY});
            myObject.collidingIDs.add(new int[]{-1});
            intersecting = true;
        }
        if (posX + radius >= worldRightBound) {
            myObject.MTVs.add(new double[]{worldRightBound - (posX + radius) - sim.MTV_EPSILON, 0.0});
            myObject.contactPoints.add(new double[]{posX + radius, posY});
            myObject.collidingIDs.add(new int[]{-1});
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
    double[] calculateAirResistance(double AIR_DENSITY, double DRAG_COEFFICIENT, double[] windSpeed, double dt) {
        Rigidbody parent = getParent();
        double[] result = new double[3];
        double vMag = Math.sqrt((parent.getVX() - windSpeed[0]) * (parent.getVX() - windSpeed[0]) + (parent.getVY() - windSpeed[1]) * (parent.getVY() - windSpeed[1]));
        result[0] = -0.5 * (4.0 * (parent.getVX() - windSpeed[0]) * vMag - Math.PI * radius * parent.getAngularV() * (parent.getVY() - windSpeed[1]));
        result[1] = -0.5 * (4.0 * (parent.getVY() - windSpeed[1]) * vMag + Math.PI * radius * parent.getAngularV() * (parent.getVX() - windSpeed[0]));
        result[2] = -2.0 * radius * parent.getAngularV() * vMag;

        double multiplier = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * radius;
        result[0] *= multiplier;
        result[1] *= multiplier;
        result[2] *= multiplier;

        int N = 5;
        double initialTheta = Math.atan2((parent.getVY() - windSpeed[1]), (parent.getVX() - windSpeed[0])) - 0.5 * Math.PI;
        double interval = (0.5 * Math.PI) / (N - 1);
        //reuse multiplier as "clamping multiplier"
        multiplier = 1.0;
        for (int i = 0; i < N; i++) {
            double[] testPoint = new double[]{radius * Math.cos(initialTheta + i * interval), radius * Math.sin(initialTheta + i * interval)};
            //calculate the predicted change
            double oldVx = parent.getVX() + parent.getAngularV() * -testPoint[1] - windSpeed[0];
            double oldVy = parent.getVY() + parent.getAngularV() * testPoint[0] - windSpeed[1];
            double magnitudeSquared = oldVx * oldVx + oldVy * oldVy;

            double predictedChangeVx = (result[0] / parent.getMass() + (result[2] / parent.getInertia()) * -testPoint[1]) * dt;
            double predictedChangeVy = (result[1] / parent.getMass() + (result[2] / parent.getInertia()) * testPoint[0]) * dt;
            double dot = predictedChangeVx * oldVx + predictedChangeVy * oldVy;
            double clamping_multiplier;
            if (dot < -magnitudeSquared) {
                clamping_multiplier = -magnitudeSquared / dot;
                multiplier = Math.min(multiplier, clamping_multiplier);
            }
        }
        result[0] *= multiplier;
        result[1] *= multiplier;
        result[2] *= multiplier;

        return result;
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
