package PhysicsSim;

import java.util.ArrayList;

class Polygon extends GeometricType {
    private final ArrayList<Double> xPoints = new ArrayList<>();
    private final ArrayList<Double> yPoints = new ArrayList<>();
    public final ArrayList<Triangle> triangles = new ArrayList<>();
    private boolean invertNormals;
    private final String pointExclusionsCategorization;

    public Polygon(double[] inputX, double[] inputY, double MTV_EPSILON) {
        super(Rigidbody.num);

        int length = Math.min(inputX.length, inputY.length);
        for (int i = 0; i < length; i = i + 1) {
            xPoints.add(inputX[i]);
            yPoints.add(inputY[i]);
        }
        double totalArea = 0.0; //the signed area of the polygon to determine the winding order
        for (int i = 0; i < length; i = i + 1) {
            //find the signed area of the trapezoid
            double trapezoidArea = 0.5 * (xPoints.get(mod(i + 1, length)) - xPoints.get(i)) * (yPoints.get(mod(i + 1, length)) + yPoints.get(i));
            //add to the total signed area of the polygon
            totalArea = totalArea + trapezoidArea;
        }
        if (totalArea < 0.0) invertNormals = true;
        area = Math.abs(totalArea);

        //create a list of triangles in which each one is unassigned
        for (int i = 0; i < length; i = i + 1) {
            triangles.add(new Triangle());
        }

        //begin earmarking triangulation process.
        int numTriangles = 0;
        int[] pointExclusions = new int[length]; //3 is covered (excluded), 2 is convex but invalid (excluded as main), 1 is concave (excluded as main), and 0 is not investigated yet
        for (int i = 0; i < 10 * length; i = i + 1) {
            //find a suitable point in front of and behind the point in question in the order for attempted triangulation
            if (numTriangles == length - 2) break;
            for (int j = 0; j < length; j = j + 1) {
                if (pointExclusions[mod(i + j,length)] != 3) {
                    i = i + j;
                    break;
                }
            }
            int backwardIndex = mod(i,length);
            int forwardIndex = mod(i,length);
            for (int j = 1; j < length; j = j + 1) {
                if (pointExclusions[mod(i - j, length)] != 3) {
                    backwardIndex = mod(i - j, length);
                    break;
                }
            }
            for (int j = 1; j < length; j = j + 1) {
                if (pointExclusions[mod(i + j, length)] != 3) {
                    forwardIndex = mod(i + j, length);
                    break;
                }
            }
            //check if the point in question has a concave angle in the proposed triangle. Check normals on centroid
            double[] center = new double[]{(xPoints.get(mod(i,length)) + xPoints.get(mod(forwardIndex,length)) + xPoints.get(mod(backwardIndex,length))) / 3.0, (yPoints.get(mod(i,length)) + yPoints.get(mod(forwardIndex,length)) + yPoints.get(mod(backwardIndex,length))) / 3.0};
            boolean concave = false;
            concave = isOnNormalSide(center,mod(backwardIndex, length), mod(i,length));
            if (!concave) {
                concave = isOnNormalSide(center,mod(i,length),mod(forwardIndex, length));
            }
            if (!concave) {
                concave = isOnNormalSide(center,mod(forwardIndex, length),mod(backwardIndex, length));
            }
            //if concave, then mark it and move on to the next point.
            if (concave) pointExclusions[mod(i,length)] = 1;
            else {
                //check if any other polygon points are within the proposed triangle to determine the triangle's validity
                boolean valid = true;
                for (int j = 0; j < length; j = j + 1) {
                    if (j != mod(i,length) && j != mod(backwardIndex, length) && j != mod(forwardIndex, length) && pointExclusions[j] != 3) { //all checked points have to not be part of the triangle and not already covered
                        double[] point = new double[]{xPoints.get(j), yPoints.get(j)};
                        boolean validPoint = isOnNormalSide(point, mod(backwardIndex, length), mod(i,length));
                        if (!validPoint) validPoint = isOnNormalSide(point, mod(i,length), mod(forwardIndex,length));
                        if (!validPoint) validPoint = isOnNormalSide(point, mod(forwardIndex,length), mod(backwardIndex,length));
                        if (!validPoint) {
                            valid = false;
                            pointExclusions[i] = 2; //2 means the point in question is convex, but invalid because it contains another polygon point in the proposed triangle
                            break;
                        }
                    }
                }
                if (valid) { //here, the proposed triangle has been verified to be both within the polygon (concave test)
                    // and to not contain any other polygon points inside it, so as to not be self-intersecting
                    //Save as a triangle and mark point as covered (3)
                    pointExclusions[mod(i,length)] = 3; //it means this point is the parent point of a triangle that is now removed from the polygon in the triangulation process
                    Triangle triangle = new Triangle(mod(backwardIndex, length), mod(i,length), mod(forwardIndex,length), this, center);
                    triangle.setMTVEpsilon(MTV_EPSILON);
                    triangles.set(mod(i,length), triangle);
                    numTriangles = numTriangles + 1;
                }
            }
        }
        StringBuilder pointExclusionsCategorization = new StringBuilder();
        //3 is covered (excluded), 2 is convex but invalid (excluded as main), 1 is concave (excluded as main), and 0 is not investigated yet
        pointExclusionsCategorization.append("Triangulation Categorization: [");
        for (int i = 0; i < pointExclusions.length; i = i + 1) {
            switch (pointExclusions[i]) {
                case (0): {
                    pointExclusionsCategorization.append("Skipped");
                    break;
                }
                case (1): {
                    pointExclusionsCategorization.append("Concave (not skipped, invalid)");
                    break;
                }
                case (2): {
                    pointExclusionsCategorization.append("Convex (not skipped, self-intersecting -> invalid)");
                    break;
                }
                case (3): {
                    pointExclusionsCategorization.append("Valid ear vertex (convex, not self-intersecting -> valid)");
                    break;
                }
            }
            if (i < pointExclusions.length - 1) pointExclusionsCategorization.append(" | ");
            else pointExclusionsCategorization.append("]");
        }
        this.pointExclusionsCategorization = pointExclusionsCategorization.toString();


        //calculate properties of polygon
        //center of mass
        double centerOfMassX = 0.0;
        double centerOfMassY = 0.0;
        double calcInertia = 0.0;
        //center of mass
        for (int i = 0; i < length; i = i + 1) {
            if (pointExclusions[i] == 3 && triangles.get(i).doesExist()) {
                triangles.get(i).calculateProperties(area);
                centerOfMassX = centerOfMassX + (triangles.get(i).getArea() / area) * triangles.get(i).getCenterX();
                centerOfMassY = centerOfMassY + (triangles.get(i).getArea() / area) * triangles.get(i).getCenterY();
            }
        }
        //moment of inertia
        for (int i = 0; i < length; i = i + 1) {
            if (triangles.get(i).doesExist()) {
                double squaredDistance = (triangles.get(i).getCenterX() - centerOfMassX) * (triangles.get(i).getCenterX() - centerOfMassX);
                squaredDistance = squaredDistance + (triangles.get(i).getCenterY() - centerOfMassY) * (triangles.get(i).getCenterY() - centerOfMassY);
                calcInertia = calcInertia + triangles.get(i).getMasslessInertia() + (triangles.get(i).getArea() / area) * squaredDistance;
            }
        }
        masslessInertia = calcInertia;
        //shift all points in the polygon's coordinate plane such that the center of mass is the origin and find the largest squared distance
        for (int i = 0; i < length; i = i + 1) {
            xPoints.set(i, xPoints.get(i) - centerOfMassX);
            if (Double.isNaN(leftBoundBox) || xPoints.get(i) < leftBoundBox) leftBoundBox = xPoints.get(i);
            if (Double.isNaN(rightBoundBox) || xPoints.get(i) > rightBoundBox) rightBoundBox = xPoints.get(i);
            yPoints.set(i, yPoints.get(i) - centerOfMassY);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) < topBoundBox) topBoundBox = yPoints.get(i);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) > bottomBoundBox) bottomBoundBox = yPoints.get(i);
            triangles.get(i).shift(centerOfMassX, centerOfMassY);
            double temp = xPoints.get(i) * xPoints.get(i) + yPoints.get(i) * yPoints.get(i);
            if (temp > largestDistanceSquared) largestDistanceSquared = temp;
        }
        largestDistance = Math.sqrt(largestDistanceSquared);
    }

    private boolean isOnNormalSide(double[] point, int index1, int index2) {
        double dotProduct = (yPoints.get(index1) - point[1]) * (xPoints.get(index2) - xPoints.get(index1));
        dotProduct = dotProduct - (xPoints.get(index1) - point[0]) * (yPoints.get(index2) - yPoints.get(index1));
        return (dotProduct > 0.0 && invertNormals) || (dotProduct < 0.0 && !invertNormals);
    }
    public double getXPoints(int index) {
        return(xPoints.get(index));
    }
    public double getYPoints(int index) {
        return(yPoints.get(index));
    }

    public String getTriangulationCategorization() {
        return pointExclusionsCategorization;
    }
    public void setTriangle(int triangleIndex, int edgeIndex, boolean isFace) {
        Triangle triangle = triangles.get(triangleIndex);
        triangle.enabled[edgeIndex] = isFace;
        triangle.partOfFace = !triangle.enabled[0] || !triangle.enabled[1] || !triangle.enabled[2];
    }

    @Override
    boolean findCollisions(GeometricType otherGeometry) {
        Rigidbody otherObject = Rigidbody.get(otherGeometry.getParentRigidbodyID());
        Rigidbody myObject = Rigidbody.get(getParentRigidbodyID());
        if (otherGeometry instanceof Polygon) {
            Polygon other = (Polygon) otherGeometry;
            if (!(otherObject.simID == myObject.simID && (myObject.isHitbox || !otherObject.isHitbox))) return(false);
            boolean intersecting = false;
            for (int i = 0; i < xPoints.size(); i = i + 1) {
                for (int j = 0; j < other.getNumPoints(); j = j + 1) {
                    if (triangles.get(i).doesExist() && other.triangles.get(j).doesExist()){
                        Triplet results = triangles.get(i).checkCollisions(other.triangles.get(j));
                        if (results.getFirstBoolean()) {
                            intersecting = true;
                            myObject.contactPoints.add(results.getThirdDoubleArrayReference());
                            myObject.MTVs.add(results.getSecondDoubleArrayReference());
                            myObject.collidingIDs.add(otherObject.ID);
                        }
                    }
                }
            }
            return(intersecting);
        }
        else if (otherGeometry instanceof Circle) {
            Circle other = (Circle) otherGeometry;
            if (!(otherObject.simID == myObject.simID && (myObject.isHitbox || !otherObject.isHitbox))) return(false);
            boolean intersecting = false;
            for (int i = 0; i < xPoints.size(); i = i + 1) {
                if (triangles.get(i).doesExist()) {
                    Triplet results = triangles.get(i).checkCollisions(other);
                    if (results.getFirstBoolean()) {
                        intersecting = true;
                        myObject.contactPoints.add(results.getThirdDoubleArrayReference());
                        myObject.MTVs.add(results.getSecondDoubleArrayReference());
                        myObject.collidingIDs.add(otherObject.ID);
                    }
                }
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
        Rigidbody rigidbody = Rigidbody.get(getParentRigidbodyID());
        boolean intersecting = false;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            Triplet results = softbody.resolvePointInside(new double[]{xPoints.get(i) + rigidbody.getPosX(), yPoints.get(i) + rigidbody.getPosY()}, 0.0);
            if (results.getFirstBoolean()) {
                Double[] MTV = new Double[2];
                double multiplier = (Rigidbody.getMass(results.getThirdInt()) / (Rigidbody.getMass(results.getThirdInt()) + rigidbody.getMass()));
                MTV[0] = results.getSecondDoubleArray()[0] * multiplier;
                MTV[1] = results.getSecondDoubleArray()[1] * multiplier;
                Double[] contactPoint = new Double[]{xPoints.get(i) + rigidbody.getPosX() + MTV[0], yPoints.get(i) + rigidbody.getPosY() + MTV[1]};
                rigidbody.contactPoints.add(contactPoint);
                rigidbody.MTVs.add(MTV);
                rigidbody.collidingIDs.add(results.getThirdInt());
                intersecting = true;
            }
        }
        return(intersecting);
    }

    @Override
    boolean checkForCollisionsWall() {
        double left = 0.0;
        double right = 0.0;
        double bottom = 0.0;
        double top = 0.0;
        Rigidbody myObject = Rigidbody.get(getParentRigidbodyID());
        double posX = myObject.getPosX();
        double posY = myObject.getPosY();
        Simulation sim = Simulation.get(myObject.simID);
        double worldBottomBound = sim.worldBottomBound;
        double worldTopBound = sim.worldTopBound;
        double worldLeftBound = sim.worldLeftBound;
        double worldRightBound = sim.worldRightBound;
        boolean intersecting = false;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            double x = xPoints.get(i) + posX;
            double y = yPoints.get(i) + posY;
            if (y >= worldBottomBound) {
                if (Math.abs(worldBottomBound - y) > Math.abs(bottom)){
                    bottom = worldBottomBound - y;
                    myObject.MTVs.add(new Double[]{0.0, bottom - sim.MTV_EPSILON});
                    myObject.contactPoints.add(new Double[]{x, worldBottomBound});
                    myObject.collidingIDs.add(-1);
                    intersecting = true;
                }
            }
            if (y <= worldTopBound) {
                if (Math.abs(worldTopBound - y) > Math.abs(top)){
                    top = worldTopBound - y;
                    myObject.MTVs.add(new Double[]{0.0, top + sim.MTV_EPSILON});
                    myObject.contactPoints.add(new Double[]{x, worldTopBound});
                    myObject.collidingIDs.add(-1);
                    intersecting = true;
                }
            }
            if (x <= worldLeftBound) {
                if (Math.abs(worldLeftBound - x) > Math.abs(left)){
                    left = worldLeftBound - x;
                    myObject.MTVs.add(new Double[]{left + sim.MTV_EPSILON, 0.0});
                    myObject.contactPoints.add(new Double[]{worldLeftBound, y});
                    myObject.collidingIDs.add(-1);
                    intersecting = true;
                }
            }
            if (x >= worldRightBound) {
                if (Math.abs(worldRightBound - x) > Math.abs(right)){
                    right = worldRightBound - x;
                    myObject.MTVs.add(new Double[]{right - sim.MTV_EPSILON, 0.0});
                    myObject.contactPoints.add(new Double[]{worldRightBound, y});
                    myObject.collidingIDs.add(-1);
                    intersecting = true;
                }
            }
        }
        return(intersecting);
    }
    /*boolean checkForCollisionsNarrow(Softbody softbody) {
        boolean intersecting = false;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            Triplet results = softbody.resolvePointInside(new double[]{xPoints.get(i) + posX, yPoints.get(i) + posY}, 0.0);
            if (results.getFirstBoolean()) {
                Double[] MTV = new Double[2];
                double multiplier = (getMass(results.getThirdInt()) / (getMass(results.getThirdInt()) + mass));
                MTV[0] = results.getSecondDoubleArray()[0] * multiplier;
                MTV[1] = results.getSecondDoubleArray()[1] * multiplier;
                Double[] contactPoint = new Double[]{xPoints.get(i) + posX + MTV[0], yPoints.get(i) + posY + MTV[1]};
                contactPoints.add(contactPoint);
                MTVs.add(MTV);
                collidingIDs.add(results.getThirdInt());
                intersecting = true;
            }
        }
        return(intersecting);
    }*/

    public int getNumPoints() {
        if (xPoints.size() == yPoints.size()) return(xPoints.size());
        else {
            System.out.println("xPoints and yPoints do not match in length");
            return (-1);
        }
    }

    @Override
    void rotateAroundCenter(double theta) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        leftBoundBox = Double.NaN;
        rightBoundBox = Double.NaN;
        topBoundBox = Double.NaN;
        bottomBoundBox = Double.NaN;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            double x = xPoints.get(i);
            double y = yPoints.get(i);
            xPoints.set(i, x * cos - y * sin);
            if (Double.isNaN(leftBoundBox) || xPoints.get(i) < leftBoundBox) {
                leftBoundBox = xPoints.get(i);
            }
            if (Double.isNaN(rightBoundBox) || xPoints.get(i) > rightBoundBox) {
                rightBoundBox = xPoints.get(i);
            }
            yPoints.set(i, y * cos + x * sin);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) < topBoundBox) {
                topBoundBox = yPoints.get(i);
            }
            if (Double.isNaN(bottomBoundBox) || yPoints.get(i) > bottomBoundBox) {
                bottomBoundBox = yPoints.get(i);
            }
            if (triangles.get(i).doesExist()) {
                triangles.get(i).rotate(theta);
            }
        }
    }

    @Override
    boolean pointInside(double[] point) {
        double[] testPoint = new double[]{point[0] - Rigidbody.get(getParentRigidbodyID()).getPosX(), point[1] - Rigidbody.get(getParentRigidbodyID()).getPosY()};
        int raycastCount = 0;
        for (int i = 0; i < xPoints.size(); i = i + 1) {
            double minX = xPoints.get(i);
            int minXindex = i;
            double maxX = xPoints.get((i + 1) % xPoints.size());
            int maxXindex = (i + 1) % xPoints.size();
            if (maxX < minX) {
                double temp = minX;
                int tempIndex = minXindex;
                minX = maxX;
                minXindex = maxXindex;
                maxX = temp;
                maxXindex = tempIndex;
            }
            if (testPoint[0] > minX && testPoint[0] < maxX && ((yPoints.get(maxXindex) - yPoints.get(minXindex)) / (maxX - minX)) * (testPoint[0] - maxX) + yPoints.get(maxXindex) > testPoint[1]) {
                raycastCount = raycastCount + 1;
            }
        }
        return (raycastCount % 2 == 1);
    }

    @Override
    double[] calculateAirResistance(double AIR_DENSITY, double DRAG_COEFFICIENT, double[] v) {
        double[] fD = new double[]{0.0, 0.0};
        double torqueSum = 0.0;
        double vX = v[0];
        double vY = v[1];
        int length = xPoints.size();
        double magnitude1 = Math.sqrt(vX * vX + vY * vY);
        if (Double.isNaN(magnitude1)) magnitude1 = 0.0001;
        double uX = vX / magnitude1;
        double uY = vY / magnitude1;
        if (Double.isNaN(uX)) uX = 0.0;
        if (Double.isNaN(uY)) uY = 0.0;
        double crossArea = 0.0;
        for (int i = 0; i < length; i = i + 1) {
            double nX = -(yPoints.get((i + 1) % length) - yPoints.get(i));
            double nY = xPoints.get((i + 1) % length) - xPoints.get(i);
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            if (invertNormals) {
                nX = -nX;
                nY = -nY;
            }
            if (nX * uX + nY * uY >= 0.0) {
                crossArea = crossArea + Math.abs((xPoints.get((i + 1) % length) - xPoints.get(i)) * -uY + (yPoints.get((i + 1) % length) - yPoints.get(i)) * uX);
            }
        }
        double magnitude2 = 0.5 * AIR_DENSITY * crossArea * DRAG_COEFFICIENT * magnitude1 * magnitude1;
        fD = new double[]{-uX * magnitude2, -uY * magnitude2};
        for (int i = 0; i < length; i = i + 1) {
            double rPerpX = -yPoints.get(i);
            double rPerpY = xPoints.get(i);
            double nX = -(yPoints.get((i + 1) % length) - yPoints.get(i));
            double nY = xPoints.get((i + 1) % length) - xPoints.get(i);
            if (Double.isNaN(nX) || Double.isNaN(nY)) {
                nX = 0.0;
                nY = 0.0;
            }
            if (invertNormals) {
                nX = -nX;
                nY = -nY;
            }
            if (nX * uX + nY * uY >= 0.0) {
                double torque = 0.5 * Math.abs((xPoints.get((i + 1) % length) - xPoints.get(i)) * -uY + (yPoints.get((i + 1) % length) - yPoints.get(i)) * uX) / crossArea;
                double perpForce1 = (fD[0] * rPerpX + fD[1] * rPerpY);
                double rPerpX2 = -yPoints.get((i + 1) % length);
                double rPerpY2 = xPoints.get((i + 1) % length);
                double perpForce2 = (fD[0] * rPerpX2 + fD[1] * rPerpY2);
                torque = torque * (perpForce1 + perpForce2);
                torqueSum = torqueSum + torque;
            }
        }

        return(new double[]{fD[0], fD[1], torqueSum});
    }

    @Override
    Triplet getDrawInt(double shiftX, double resolutionCenterX, double pixelShiftX,
                       double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling) {
        double posX = Rigidbody.get(getParentRigidbodyID()).getPosX();
        double posY = Rigidbody.get(getParentRigidbodyID()).getPosY();
        int[] x = new int[xPoints.size()];
        int[] y = new int[yPoints.size()];
        for (int i = 0; i < getNumPoints(); i = i + 1) {
            x[i] = (int) Math.round(((xPoints.get(i) + posX - shiftX - resolutionCenterX) / resolutionScaling) + resolutionCenterX + pixelShiftX);
            y[i] = (int) Math.round(((yPoints.get(i) + posY - shiftY - resolutionCenterY) / resolutionScaling) + resolutionCenterY + pixelShiftY);
        }
        return new Triplet(RigidbodyGeometries.Polygon, x, y);
    }

    @Override
    Triplet getDrawDoubles(double shiftX, double resolutionCenterX, double pixelShiftX, double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling) {
        double posX = Rigidbody.get(getParentRigidbodyID()).getPosX();
        double posY = Rigidbody.get(getParentRigidbodyID()).getPosY();
        double[] x = new double[xPoints.size()];
        double[] y = new double[yPoints.size()];
        for (int i = 0; i < getNumPoints(); i = i + 1) {
            x[i] = ((xPoints.get(i) + posX - shiftX - resolutionCenterX) / resolutionScaling) + resolutionCenterX + pixelShiftX;
            y[i] = ((yPoints.get(i) + posY - shiftY - resolutionCenterY) / resolutionScaling) + resolutionCenterY + pixelShiftY;
        }
        return new Triplet(RigidbodyGeometries.Polygon, x, y);
    }
}
