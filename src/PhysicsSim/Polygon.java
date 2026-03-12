package PhysicsSim;

import java.util.ArrayList;
import java.util.Arrays;

class Polygon extends GeometricType {
    private final ArrayList<Double> xPoints = new ArrayList<>();
    private final ArrayList<Double> yPoints = new ArrayList<>();
    public final ArrayList<ConvexPolygon> convexDecomposition = new ArrayList<>();
    private boolean invertNormals;
    private final String pointExclusionsCategorization;
    private final String convexDecompositionCategorization;

    public Polygon(double[] inputX, double[] inputY, double MTV_EPSILON) {
        super(Rigidbody.num);
        final ArrayList<Triangle> constructionTriangles = new ArrayList<>();

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
            constructionTriangles.add(new Triangle());
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
            boolean concave = validateAngleConcavity(mod(backwardIndex,length), mod(i, length), mod(forwardIndex, length));
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
                    constructionTriangles.set(mod(i,length), triangle);
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
            if (pointExclusions[i] == 3 && constructionTriangles.get(i).doesExist()) {
                constructionTriangles.get(i).calculateProperties(area);
                centerOfMassX = centerOfMassX + (constructionTriangles.get(i).getArea() / area) * constructionTriangles.get(i).getCenterX();
                centerOfMassY = centerOfMassY + (constructionTriangles.get(i).getArea() / area) * constructionTriangles.get(i).getCenterY();
            }
        }
        //moment of inertia
        for (int i = 0; i < length; i = i + 1) {
            if (constructionTriangles.get(i).doesExist()) {
                double squaredDistance = (constructionTriangles.get(i).getCenterX() - centerOfMassX) * (constructionTriangles.get(i).getCenterX() - centerOfMassX);
                squaredDistance = squaredDistance + (constructionTriangles.get(i).getCenterY() - centerOfMassY) * (constructionTriangles.get(i).getCenterY() - centerOfMassY);
                calcInertia = calcInertia + constructionTriangles.get(i).getMasslessInertia() + (constructionTriangles.get(i).getArea() / area) * squaredDistance;
            }
        }
        masslessInertia = calcInertia;

        //triangulation and properties calculation is now done. We can now proceed to merging triangles into convex polygons.
        for (int i = 0; i < constructionTriangles.size(); i++) {
            if (constructionTriangles.get(i).doesExist()) {
                convexDecomposition.add(new ConvexPolygon(constructionTriangles.get(i), this));
            }
            else convexDecomposition.add(new ConvexPolygon());
        }
        for (int i = 0; i < convexDecomposition.size(); i++) {
            if (!convexDecomposition.get(i).doesExist()) continue;
            ConvexPolygon currentConvex = convexDecomposition.get(i);
            //check all other convex polygons for shared edges
            boolean valid = false;
            for (int edgeIndex = 0; edgeIndex < currentConvex.size(); edgeIndex++) {
                int edgePointA0 = currentConvex.indices[mod(edgeIndex - 1, currentConvex.size())];
                int edgePointA1 = currentConvex.indices[edgeIndex];
                int edgePointB1 = currentConvex.indices[mod(edgeIndex + 1, currentConvex.size())];
                int edgePointB2 = currentConvex.indices[mod(edgeIndex + 2, currentConvex.size())];

                //find another polygon that shares the edge, if any
                ConvexPolygon option = new ConvexPolygon();
                for (int j = i + 1; j < convexDecomposition.size(); j++) {
                    if (convexDecomposition.get(j).doesExist()) {
                        option = convexDecomposition.get(j);
                        for (int optionEdgeIndex = 0; optionEdgeIndex < option.size(); optionEdgeIndex++) {
                            int optionPointA = option.indices[optionEdgeIndex];
                            int optionPointB = option.indices[(optionEdgeIndex + 1) % option.size()];
                            if (edgePointA1 == optionPointA && edgePointB1 == optionPointB) {
                                System.out.println("There is a winding order issue present.");
                            }
                            else if (edgePointA1 == optionPointB && edgePointB1 == optionPointA) {
                                int edgePointB0 = option.indices[mod(optionEdgeIndex - 1, option.size())];
                                int edgePointA2 = option.indices[mod(optionEdgeIndex + 2, option.size())];

                                //check if the proposed triangles A and B are concave. Triangles formed between two convex polygons cannot be self intersecting
                                //as a consequence of lack of concavity (which means the angles must be convex).
                                valid = !validateAngleConcavity(edgePointA0, edgePointA1, edgePointA2) && !validateAngleConcavity(edgePointB0, edgePointB1, edgePointB2);
                                if (valid) {
                                    i = i - 1;
                                    currentConvex.merge(option, edgePointA1, edgePointB1);
                                    convexDecomposition.set(j, new ConvexPolygon());
                                    break;
                                }
                            }
                        }
                        if (valid) break;
                    }
                }
                if (valid) break;
            }
        }

        for (int i = 0; i < length; i++) {
            if (i >= convexDecomposition.size()) break;
            ConvexPolygon convexPolygon = convexDecomposition.get(i);
            if (convexPolygon.doesExist()) {
                convexPolygon.calculateProperties(area);
                convexPolygon.shift(centerOfMassX, centerOfMassY);
            }
            else {
                convexDecomposition.remove(convexPolygon);
                i = i - 1;
            }
        }
        //shift all points in the polygon's coordinate plane such that the center of mass is the origin and find the largest squared distance
        for (int i = 0; i < length; i = i + 1) {
            xPoints.set(i, xPoints.get(i) - centerOfMassX);
            if (Double.isNaN(leftBoundBox) || xPoints.get(i) < leftBoundBox) leftBoundBox = xPoints.get(i);
            if (Double.isNaN(rightBoundBox) || xPoints.get(i) > rightBoundBox) rightBoundBox = xPoints.get(i);
            yPoints.set(i, yPoints.get(i) - centerOfMassY);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) < topBoundBox) topBoundBox = yPoints.get(i);
            if (Double.isNaN(topBoundBox) || yPoints.get(i) > bottomBoundBox) bottomBoundBox = yPoints.get(i);
            double temp = xPoints.get(i) * xPoints.get(i) + yPoints.get(i) * yPoints.get(i);
            if (temp > largestDistanceSquared) largestDistanceSquared = temp;
        }
        largestDistance = Math.sqrt(largestDistanceSquared);

        StringBuilder convexCategorization = new StringBuilder();
        convexCategorization.append("Convex Decomposition by edge index: {");
        for (int i = 0; i < convexDecomposition.size(); i++) {
            ConvexPolygon convexPolygon = convexDecomposition.get(i);
            convexCategorization.append(toAlphabetic(i));
            convexCategorization.append(": ");
            convexCategorization.append(Arrays.toString(convexPolygon.indices));
            if (i < convexDecomposition.size() - 1) convexCategorization.append(", ");
        }
        convexCategorization.append("}");
        this.convexDecompositionCategorization = convexCategorization.toString();

    }
    private static String toAlphabetic(int i) {
        if(i < 0) {
            return "-" + toAlphabetic(-i-1);
        }

        int quot = i/26;
        int rem = i%26;
        char letter = (char)((int)'A' + rem);
        if( quot == 0 ) {
            return ""+letter;
        } else {
            return toAlphabetic(quot-1) + letter;
        }
    }

    private boolean isOnNormalSide(double[] point, int index1, int index2) {
        double dotProduct = (yPoints.get(index1) - point[1]) * (xPoints.get(index2) - xPoints.get(index1));
        dotProduct = dotProduct - (xPoints.get(index1) - point[0]) * (yPoints.get(index2) - yPoints.get(index1));
        return (dotProduct > 0.0 && invertNormals) || (dotProduct < 0.0 && !invertNormals);
    }
    private boolean validateAngleConcavity(int index1, int index2, int index3) {
        double[] center = new double[]{(xPoints.get(index1) + xPoints.get(index2) + xPoints.get(index3)) / 3.0,
                (yPoints.get(index1) + yPoints.get(index2) + yPoints.get(index3)) / 3.0};
        return isOnNormalSide(center,index1, index2) || isOnNormalSide(center,index2, index3) || isOnNormalSide(center, index3, index1);
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
    public String getConvexCategorization() {
        return convexDecompositionCategorization;
    }
    public void setFace(int edgeIndex) {
        for (ConvexPolygon option : convexDecomposition) {
            for (int i = 0; i < option.indices.length; i++) {
                //make sure the edgeIndex matches and it is an external edge.
                if (option.indices[i] == edgeIndex && option.indices[mod(i + 1, option.indices.length)] == mod(option.indices[i] + 1, xPoints.size())) {
                    option.faceEdgeIndex = i;
                    break;
                }
            }
        }
    }

    @Override
    boolean findCollisions(GeometricType otherGeometry) {
        Rigidbody otherObject = Rigidbody.get(otherGeometry.getParentRigidbodyID());
        Rigidbody myObject = Rigidbody.get(getParentRigidbodyID());
        if (otherObject.simID != myObject.simID) return(false);
        if (otherGeometry instanceof Polygon) {
            boolean intersecting = false;
            for (ConvexPolygon myConvex : convexDecomposition) {
                for (ConvexPolygon otherConvex : ((Polygon) otherGeometry).convexDecomposition) {
                    Triplet results = myConvex.checkCollisions(otherConvex);
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
        else if (otherGeometry instanceof Circle other) {
            boolean intersecting = false;
            for (ConvexPolygon myConvex : convexDecomposition) {
                Triplet results = myConvex.checkCollisions(other);
                if (results.getFirstBoolean()) {
                    intersecting = true;
                    myObject.contactPoints.add(results.getThirdDoubleArrayReference());
                    myObject.MTVs.add(results.getSecondDoubleArrayReference());
                    myObject.collidingIDs.add(otherObject.ID);
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
    boolean findCollisions(Joint solidJoint) {
        double lineThickness = Simulation.get(solidJoint.parent.simID).solidJointCollisionLineThickness;
        lineThickness = Math.min(Math.min(lineThickness, solidJoint.parent.geometry.getLargestDistance()), solidJoint.connection.geometry.getLargestDistance());
        double x1 = solidJoint.parent.getPosX() + solidJoint.offsetFromCMParent[0];
        double x2 = solidJoint.connection.getPosX() + solidJoint.offsetFromCMOther[0];
        double y1 = solidJoint.parent.getPosY() + solidJoint.offsetFromCMParent[1];
        double y2 = solidJoint.connection.getPosY() + solidJoint.offsetFromCMOther[1];
        double magnitude = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        double n1 = -(y2 - y1) / magnitude;
        double n2 = (x2 - x1) / magnitude;
        double[] nX = new double[]{n1, n2, -n1, -n2};
        double[] nY = new double[]{n2, -n1, -n2, n1};
        double[] posX = new double[]{x1 + n1 * lineThickness, x2 + n1 * lineThickness, x2 - n1 * lineThickness, x1 - n1 * lineThickness};
        double[] posY = new double[]{y1 + n2 * lineThickness, y2 + n2 * lineThickness, y2 - n2 * lineThickness, y1 - n2 * lineThickness};
        boolean intersecting = false;
        Rigidbody myObject = Rigidbody.get(getParentRigidbodyID());
        for (ConvexPolygon myConvex : convexDecomposition) {
            Triplet results = myConvex.checkCollisions(posX, posY, nX, nY,
                    solidJoint.parent.getCompoundMass() + solidJoint.connection.getCompoundMass(), !solidJoint.parent.isMovable() && !solidJoint.connection.isMovable());
            if (results.getFirstBoolean()) {
                intersecting = true;
                myObject.contactPoints.add(results.getThirdDoubleArrayReference());
                myObject.contactPoints.add(new Double[]{Double.NaN, 0.0});
                myObject.MTVs.add(results.getSecondDoubleArrayReference());
                myObject.MTVs.add(results.getSecondDoubleArrayReference());
                myObject.collidingIDs.add(-solidJoint.parent.ID - 2);
                myObject.collidingIDs.add(-solidJoint.connection.ID - 2);
            }
        }
        return(false);
    }

    @Override
    boolean checkForCollisionsWall() {
        Rigidbody myObject = getParent();
        Simulation sim = myObject.sim;
        double posX = myObject.getPosX();
        double posY = myObject.getPosY();
        double worldBottomBound = sim.worldBottomBound;
        double worldTopBound = sim.worldTopBound;
        double worldLeftBound = sim.worldLeftBound;
        double worldRightBound = sim.worldRightBound;
        boolean intersecting = false;

        for (int i = 0; i < xPoints.size(); i = i + 1) {
            double x = xPoints.get(i) + posX;
            double y = yPoints.get(i) + posY;
            int nextIndex = mod(i + 1, xPoints.size());
            if (y >= worldBottomBound) {
                myObject.MTVs.add(new Double[]{0.0, worldBottomBound - y - sim.MTV_EPSILON});
                myObject.collidingIDs.add(-1);
                if (yPoints.get(nextIndex) + posY >= worldBottomBound) {
                    myObject.contactPoints.add(new Double[]{0.5 * (x + xPoints.get(nextIndex) + posX), 0.5 * (y + yPoints.get(nextIndex) + posY)});
                    i++;
                }
                else myObject.contactPoints.add(new Double[]{x, y});
                intersecting = true;
            }
            if (y <= worldTopBound) {
                myObject.MTVs.add(new Double[]{0.0, worldTopBound - y + sim.MTV_EPSILON});
                myObject.collidingIDs.add(-1);
                if (yPoints.get(nextIndex) + posY <= worldBottomBound) {
                    myObject.contactPoints.add(new Double[]{0.5 * (x + xPoints.get(nextIndex) + posX), 0.5 * (y + yPoints.get(nextIndex) + posY)});
                    i++;
                }
                else myObject.contactPoints.add(new Double[]{x, y});
                intersecting = true;
            }
            if (x <= worldLeftBound) {
                myObject.MTVs.add(new Double[]{worldLeftBound - x + sim.MTV_EPSILON, 0.0});
                myObject.collidingIDs.add(-1);
                if (xPoints.get(nextIndex) + posY <= worldLeftBound) {
                    myObject.contactPoints.add(new Double[]{0.5 * (x + xPoints.get(nextIndex) + posX), 0.5 * (y + yPoints.get(nextIndex) + posY)});
                    i++;
                }
                else myObject.contactPoints.add(new Double[]{x, y});
                intersecting = true;
            }
            if (x >= worldRightBound) {
                myObject.MTVs.add(new Double[]{worldRightBound - x - sim.MTV_EPSILON, 0.0});
                myObject.collidingIDs.add(-1);
                if (xPoints.get(nextIndex) + posX >= worldRightBound) {
                    myObject.contactPoints.add(new Double[]{0.5 * (x + xPoints.get(nextIndex) + posX), 0.5 * (y + yPoints.get(nextIndex) + posY)});
                    i++;
                }
                else myObject.contactPoints.add(new Double[]{x, y});
                intersecting = true;
            }
        }
        return(intersecting);
    }

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
        }
        for (ConvexPolygon part : convexDecomposition) part.rotate(theta);
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
        if (Double.isNaN(magnitude1)) {
            return new double[]{0.0, 0.0, 0.0};
        }
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
        //the 10.0 here is arbitrary, and is a stand-in to prevent quick changes in velocity (impulses) from causing
        //the simplified drag force from "over-correcting"
        magnitude2 = Math.min(Rigidbody.get(getParentRigidbodyID()).getMass() * magnitude1 * 10.0, magnitude2);
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
        if (!Double.isFinite(fD[0]) || !Double.isFinite(fD[1]) || !Double.isFinite(torqueSum)) return new double[]{0.0, 0.0, 0.0};

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
