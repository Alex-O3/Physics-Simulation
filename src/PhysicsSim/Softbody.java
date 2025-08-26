package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

class Softbody {
    public int simID;

    protected final double mass;
    private final double initialArea;
    private final Color color;
    private final double pointRadius;
    private final ArrayList<Point> members = new ArrayList<>();
    protected ArrayList<Integer> boundaryMembers = new ArrayList<>();
    private static final ArrayList<Softbody> softbodies = new ArrayList<>();
    private final ArrayList<Double[]> idealShapeVectors = new ArrayList<>();
    public static int num = 0;
    public final int ID;

    public double largestSquaredDistance;
    public double minX;
    public double maxX;
    public double minY;
    public double maxY;
    public double[] cM;

    private final double initialPressure;
    private final boolean hasPressure;
    private final boolean isShapeMatch;
    public boolean boundaryCollision = true;
    public double MTV_EPSILON = 0.01625;
    public double SHAPE_MATCH_STRENGTH = 100.0;
    public double SHAPE_MATCH_DAMPING = 0.25;
    public double DAMP_COEFFICIENT = 0.6;

    public Softbody(int type, double[] borderX, double[] borderY, double[] movingMotion, double pointRadius, double targetDensity, double stiffness, double initialPressure, double mass, Color color, double max_dist_multiplier, double min_dist_multiplier, int simID) {
        this.simID = simID;

        //find the distance between points from the target density (points per 50 x 50 area)
        double r = 50.0 * Math.sqrt(1.0 / targetDensity);
        this.hasPressure = type == 1;
        this.isShapeMatch = type == 2 || type == 3;
        if (!(type >= 0 && type <= 3)) System.out.println("Incorrect softbody type. Must be 0, 1, 2, or 3 for spring-mass, pressure-spring, or shape-matching (solid or hollow) respectively.");
        //types == 0 means spring-mass, 1 means spring-pressure border, and 2 means shape-match
        ID = num;
        num = num + 1;
        softbodies.add(this);
        this.mass = mass;
        this.color = color;
        this.pointRadius = pointRadius;
        this.initialPressure = initialPressure;
        //find area to find normals
        boolean invertNormals = false;
        double totalSignedArea = 0.0;
        for (int i = 0; i < borderX.length; i = i + 1) {
            double smallArea = 0.5 * (borderX[(i + 1) % borderX.length] - borderX[i]) * (borderY[(i + 1) % borderX.length] + borderY[i]);
            totalSignedArea = totalSignedArea + smallArea;
        }
        if (totalSignedArea < 0.0) invertNormals = true;
        initialArea = Math.abs(totalSignedArea);


        if (type == 0 || type == 2) {
            double cX = 0.0;
            double cY = 0.0;
            for (int i = 0; i < borderX.length; i = i + 1) {
                cX = cX + borderX[i];
                cY = cY + borderY[i];
            }
            cX = cX / borderX.length;
            cY = cY / borderX.length;

            double[] pointTest = new double[]{cX, cY};
            //check if this average point is inside the defining shape
            int count = 0;
            for (int i = 0; i < borderX.length; i = i + 1) {
                double minX = borderX[i];
                double maxX = borderX[(i + 1) % borderX.length];
                if (maxX < minX) {
                    minX = maxX;
                    maxX = borderX[i];
                }
                double minY = borderY[i];
                double maxY = borderY[(i + 1) % borderX.length];
                if (maxY < minY) {
                    minY = maxY;
                    maxY = borderY[i];
                }
                if (pointTest[0] > minX && pointTest[0] < maxX) {
                    if (pointTest[1] < minY) count = count + 1;
                    else if (borderX[(i + 1) % borderX.length] - borderX[i] != 0.0) {
                        double m = (borderY[(i + 1) % borderX.length] - borderY[i]) / (borderX[(i + 1) % borderX.length] - borderX[i]);
                        double b = borderY[i] - m * (borderX[i]);
                        if (m * pointTest[0] + b > pointTest[1]) count = count + 1;
                    }
                }
            }
            //if the point is not inside, then put it on the closest edge point
            boolean onBoundary = false;
            if (count % 2 == 0) {

                for (int i = 0; i < borderX.length; i = i + 1) {
                    double nX = -(borderY[(i + 1) % borderX.length] - borderY[i]);
                    double nY = borderX[(i + 1) % borderX.length] - borderX[i];
                    double magnitude = Math.sqrt(nX * nX + nY * nY);
                    nX = nX / magnitude;
                    nY = nY / magnitude;
                    if (invertNormals) {
                        nX = -nX;
                        nY = -nY;
                    }
                    double orthoDot = pointTest[0] * -nY + pointTest[1] * nX;
                    double orthoDotMin = borderX[i] * -nY + borderY[i] * nX;
                    double orthoDotMax = borderX[(i + 1) % borderX.length] * -nY + borderY[(i + 1) % borderX.length] * nX;
                    if (orthoDotMax < orthoDotMin) {
                        double temp = orthoDotMin;
                        orthoDotMin = orthoDotMax;
                        orthoDotMax = temp;
                    }
                    if (orthoDot < orthoDotMax && orthoDot > orthoDotMin) {
                        double dot = pointTest[0] * nX + pointTest[1] * nY;
                        if (dot > 0.0) {
                            pointTest[0] = pointTest[0] - dot * nX;
                            pointTest[1] = pointTest[1] - dot * nY;
                            onBoundary = true;
                        }
                    }

                }
            }

            //create a point at the proposed location as the seed from which the lattice triangular structure of the softbody will be generated
            Point seedPoint = new Point(new double[]{pointTest[0], pointTest[1], 0.0, 0.0, 0.0, 0.0}, pointRadius, 1.0, color, true, simID);
            Simulation.get(simID).physicsObjects.add(new PhysicsObject(seedPoint));
            addMember(seedPoint, onBoundary);

            //start the lattice structure generation process
            seedPoint.generatePoints(0.0, r, borderX, borderY, invertNormals);
        }
        else {
            //create border points
            int lastIndex = -1;
            int firstIndex = -1;
            for (int j = 0; j < borderX.length; j = j + 1) {
                double dx = borderX[(j + 1) % borderX.length] - borderX[j];
                double dy = borderY[(j + 1) % borderX.length] - borderY[j];
                double distance = Math.sqrt(dx * dx + dy * dy);
                int iterations = (int) (distance / r);
                for (int i = 1; i < iterations; i = i + 1) {
                    double x = (dx / iterations) * i + borderX[j];
                    double y = (dy / iterations) * i + borderY[j];
                    Color a = color;
                    Point generatedPoint = new Point(new double[]{x, y, 0.0, 0.0, 0.0, 0.0}, pointRadius, 1.0, a, true, simID);
                    Simulation.get(simID).physicsObjects.add(new PhysicsObject(generatedPoint));
                    addMember(generatedPoint, true);
                    if (lastIndex != -1) Point.get(lastIndex).attach(generatedPoint);
                    else firstIndex = generatedPoint.ID;
                    lastIndex = generatedPoint.ID;
                }
            }
            Point.get(lastIndex).attach(Point.get(firstIndex));
        }



        //adding the boundary results in unstable lattice spring structures
        //connect boundary members
        for (int i = 0; i < members.size(); i = i + 1) {
            if (members.get(i).futureBoundary) {
                boundaryMembers.add(members.get(i).ID);
            }
        }

        double checkRadius = r * Math.sqrt(2.0) + 0.01;
        //the sqrt(2) covers diagonal connections while the 0.01 fights floating point precision errors
        if (type == 0 || type == 2) sortBoundaryMembers(checkRadius);

        //connect close points in the structure and assign values like mass
        double cX = Double.NaN;
        double cY = Double.NaN;
        if (isShapeMatch) {
            double[] results = calculateCenterOfMass();
            cX = results[0];
            cY = results[1];
        }
        double massPer = mass / members.size();
        double HOOKE_CONSTANT = stiffness * massPer;
        double SPRING_DAMPING = DAMP_COEFFICIENT * Math.sqrt(HOOKE_CONSTANT * massPer);
        for (int i = 0; i < members.size(); i = i + 1) {
            //Assign motion and mass
            members.get(i).setMass(massPer);
            members.get(i).setMovingMotion(movingMotion);
            members.get(i).HOOKE_SPRING_CONSTANT = HOOKE_CONSTANT;
            members.get(i).SPRING_DAMPING_COEFFICIENT = SPRING_DAMPING;
            members.get(i).SPRING_MAX_DIST_MULTIPLIER = max_dist_multiplier;
            members.get(i).SPRING_MIN_DIST_MULTIPLIER = min_dist_multiplier;

            //connect to other nearby points
            if (Point.get(i).isAttached()) for (int j = 0; j < members.size(); j = j + 1) {
                if (i != j && members.get(j).isAttached()) {
                    double distance = (members.get(i).getX() - members.get(j).getX()) * (members.get(i).getX() - members.get(j).getX());
                    distance = distance + (members.get(i).getY() - members.get(j).getY()) * (members.get(i).getY() - members.get(j).getY());
                    distance = Math.sqrt(distance);
                    if (distance < checkRadius) {
                        members.get(i).attach(members.get(j));
                    }
                }
            }

            //store ideal vector from center
            if (isShapeMatch) {
                idealShapeVectors.add(new Double[]{members.get(i).getX() - cX, members.get(i).getY() - cY});
            }
        }

        calculateProperties();
    }
    public static void step(int simID) {
        for (int i = 0; i < num; i = i + 1) {
            softbodies.get(i).calculateProperties();
            if (softbodies.get(i).simID != simID) continue;
            if (softbodies.get(i).hasPressure) softbodies.get(i).calcPressure();
            if (softbodies.get(i).isShapeMatch) softbodies.get(i).springTowardsShapeMatch();
        }
    }

    private void calcPressure() {
        double area = findSignedArea();
        boolean invertNormals = false;
        if (area < 0.0) {
            area = -area;
            invertNormals = true;
        }
        double pressure = (initialPressure * (initialArea / area)) - initialPressure;
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            double dx = Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getX() - Point.get(boundaryMembers.get(i)).getX();
            double dy = Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getY() - Point.get(boundaryMembers.get(i)).getY();
            double l = Math.sqrt(dx * dx + dy * dy);
            double nX = -dy / l;
            double nY = dx / l;
            if (invertNormals) {
                nX = -nX;
                nY = -nY;
            }
            double magnitudeUpdate = (pressure * l) / Point.get(boundaryMembers.get(i)).getMass();
            Point.get(boundaryMembers.get(i)).changeAX(magnitudeUpdate * nX);
            Point.get(boundaryMembers.get(i)).changeAY(magnitudeUpdate * nY);
            magnitudeUpdate = (pressure * l) / Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getMass();
            Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).changeAX(magnitudeUpdate * nX);
            Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).changeAY(magnitudeUpdate * nY);
        }
    }
    private void springTowardsShapeMatch() {
        double crossSum = 0.0;
        double dotSum = 0.0;
        for (int i = 0; i < members.size(); i = i + 1) {
            double dx = members.get(i).getX() - cM[0];
            double dy = members.get(i).getY() - cM[1];
            crossSum = crossSum + (dx * -idealShapeVectors.get(i)[1] + dy * idealShapeVectors.get(i)[0]);
            dotSum = dotSum + (dx * idealShapeVectors.get(i)[0] + dy * idealShapeVectors.get(i)[1]);
        }
        double theta = Math.atan2(crossSum, dotSum);

        //iterate through the points and apply a spring force towards the ideal position
        for (int i = 0; i < members.size(); i = i + 1) {
            double iX = idealShapeVectors.get(i)[0] * Math.cos(theta) - idealShapeVectors.get(i)[1] * Math.sin(theta);
            double iY = idealShapeVectors.get(i)[1] * Math.cos(theta) + idealShapeVectors.get(i)[0] * Math.sin(theta);
            double dx = iX - (members.get(i).getX() - cM[0]);
            double dy = iY - (members.get(i).getY() - cM[1]);
            double distance = Math.sqrt(dx * dx + dy * dy);
            double magnitude1 = (members.get(i).getVX() * dx + members.get(i).getVY() * dy) / distance;
            if (magnitude1 < 0.0) magnitude1 = 0.0;
            double temp1 = magnitude1 * SHAPE_MATCH_DAMPING;
            double temp2 = SHAPE_MATCH_STRENGTH * distance;
            if (Math.abs(temp2) > Math.abs(temp1)) {
                double update = SHAPE_MATCH_STRENGTH * dx - magnitude1 * SHAPE_MATCH_DAMPING * (dx / distance);
                if (Double.isNaN(update)) update = 0.0;
                members.get(i).changeAX(update);
                update = SHAPE_MATCH_STRENGTH * dy - magnitude1 * SHAPE_MATCH_DAMPING * (dy / distance);
                if (Double.isNaN(update)) update = 0.0;
                members.get(i).changeAY(update);
            }
        }

    }
    private double findSignedArea() {
        double totalArea = 0.0;
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            double smallArea = Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getX() - Point.get(boundaryMembers.get(i)).getX();
            smallArea = 0.5 * smallArea * (Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getY() + Point.get(boundaryMembers.get(i)).getY());
            totalArea = totalArea + smallArea;
        }
        return(totalArea);
    }
    private double[] calculateCenterOfMass() {
        double cX = 0.0;
        double cY = 0.0;
        for (int i = 0; i < Point.num; i = i + 1) {
            if (Point.get(i).parentSoftbody == ID) {
                cX = cX + Point.get(i).getX();
                cY = cY + Point.get(i).getY();
            }
        }
        cX = cX / members.size();
        cY = cY / members.size();
        return(new double[]{cX, cY});
    }
    private void calculateProperties() {
        cM = calculateCenterOfMass();
        for (Point point : members) {
            double dx = point.getX() - cM[0];
            double dy = point.getY() - cM[1];
            double squaredDistance = point.getX() * point.getX() + point.getY() * point.getY();
            if (squaredDistance > largestSquaredDistance) largestSquaredDistance = squaredDistance;
            double radius = point.getSolidRadius();
            if (point.getX() - radius < minX) minX = point.getX() - radius;
            if (point.getX() + radius > maxX) maxX = point.getX() + radius;
            if (point.getY() - radius < minY) minY = point.getY() - radius;
            if (point.getY() + radius > maxY) maxY = point.getY() + radius;
        }
    }
    //boundary members don't have to be sorted in winding order, but the neighboring indices to a boundary members must be valid edges
    private void sortBoundaryMembers(double checkRadius) {
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            ArrayList<Integer> possibleAttachmentID = new ArrayList<>();
            for (int j = 0; j < boundaryMembers.size(); j = j + 1) {
                if (i != j) {
                    double dx = Point.get(boundaryMembers.get(i)).getX() - Point.get(boundaryMembers.get(j)).getX();
                    double dy = Point.get(boundaryMembers.get(i)).getY() - Point.get(boundaryMembers.get(j)).getY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (distance <= checkRadius) possibleAttachmentID.add(boundaryMembers.get(j));
                }
            }
            for (int j = 0; j < possibleAttachmentID.size(); j = j + 1) {
                boolean valid = true;
                for (int k = 0; k < possibleAttachmentID.size(); k = k + 1) {
                    if (j == k) continue;
                    double dx = Point.get(possibleAttachmentID.get(j)).getX() - Point.get(possibleAttachmentID.get(k)).getX();
                    double dy = Point.get(possibleAttachmentID.get(j)).getY() - Point.get(possibleAttachmentID.get(k)).getY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (distance <= checkRadius) {
                        valid = false;
                        break;
                    }
                }
                if (valid) Point.get(boundaryMembers.get(i)).attach(Point.get(possibleAttachmentID.get(j)));
            }
        }
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            if (Point.get(boundaryMembers.get(i)).getBoundaryAttachmentNum() != 2) System.out.println("Boundary Member not with 2 buddies: " + boundaryMembers.get(i));
        }

        ArrayList<Integer> includedList = new ArrayList<>();
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            if (Point.get(boundaryMembers.get(i)).getBoundaryAttachmentNum() == 2) includedList.add(boundaryMembers.get(i));
        }
        ArrayList<Integer> boundaryIndicesPotentiallyCovered = new ArrayList<>();
        ArrayList<Integer> boundaryIndicesCovered = new ArrayList<>();
        for (int i = 0; i < includedList.size(); i = i + 1) {
            int minIndex1 = -1;
            int minIndex2 = -1;
            for (int j = 0; j < Point.get(includedList.get(i)).getAttachmentNum(); j = j + 1) {
                Point point = Point.get(includedList.get(i)).getAttachment(j);
                if (boundaryIndicesCovered.contains(Point.get(includedList.get(i)).getAttachments().get(j)) || point.getBoundaryAttachmentNum() != 2) continue;
                minIndex1 = point.ID;
                break;
            }
            for (int j = 0; j < Point.get(includedList.get(i)).getAttachmentNum(); j = j + 1) {
                Point point = Point.get(includedList.get(i)).getAttachment(j);
                if (minIndex1 == point.ID || boundaryIndicesCovered.contains(Point.get(includedList.get(i)).getAttachments().get(j)) || point.getBoundaryAttachmentNum() != 2) continue;
                minIndex2 = point.ID;
                break;
            }
            double minDist = Double.NaN;
            if (minIndex2 == -1) for (int j = 0; j < includedList.size(); j = j + 1) {
                if (i != j && includedList.get(j) != minIndex1){
                    double dx = Point.get(includedList.get(i)).getX() - Point.get(includedList.get(j)).getX();
                    double dy = Point.get(includedList.get(i)).getY() - Point.get(includedList.get(j)).getY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (Double.isNaN(minDist) || distance < minDist) {
                        minDist = distance;
                        minIndex2 = includedList.get(j);
                    }
                }
            }
            Point.get(includedList.get(i)).minIndex1 = minIndex1;
            Point.get(includedList.get(i)).minIndex2 = minIndex2;

            if (boundaryIndicesPotentiallyCovered.contains(minIndex1)) {
                boundaryIndicesCovered.add(minIndex1);
            }
            else {
                boundaryIndicesPotentiallyCovered.add(minIndex1);
            }

            if (boundaryIndicesPotentiallyCovered.contains(minIndex2)) {
                boundaryIndicesCovered.add(minIndex2);
            }
            else {
                boundaryIndicesPotentiallyCovered.add(minIndex2);
            }
        }

        int onIndex = includedList.get(0);
        int findIndex = Point.get(onIndex).minIndex1;
        ArrayList<Integer> sortedList = new ArrayList<>();
        for (int i = 0; i < includedList.size(); i = i + 1) {
            int temp = -1;
            int test = Point.get(findIndex).minIndex1;
            int test2 = Point.get(findIndex).minIndex2;
            if (Point.get(findIndex).minIndex1 != onIndex) {
                temp = Point.get(findIndex).minIndex1;
                sortedList.add(findIndex);
            } else if (Point.get(findIndex).minIndex2 != onIndex) {
                temp = Point.get(findIndex).minIndex2;
                sortedList.add(findIndex);
            }
            onIndex = findIndex;
            findIndex = temp;
        }
        boundaryMembers = sortedList;
    }
    public void setBoundaryColor(Color color) {
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            Point.get(boundaryMembers.get(i)).setColor(color);
        }
    }
    public boolean pointInside(double[] point) {
        int raycastCount = 0;
        int size = boundaryMembers.size();
        for (int i = 0; i < size; i = i + 1) {
            double minX = Point.get(boundaryMembers.get(i)).getX();
            int minXindex = i;
            double maxX = Point.get(boundaryMembers.get((i + 1) % size)).getX();
            int maxXindex = (i + 1) % size;
            if (maxX < minX) {
                double temp = minX;
                int tempIndex = minXindex;
                minX = maxX;
                minXindex = maxXindex;
                maxX = temp;
                maxXindex = tempIndex;
            }
            double maxY = Point.get(boundaryMembers.get(minXindex)).getY();
            double temp = Point.get(boundaryMembers.get(maxXindex)).getY();
            if (temp > maxY) maxY = temp;
            if (point[0] >= minX && point[0] < maxX && point[1] <= maxY) {
                if (((Point.get(boundaryMembers.get(maxXindex)).getY() - Point.get(boundaryMembers.get(minXindex)).getY()) / (maxX - minX)) * (point[0] - maxX) + Point.get(boundaryMembers.get(maxXindex)).getY() > point[1]) {
                    raycastCount = raycastCount + 1;
                }
            }
        }
        return (raycastCount % 2 == 1);
    }
    public Triplet resolvePointInside(double[] point, double radius) {
        boolean intersecting = false;
        int size = boundaryMembers.size();
        double closestEdgeDistance = Double.NaN;
        double closestEdgeNX = Double.NaN;
        double closestEdgeNY = Double.NaN;
        int closestEdgeIndex = -1;
        boolean invertNormals = findSignedArea() < 0.0;
        if (radius <= 0.0) intersecting = pointInside(point);
        if (radius > 0.0 || intersecting) for (int i = 0; i < size; i = i + 1) {
            double x1 = Point.get(boundaryMembers.get(i)).getX();
            double x2 = Point.get(boundaryMembers.get((i + 1) % size)).getX();
            double y1 = Point.get(boundaryMembers.get(i)).getY();
            double y2 = Point.get(boundaryMembers.get((i + 1) % size)).getY();
            double nX = -(y2 - y1);
            double nY = x2 - x1;
            double magnitude = Math.sqrt(nX * nX + nY * nY);
            nX = nX / magnitude;
            nY = nY / magnitude;
            if (invertNormals) {
                nX = -nX;
                nY = -nY;
            }

            if (radius > 0.0) {
                boolean rimIntersecting = pointInside(new double[]{point[0] - nX * radius, point[1] - nY * radius});
                if (!intersecting) intersecting = rimIntersecting;
                if (!rimIntersecting) continue;
            }

            double distance = (point[0] - x1) * nX + (point[1] - y1) * nY - radius;
            if ((Double.isNaN(closestEdgeDistance) || distance > closestEdgeDistance) && distance < 0.0) {
                double orthoDotPoint = point[0] * -nY + point[1] * nX;
                double orthoDotMin = x1 * -nY + y1 * nX;
                double orthoDotMax = x2 * -nY + y2 * nX;
                if (orthoDotMax < orthoDotMin) {
                    double temp1 = orthoDotMax;
                    orthoDotMax = orthoDotMin;
                    orthoDotMin = temp1;
                }
                if (orthoDotPoint > orthoDotMin && orthoDotPoint < orthoDotMax) {
                    closestEdgeDistance = distance;
                    closestEdgeNX = nX;
                    closestEdgeNY = nY;
                    closestEdgeIndex = i;
                }
            }
        }
        closestEdgeDistance *= 1.0 + (MTV_EPSILON / Math.abs(closestEdgeDistance));
        double[] MTV = new double[]{-closestEdgeDistance * closestEdgeNX, -closestEdgeDistance * closestEdgeNY};
        if (closestEdgeIndex >= 0) closestEdgeIndex = boundaryMembers.get(closestEdgeIndex) * -2 - 3;
        return (new Triplet(intersecting, MTV, closestEdgeIndex));
    }


    public Color getColor() {
        return(color);
    }
    public double getPointRadius() {
        return(pointRadius);
    }
    public void addMember(Point input, boolean onBoundary) {
        members.add(input);
        if (onBoundary) input.futureBoundary = true;
        input.parentSoftbody = ID;
    }
    public Point getMember(int index) {
        return(members.get(index));
    }
    public int size() {
        return(members.size());
    }
    public int boundarySize() {
        return(boundaryMembers.size());
    }
    public static Softbody get(int index) {
        return(softbodies.get(index));
    }
}
