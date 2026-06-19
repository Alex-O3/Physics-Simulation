package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;

class Softbody {
    public int simID;

    protected final double mass;
    private final double initialArea;
    private final Color color;
    private final double pointRadius;
    protected final ArrayList<Rigidbody> members = new ArrayList<>();
    protected ArrayList<Integer> boundaryMembers = new ArrayList<>();
    private static final ArrayList<Softbody> softbodies = new ArrayList<>();
    private final ArrayList<Double[]> idealShapeVectors = new ArrayList<>();
    public static int num = 0;
    public final int ID;

    private double largestDistance;
    public double minX;
    public double maxX;
    public double minY;
    public double maxY;
    public double[] cM;

    private final double initialPressure;
    private final boolean hasPressure;
    private final boolean isShapeMatch;
    public double SHAPE_MATCH_STRENGTH = 100.0;
    public double SHAPE_MATCH_DAMPING = 0.25;

    public Softbody(SoftbodyType type, double[] borderX, double[] borderY, double[] movingMotion, double pointRadius, double targetDensity, double stiffness, double initialPressure, double mass, Color color, double max_dist_multiplier, double min_dist_multiplier, int simID) {
        this.simID = simID;

        //find the distance between points from the target density (points per 50 x 50 area)
        double r = 50.0 * Math.sqrt(1.0 / targetDensity);
        this.hasPressure = type == SoftbodyType.PressureSpring;
        this.isShapeMatch = type == SoftbodyType.ShapeMatchSolid || type == SoftbodyType.ShapeMatchHollow;
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


        if (type == SoftbodyType.SpringMass || type == SoftbodyType.ShapeMatchSolid) {
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
            Rigidbody seedPoint = new Rigidbody(new Circle(pointRadius), new double[]{pointTest[0], pointTest[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0, color, simID);
            Simulation.get(simID).physicsObjects.add(new PhysicsObject(seedPoint));
            Simulation.rigidbodyObjectsIDToGlobalID.add(Simulation.get(simID).physicsObjects.size() - 1);
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
                    Rigidbody generatedPoint = new Rigidbody(new Circle(pointRadius), new double[]{x, y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0, a, simID);
                    Simulation.get(simID).physicsObjects.add(new PhysicsObject(generatedPoint));
                    Simulation.rigidbodyObjectsIDToGlobalID.add(Simulation.get(simID).physicsObjects.size() - 1);
                    addMember(generatedPoint, true);
                    if (lastIndex != -1) Rigidbody.get(lastIndex).springAttachSoftbodyConstruction(generatedPoint);
                    else firstIndex = generatedPoint.ID;
                    lastIndex = generatedPoint.ID;
                }
            }
            Rigidbody.get(lastIndex).springAttachSoftbodyConstruction(Rigidbody.get(firstIndex));
        }



        //adding the boundary results in unstable lattice spring structures
        //connect boundary members
        for (int i = 0; i < members.size(); i = i + 1) {
            if (members.get(i).futureBoundary) {
                boundaryMembers.add(members.get(i).ID);
            }
        }

        double checkRadius = r * Math.sqrt(2.0) + Simulation.get(simID).MTV_EPSILON;
        //the sqrt(2) covers diagonal connections while the 0.01 fights floating point precision errors
        if (type == SoftbodyType.SpringMass || type == SoftbodyType.ShapeMatchSolid) sortBoundaryMembers(checkRadius);

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
        double SPRING_DAMPING = Simulation.get(simID).DAMPING_COEFFICIENT_RELATOR * Math.sqrt(HOOKE_CONSTANT * massPer);
        for (int i = 0; i < members.size(); i = i + 1) {
            //Assign motion and mass
            members.get(i).mass = massPer;
            members.get(i).setMovingMotion(movingMotion);
            members.get(i).lockRotation(true);
            members.get(i).setRepulseRadius(Simulation.get(simID).REPULSE_RADIUS_MULTIPLIER);

            //connect to other nearby points
            if (type != SoftbodyType.PressureSpring && type != SoftbodyType.ShapeMatchHollow && members.get(i).isAttached) for (int j = 0; j < members.size(); j = j + 1) {
                if (i != j && members.get(j).isAttached) {
                    double distance = (members.get(i).getPosX() - members.get(j).getPosX()) * (members.get(i).getPosX() - members.get(j).getPosX());
                    distance = distance + (members.get(i).getPosY() - members.get(j).getPosY()) * (members.get(i).getPosY() - members.get(j).getPosY());
                    distance = Math.sqrt(distance);
                    if (distance <= checkRadius) {
                        members.get(i).springAttachSoftbodyConstruction(members.get(j));
                    }
                }
            }

            //store ideal vector from center
            if (isShapeMatch) {
                idealShapeVectors.add(new Double[]{members.get(i).getPosX() - cX, members.get(i).getPosY() - cY});
            }

            members.get(i).setAllSoftbodyJoints(HOOKE_CONSTANT, SPRING_DAMPING, min_dist_multiplier, max_dist_multiplier);
        }
        for (Integer boundaryMember : boundaryMembers) {
            for (Joint spring : Rigidbody.get(boundaryMember).attachments) {
                if (spring.connection.futureBoundary) {
                    spring.makeSolid();
                }
            }
        }

        calculateProperties();
    }
    public static void step(int simID) {
        for (int i = 0; i < num; i = i + 1) {
            if (softbodies.get(i).simID != simID) continue;
            softbodies.get(i).calculateProperties();
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
            double dx = Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getPosX() - Rigidbody.get(boundaryMembers.get(i)).getPosX();
            double dy = Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getPosY() - Rigidbody.get(boundaryMembers.get(i)).getPosY();
            double l = Math.sqrt(dx * dx + dy * dy);
            double nX = -dy / l;
            double nY = dx / l;
            if (invertNormals) {
                nX = -nX;
                nY = -nY;
            }
            double magnitudeUpdate = (pressure * l) / Rigidbody.get(boundaryMembers.get(i)).getMass();
            Rigidbody.get(boundaryMembers.get(i)).changeAX(magnitudeUpdate * nX);
            Rigidbody.get(boundaryMembers.get(i)).changeAY(magnitudeUpdate * nY);
            magnitudeUpdate = (pressure * l) / Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getMass();
            Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).changeAX(magnitudeUpdate * nX);
            Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).changeAY(magnitudeUpdate * nY);
        }
    }
    private void springTowardsShapeMatch() {
        double crossSum = 0.0;
        double dotSum = 0.0;
        double[] cMvelocity = new double[]{0.0,0.0};
        for (int i = 0; i < members.size(); i = i + 1) {
            double dx = members.get(i).getPosX() - cM[0];
            double dy = members.get(i).getPosY() - cM[1];
            crossSum = crossSum + (dx * -idealShapeVectors.get(i)[1] + dy * idealShapeVectors.get(i)[0]);
            dotSum = dotSum + (dx * idealShapeVectors.get(i)[0] + dy * idealShapeVectors.get(i)[1]);
            cMvelocity[0] += members.get(i).getVX() * members.get(i).getMass();
            cMvelocity[1] += members.get(i).getVY() * members.get(i).getMass();
        }
        cMvelocity[0] /= mass;
        cMvelocity[1] /= mass;
        double theta = Math.atan2(crossSum, dotSum);

        //iterate through the points and apply a spring force towards the ideal position
        for (int i = 0; i < members.size(); i = i + 1) {
            double iX = idealShapeVectors.get(i)[0] * Math.cos(theta) - idealShapeVectors.get(i)[1] * Math.sin(theta);
            double iY = idealShapeVectors.get(i)[1] * Math.cos(theta) + idealShapeVectors.get(i)[0] * Math.sin(theta);
            double dx = iX - (members.get(i).getPosX() - cM[0]);
            double dy = iY - (members.get(i).getPosY() - cM[1]);
            double distance = Math.sqrt(dx * dx + dy * dy);
            double magnitude1 = ((members.get(i).getVX() - cMvelocity[0]) * dx + (members.get(i).getVY() - cMvelocity[1]) * dy) / distance;
            if (magnitude1 < 0.0) magnitude1 = 0.0;
            double temp1 = magnitude1 * SHAPE_MATCH_DAMPING;
            double temp2 = SHAPE_MATCH_STRENGTH * distance;
            if (Math.abs(temp2) > Math.abs(temp1)) {
                double update = SHAPE_MATCH_STRENGTH * dx - temp1 * (dx / distance);
                if (Double.isNaN(update)) update = 0.0;
                members.get(i).changeAX(update / members.get(i).getMass());
                update = SHAPE_MATCH_STRENGTH * dy - temp1 * (dy / distance);
                if (Double.isNaN(update)) update = 0.0;
                members.get(i).changeAY(update / members.get(i).getMass());
            }
        }

    }
    private double findSignedArea() {
        double totalArea = 0.0;
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            double smallArea = Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getPosX() - Rigidbody.get(boundaryMembers.get(i)).getPosX();
            smallArea = 0.5 * smallArea * (Rigidbody.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).getPosY() + Rigidbody.get(boundaryMembers.get(i)).getPosY());
            totalArea = totalArea + smallArea;
        }
        return(totalArea);
    }
    private double[] calculateCenterOfMass() {
        double cX = 0.0;
        double cY = 0.0;
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            if (Rigidbody.get(i).parentSoftbody == ID) {
                cX = cX + Rigidbody.get(i).getPosX();
                cY = cY + Rigidbody.get(i).getPosY();
            }
        }
        cX = cX / members.size();
        cY = cY / members.size();
        return(new double[]{cX, cY});
    }
    private void calculateProperties() {
        cM = calculateCenterOfMass();
        minX = Double.NaN;
        maxX = Double.NaN;
        minY = Double.NaN;
        maxY = Double.NaN;
        double largestDistanceSquared = Double.NaN;
        for (Rigidbody point : members) {
            double dx = point.getPosX() - cM[0];
            double dy = point.getPosY() - cM[1];
            double squaredDistance = dx * dx + dy * dy;
            if (Double.isNaN(largestDistanceSquared) || squaredDistance > largestDistanceSquared) largestDistanceSquared = squaredDistance;
            if (Double.isNaN(minX) || point.geometry.leftBoundBox + point.getPosX() < minX) minX = point.geometry.leftBoundBox + point.getPosX();
            if (Double.isNaN(maxX) || point.geometry.rightBoundBox + point.getPosX() > maxX) maxX = point.geometry.rightBoundBox + point.getPosX();
            if (Double.isNaN(minY) || point.geometry.topBoundBox + point.getPosY() < minY) minY = point.geometry.topBoundBox + point.getPosY();
            if (Double.isNaN(maxY) || point.geometry.bottomBoundBox + point.getPosY() > maxY) maxY = point.geometry.bottomBoundBox + point.getPosY();
        }
        largestDistance = Math.sqrt(largestDistanceSquared);
    }
    //boundary members don't have to be sorted in winding order, but the neighboring indices to a boundary members must be valid edges
    private void sortBoundaryMembers(double checkRadius) {
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            ArrayList<Integer> possibleAttachmentID = new ArrayList<>();
            for (int j = 0; j < boundaryMembers.size(); j = j + 1) {
                if (i != j) {
                    double dx = Rigidbody.get(boundaryMembers.get(i)).getPosX() - Rigidbody.get(boundaryMembers.get(j)).getPosX();
                    double dy = Rigidbody.get(boundaryMembers.get(i)).getPosY() - Rigidbody.get(boundaryMembers.get(j)).getPosY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (distance <= checkRadius) possibleAttachmentID.add(boundaryMembers.get(j));
                }
            }
            for (int j = 0; j < possibleAttachmentID.size(); j = j + 1) {
                boolean valid = true;
                for (int k = 0; k < possibleAttachmentID.size(); k = k + 1) {
                    if (j == k) continue;
                    double dx = Rigidbody.get(possibleAttachmentID.get(j)).getPosX() - Rigidbody.get(possibleAttachmentID.get(k)).getPosX();
                    double dy = Rigidbody.get(possibleAttachmentID.get(j)).getPosY() - Rigidbody.get(possibleAttachmentID.get(k)).getPosY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (distance <= checkRadius) {
                        valid = false;
                        break;
                    }
                }
                if (valid) Rigidbody.get(boundaryMembers.get(i)).springAttachSoftbodyConstruction(Rigidbody.get(possibleAttachmentID.get(j)));
            }
        }
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            if (Rigidbody.get(boundaryMembers.get(i)).getBoundaryAttachmentNum() != 2) System.out.println("Boundary Member not with 2 buddies: " + boundaryMembers.get(i));
        }

        ArrayList<Integer> includedList = new ArrayList<>();
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            if (Rigidbody.get(boundaryMembers.get(i)).getBoundaryAttachmentNum() == 2) includedList.add(boundaryMembers.get(i));
        }
        ArrayList<Integer> boundaryIndicesPotentiallyCovered = new ArrayList<>();
        ArrayList<Integer> boundaryIndicesCovered = new ArrayList<>();
        for (int i = 0; i < includedList.size(); i = i + 1) {
            int minIndex1 = -1;
            int minIndex2 = -1;
            for (int j = 0; j < Rigidbody.get(includedList.get(i)).getAttachmentNum(); j = j + 1) {
                Rigidbody point = Rigidbody.get(includedList.get(i)).getAttachment(j);
                if (boundaryIndicesCovered.contains(Rigidbody.get(includedList.get(i)).getAttachmentInt(j))
                        || point.getBoundaryAttachmentNum() != 2) continue;
                minIndex1 = point.ID;
                break;
            }
            for (int j = 0; j < Rigidbody.get(includedList.get(i)).getAttachmentNum(); j = j + 1) {
                Rigidbody point = Rigidbody.get(includedList.get(i)).getAttachment(j);
                if (minIndex1 == point.ID || boundaryIndicesCovered.contains(Rigidbody.get(includedList.get(i)).getAttachmentInt(j))
                        || point.getBoundaryAttachmentNum() != 2) continue;
                minIndex2 = point.ID;
                break;
            }
            double minDist = Double.NaN;
            if (minIndex2 == -1) for (int j = 0; j < includedList.size(); j = j + 1) {
                if (i != j && includedList.get(j) != minIndex1){
                    double dx = Rigidbody.get(includedList.get(i)).getPosX() - Rigidbody.get(includedList.get(j)).getPosX();
                    double dy = Rigidbody.get(includedList.get(i)).getPosY() - Rigidbody.get(includedList.get(j)).getPosY();
                    double distance = Math.sqrt(dx * dx + dy * dy);
                    if (Double.isNaN(minDist) || distance < minDist) {
                        minDist = distance;
                        minIndex2 = includedList.get(j);
                    }
                }
            }
            Rigidbody.get(includedList.get(i)).minIndex1 = minIndex1;
            Rigidbody.get(includedList.get(i)).minIndex2 = minIndex2;

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
        int findIndex = Rigidbody.get(onIndex).minIndex1;
        ArrayList<Integer> sortedList = new ArrayList<>();
        for (int i = 0; i < includedList.size(); i = i + 1) {
            int temp = -1;
            if (Rigidbody.get(findIndex).minIndex1 != onIndex) {
                temp = Rigidbody.get(findIndex).minIndex1;
                sortedList.add(findIndex);
            } else if (Rigidbody.get(findIndex).minIndex2 != onIndex) {
                temp = Rigidbody.get(findIndex).minIndex2;
                sortedList.add(findIndex);
            }
            onIndex = findIndex;
            findIndex = temp;
        }
        boundaryMembers = sortedList;
    }
    public void setBoundaryColor(Color color) {
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            Rigidbody.get(boundaryMembers.get(i)).geometry.setColor(color);
        }
    }
    public boolean pointInside(double[] point) {
        int raycastCount = 0;
        int size = boundaryMembers.size();
        for (int i = 0; i < size; i = i + 1) {
            double minX = Rigidbody.get(boundaryMembers.get(i)).getPosX();
            int minXindex = i;
            double maxX = Rigidbody.get(boundaryMembers.get((i + 1) % size)).getPosX();
            int maxXindex = (i + 1) % size;
            if (maxX < minX) {
                double temp = minX;
                int tempIndex = minXindex;
                minX = maxX;
                minXindex = maxXindex;
                maxX = temp;
                maxXindex = tempIndex;
            }
            double maxY = Rigidbody.get(boundaryMembers.get(minXindex)).getPosY();
            double temp = Rigidbody.get(boundaryMembers.get(maxXindex)).getPosY();
            if (temp > maxY) maxY = temp;
            if (point[0] >= minX && point[0] < maxX && point[1] <= maxY) {
                if (point[1] <= minY) {
                    raycastCount += 1;
                    continue;
                }
                if (((Rigidbody.get(boundaryMembers.get(maxXindex)).getPosY()
                        - Rigidbody.get(boundaryMembers.get(minXindex)).getPosY()) / (maxX - minX)) * (point[0] - maxX)
                        + Rigidbody.get(boundaryMembers.get(maxXindex)).getPosY() > point[1]) {
                    raycastCount = raycastCount + 1;
                }
            }
        }
        return (raycastCount % 2 == 1);
    }
    public SoftbodyType getType() {
        if (hasPressure) return SoftbodyType.PressureSpring;
        if (isShapeMatch) {
            if (members.size() > boundaryMembers.size()) return SoftbodyType.ShapeMatchSolid;
            else return SoftbodyType.ShapeMatchHollow;
        }
        return SoftbodyType.SpringMass;
    }
    public boolean selfCollides() {
        SoftbodyType type = getType();
        if (simID == -1 || !Simulation.get(simID).hollowSoftbodiesSelfCollide) return false;
        return type == SoftbodyType.PressureSpring || type == SoftbodyType.ShapeMatchHollow;
    }


    public Color getColor() {
        return(color);
    }
    public double getPointRadius() {
        return(pointRadius);
    }
    public double getRadius() {
        return(largestDistance);
    }
    public void addMember(Rigidbody input, boolean onBoundary) {
        members.add(input);
        if (onBoundary) input.futureBoundary = true;
        input.parentSoftbody = ID;
    }
    public Rigidbody getMember(int index) {
        return(members.get(index));
    }
    public Rigidbody getBoundaryMember(int index) {
        return (Rigidbody.get(boundaryMembers.get(index)));
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

    public double getInitialPressure() {
        return initialPressure;
    }
}
