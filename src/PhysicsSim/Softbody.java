package PhysicsSim;
import java.awt.*;
import java.util.ArrayList;

class Softbody {
    private final double mass;
    private double initialArea;
    private final Color color;
    private final double pointRadius;
    private final ArrayList<Point> members = new ArrayList<>();
    private ArrayList<Integer> boundaryMembers = new ArrayList<>();
    private static final ArrayList<Softbody> softbodies = new ArrayList<>();
    public static int num = 0;
    public final int ID;

    private final double initialPressure;
    private final boolean hasPressure;
    public final double HOOKE_CONSTANT;
    public final double SPRING_DAMPING;
    public static double DAMP_COEFFICIENT = 0.6;

    public Softbody(boolean hasPressure, double[] borderX, double[] borderY, double[] movingMotion, double pointRadius, double targetDensity, double stiffness, double initialPressure, double mass, Color color, double max_dist_multiplier, double min_dist_multiplier) {
        //find the distance between points from the target density (points per 25 x 25 area)
        double r = Math.sqrt(((4.0 / 3.0) * Math.sqrt(3) * 625.0) / targetDensity);
        this.hasPressure = hasPressure;
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

        if (!hasPressure) {
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
            Point seedPoint = new Point(new double[]{pointTest[0], pointTest[1], 0.0, 0.0, 0.0, 0.0}, pointRadius, 1.0, color, true);
            addMember(seedPoint, onBoundary);

            //start the lattice structure generation process
            seedPoint.generatePoints(0.0, r, borderX, borderY, invertNormals);
        }
        else {
            //create border points
            for (int j = 0; j < borderX.length; j = j + 1) {
                double dx = borderX[(j + 1) % borderX.length] - borderX[j];
                double dy = borderY[(j + 1) % borderX.length] - borderY[j];
                double distance = Math.sqrt(dx * dx + dy * dy);
                int iterations = (int) (distance / r);
                for (int i = 1; i < iterations; i = i + 1) {
                    double x = (dx / iterations) * i + borderX[j];
                    double y = (dy / iterations) * i + borderY[j];
                    Color a = color;
                    addMember(new Point(new double[]{x, y, 0.0, 0.0, 0.0, 0.0}, pointRadius, 1.0, a, true), true);
                }
            }
        }

        //connect close points in the structure and assign values like mass
        double massPer = mass / members.size();
        HOOKE_CONSTANT = stiffness * massPer;
        SPRING_DAMPING = DAMP_COEFFICIENT * Math.sqrt(HOOKE_CONSTANT * massPer);
        for (int i = 0; i < Point.num; i = i + 1) {
            if (Point.get(i).parentSoftbody == ID) {
                //Assign motion and mass
                Point.get(i).setMass(massPer);
                Point.get(i).setMovingMotion(movingMotion);
                Point.get(i).HOOKE_SPRING_CONSTANT = HOOKE_CONSTANT;
                Point.get(i).SPRING_DAMPING_COEFFICIENT = SPRING_DAMPING;
                Point.get(i).SPRING_MAX_DIST_MULTIPLIER = max_dist_multiplier;
                Point.get(i).SPRING_MIN_DIST_MULTIPLIER = min_dist_multiplier;

                //connect to other nearby points
                if (Point.get(i).isAttached()) for (int j = 0; j < Point.num; j = j + 1) {
                    if (i != j && Point.get(j).parentSoftbody == ID && Point.get(j).isAttached()) {
                        double distance = (Point.get(i).getX() - Point.get(j).getX()) * (Point.get(i).getX() - Point.get(j).getX());
                        distance = distance + (Point.get(i).getY() - Point.get(j).getY()) * (Point.get(i).getY() - Point.get(j).getY());
                        distance = Math.sqrt(distance);
                        if (distance < r * Math.sqrt(2.0) + 1.0) {
                            Point.get(i).attach(Point.get(j));
                        }
                    }
                }
            }
        }

        //adding the boundary results in unstable lattice spring structures
        //connect boundary members
        for (int i = 0; i < Point.num; i = i + 1) {
            if (Point.get(i).parentSoftbody == ID && Point.get(i).futureBoundary) {
                boundaryMembers.add(Point.get(i).ID);
            }
        }
        for (int i = 0; i < boundaryMembers.size(); i = i + 1) {
            //Point.get(boundaryMembers.get(i))
            int minIndex1 = -1;
            double minDist1 = Double.NaN;
            int minIndex2 = -1;
            double minDist2 = Double.NaN;
            for (int j = 0; j < boundaryMembers.size(); j = j + 1) {
                if (i == j) continue;
                double dx = Point.get(boundaryMembers.get(i)).getX() - Point.get(boundaryMembers.get(j)).getX();
                double dy = Point.get(boundaryMembers.get(i)).getY() - Point.get(boundaryMembers.get(j)).getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (Double.isNaN(minDist1) || distance < minDist1) {
                    minDist1 = distance;
                    minIndex1 = boundaryMembers.get(j);
                }
            }
            for (int j = 0; j < boundaryMembers.size(); j = j + 1) {
                if (i == j || boundaryMembers.get(j) == minIndex1) continue;
                double dx = Point.get(boundaryMembers.get(i)).getX() - Point.get(boundaryMembers.get(j)).getX();
                double dy = Point.get(boundaryMembers.get(i)).getY() - Point.get(boundaryMembers.get(j)).getY();
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (Double.isNaN(minDist2) || distance < minDist2) {
                    minDist2 = distance;
                    minIndex2 = boundaryMembers.get(j);
                }
            }
            Point.get(boundaryMembers.get(i)).attach(Point.get(minIndex1));
            Point.get(boundaryMembers.get(i)).minIndex1 = minIndex1;
            Point.get(boundaryMembers.get(i)).attach(Point.get(minIndex2));
            Point.get(boundaryMembers.get(i)).minIndex2 = minIndex2;
        }
        sortBoundaryMembers();
    }
    public static void step(double dt) {
        for (int i = 0; i < num; i = i + 1) {
            if (softbodies.get(i).hasPressure) softbodies.get(i).calcPressure(dt);
        }
    }

    private void calcPressure(double dt) {
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
            double magnitudeUpdate = (pressure * dt) / (l * mass);
            Point.get(boundaryMembers.get(i)).changeVX(magnitudeUpdate * nX);
            Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).changeVX(magnitudeUpdate * nX);
            Point.get(boundaryMembers.get(i)).changeVY(magnitudeUpdate * nY);
            Point.get(boundaryMembers.get((i + 1) % boundaryMembers.size())).changeVY(magnitudeUpdate * nX);
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
    private void sortBoundaryMembers() {
        ArrayList<Integer> sortedList = new ArrayList<>();
        sortedList.add(Point.get(boundaryMembers.get(0)).ID);
        int onIndex = Point.get(boundaryMembers.get(0)).ID;
        int findIndex = Point.get(boundaryMembers.get(0)).minIndex1;

        for (int i = 0; i < boundaryMembers.size() - 1; i = i + 1) {
            sortedList.add(findIndex);
            int temp = -1;
            if (Point.get(findIndex).minIndex1 != onIndex) {
                temp = Point.get(findIndex).minIndex1;
            }
            else if (Point.get(findIndex).minIndex2 != onIndex) {
                temp = Point.get(findIndex).minIndex2;
            }
            onIndex = findIndex;
            findIndex = temp;
        }

        ArrayList<Integer> tempList = new ArrayList<>();
        for(int i = 0; i < boundaryMembers.size(); i = i + 1) {
            if (!tempList.contains(sortedList.get(0))) {
                tempList.add(sortedList.get(0));
                sortedList.remove(0);
            }
            else {
                break;
            }
        }
        boundaryMembers = tempList;

    }


    public Color getColor() {
        return(color);
    }
    public double getPointRadius() {
        return(pointRadius);
    }
    public void addMember(Point input, boolean onBoundary) {
        members.add(input);
        if (onBoundary) boundaryMembers.add(input.ID);
        input.parentSoftbody = ID;
    }
    public static Softbody get(int index) {
        return(softbodies.get(index));
    }
}
