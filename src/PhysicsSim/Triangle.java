package PhysicsSim;
class Triangle { //this class is closely tied to the Rigidbody class
    private static double MTV_EPSILON = 500.0;

    private double[] x = new double[3];
    private double[] y = new double[3];
    private double[] centerOfMass = new double[2];
    private double area;
    private double mass;
    private double inertia;

    private int[] indices = new int[3];
    public boolean[] enabled = new boolean[]{true, true, true};
    public boolean partOfFace = false;
    private double[] normalX = new double[3];
    private double[] normalY = new double[3];
    private int parentRigidbody;
    boolean exists;

    public Triangle(int backwardIndex, int parentIndex, int forwardIndex, int parentRigidbody, double[] pos) {
        this.parentRigidbody = parentRigidbody;
        indices[0] = backwardIndex;
        indices[1] = parentIndex;
        indices[2] = forwardIndex;
        this.centerOfMass[0] = pos[0];
        this.centerOfMass[1] = pos[1];
        exists = true;
    }
    public Triangle() {
        exists = false;
    }
    public void calculateProperties() {
        x[0] = Rigidbody.get(parentRigidbody).getXPoints(indices[0]);
        y[0] = Rigidbody.get(parentRigidbody).getYPoints(indices[0]);
        x[1] = Rigidbody.get(parentRigidbody).getXPoints(indices[1]);
        y[1] = Rigidbody.get(parentRigidbody).getYPoints(indices[1]);
        x[2] = Rigidbody.get(parentRigidbody).getXPoints(indices[2]);
        y[2] = Rigidbody.get(parentRigidbody).getYPoints(indices[2]);
        double yMin = y[0];
        if (y[1] < yMin) yMin = y[1];
        if (y[2] < yMin) yMin = y[2];
        area = 0.0;
        //calculate area and normals
        for (int i = 0; i < 3; i = i + 1) {
            double trapezoidArea = 0.5 * (x[(i + 1) % 3] - x[i]) * (y[(i + 1) % 3] + y[i] - 2.0 * yMin);
            area = area + trapezoidArea;
            normalX[i] = -(y[(i + 1) % 3] - y[i]);
            normalY[i] = (x[(i + 1) % 3] - x[i]);
            double magnitude = Math.sqrt(normalX[i] * normalX[i] + normalY[i] * normalY[i]);
            normalX[i] = normalX[i] / magnitude;
            normalY[i] = normalY[i] / magnitude;
        }
        if (area < 0.0) {
            normalX[0] = -normalX[0];
            normalY[0] = -normalY[0];
            normalX[1] = -normalX[1];
            normalY[1] = -normalY[1];
            normalX[2] = -normalX[2];
            normalY[2] = -normalY[2];
        }
        area = Math.abs(area);
        mass = (Rigidbody.get(parentRigidbody).getMass() / Rigidbody.get(parentRigidbody).getArea()) * area;
        //side lengths
        double a = Math.sqrt((x[1] - x[0]) * (x[1] - x[0]) + (y[1] - y[0]) * (y[1] - y[0]));
        double b = Math.sqrt((x[2] - x[1]) * (x[2] - x[1]) + (y[2] - y[1]) * (y[2] - y[1]));
        double c = Math.sqrt((x[0] - x[2]) * (x[0] - x[2]) + (y[0] - y[2]) * (y[0] - y[2]));
        double d = (a * a + b * b - c * c) / (2.0 * a); //length until height along the side
        //calculate inertia
        inertia = 2.0 * a * d * (a - d) + d * b * b + (a - d) * c * c; //formula derived from calculus to find the moment of y = (b / a) * x, shifted to find the right triangle's moment by parallel axis theorem
        inertia = inertia / (18.0 * a); //then, parallel axis theorem is used to get the moment of the whole triangle out of the two right triangles that compose it
        inertia = mass * inertia;
    }
    public void shift(double shiftX, double shiftY) {
        x[0] = x[0] - shiftX;
        y[0] = y[0] - shiftY;
        x[1] = x[1] - shiftX;
        y[1] = y[1] - shiftY;
        x[2] = x[2] - shiftX;
        y[2] = y[2] - shiftY;
        centerOfMass[0] = centerOfMass[0] - shiftX;
        centerOfMass[1] = centerOfMass[1] - shiftY;
    }
    public void rotate(double theta) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        for (int i = 0; i < 3; i = i + 1) {
            double X = x[i];
            double Y = y[i];
            x[i] = X * cos - Y * sin;
            y[i] = X * sin + Y * cos;
            X = normalX[i];
            Y = normalY[i];
            normalX[i] = X * cos - Y * sin;
            normalY[i] = X * sin + Y * cos;
        }
        double X = centerOfMass[0];
        double Y = centerOfMass[1];
        centerOfMass[0] = X * cos - Y * sin;
        centerOfMass[1] = X * sin + Y * cos;
    }
    public Triplet checkCollisions(Triangle otherTriangle) {
        boolean intersecting = true;
        //use separating axis theorem
        double[] otherX = otherTriangle.getX();
        double[] otherY = otherTriangle.getY();
        double[] dotProductResults = new double[6];
        double overlap = Double.NaN; //NaN initialization
        double[] MTV = new double[]{Double.NaN, Double.NaN};
        double[] pointOfContact = new double[]{Double.NaN, Double.NaN}; //initialize the point of contact to NaN
        //first check the normals of this triangle
        if (!otherTriangle.partOfFace) for (int i = 0; i < 3; i = i + 1) {
            //first three are the points of this triangle on the normal axis and the second three are the same but for the other triangle
            dotProductResults[0] = normalX[i] * (x[0] + Rigidbody.get(parentRigidbody).getPosX()) + normalY[i] * (y[0] + Rigidbody.get(parentRigidbody).getPosY());
            dotProductResults[1] = normalX[i] * (x[1] + Rigidbody.get(parentRigidbody).getPosX()) + normalY[i] * (y[1] + Rigidbody.get(parentRigidbody).getPosY());
            dotProductResults[2] = normalX[i] * (x[2] + Rigidbody.get(parentRigidbody).getPosX()) + normalY[i] * (y[2] + Rigidbody.get(parentRigidbody).getPosY());
            dotProductResults[3] = normalX[i] * otherX[0] + normalY[i] * otherY[0];
            dotProductResults[4] = normalX[i] * otherX[1] + normalY[i] * otherY[1];
            dotProductResults[5] = normalX[i] * otherX[2] + normalY[i] * otherY[2];
            //find the min and max of both
            double dotMin = dotProductResults[0];
            double dotMax = dotProductResults[0];
            int dotMinIndex = 0;
            int dotMaxIndex = 0;
            if (dotProductResults[1] < dotMin) {
                dotMin = dotProductResults[1];
                dotMinIndex = 1;
            }
            if (dotProductResults[1] > dotMax) {
                dotMax = dotProductResults[1];
                dotMaxIndex = 1;
            }
            if (dotProductResults[2] < dotMin) {
                dotMin = dotProductResults[2];
                dotMinIndex = 2;
            }
            if (dotProductResults[2] > dotMax) {
                dotMax = dotProductResults[2];
                dotMaxIndex = 2;
            }

            double otherDotMin = dotProductResults[3];
            double otherDotMax = dotProductResults[3];
            int otherDotMinIndex = 3;
            int otherDotMaxIndex = 3;
            if (dotProductResults[4] < otherDotMin) {
                otherDotMin = dotProductResults[4];
                otherDotMinIndex = 4;
            }
            if (dotProductResults[4] > otherDotMax) {
                otherDotMax = dotProductResults[4];
                otherDotMaxIndex = 4;
            }
            if (dotProductResults[5] < otherDotMin) {
                otherDotMin = dotProductResults[5];
                otherDotMinIndex = 5;
            }
            if (dotProductResults[5] > otherDotMax) {
                otherDotMax = dotProductResults[5];
                otherDotMaxIndex = 5;
            }
            //determine if there is an intersection on that axis. If not, the triangles are not intersecting. If so, determine that overlap and find the one of least magnitude
            if (!(dotMax > otherDotMin && dotMin < otherDotMax)) {
                intersecting = false;
                break;
            }
            else if (mod(indices[(i + 1) % 3] - indices[i], Rigidbody.get(parentRigidbody).getNumPoints()) == 1 && enabled[i]) {
                if (partOfFace && !(Rigidbody.get(otherTriangle.parentRigidbody).getVX() * normalX[i] + Rigidbody.get(otherTriangle.parentRigidbody).getVY() * normalY[i] < 0.0)){
                    intersecting = false;
                    break;
                }
                double temp = Math.abs(dotMax - otherDotMin);
                if (Math.abs(otherDotMax - dotMin) < temp) temp = Math.abs(otherDotMax - dotMin);
                if (Double.isNaN(overlap) || temp < overlap) {
                    overlap = temp;
                    if(Rigidbody.get(otherTriangle.getParentID()).isMovable()) {
                        temp = -overlap * (otherTriangle.getParentMass() / (getParentMass() + otherTriangle.getParentMass())) - MTV_EPSILON;
                        MTV[0] = normalX[i] * temp;
                        MTV[1] = normalY[i] * temp;
                        pointOfContact[0] = otherTriangle.getX()[otherDotMinIndex - 3] - MTV[0] * (getParentMass() / otherTriangle.getParentMass());
                        pointOfContact[1] = otherTriangle.getY()[otherDotMinIndex - 3] - MTV[1] * (getParentMass() / otherTriangle.getParentMass());
                    }
                    else {
                        temp = -overlap - MTV_EPSILON;
                        MTV[0] = normalX[i] * temp;
                        MTV[1] = normalY[i] * temp;
                        pointOfContact[0] = otherTriangle.getX()[otherDotMinIndex - 3];
                        pointOfContact[1] = otherTriangle.getY()[otherDotMinIndex - 3];
                    }

                }
            }
        }
        //then check the normals of the other triangle
        if (!otherTriangle.enabled[0] && !otherTriangle.enabled[1] && !otherTriangle.enabled[2]) intersecting = false;
        if (intersecting) for (int i = 0; i < 3; i = i + 1) {
            //first three are the points of this triangle on the normal axis and the second three are the same but for the other triangle
            dotProductResults[0] = otherTriangle.getNormalX()[i] * (x[0] + Rigidbody.get(parentRigidbody).getPosX()) + otherTriangle.getNormalY()[i] * (y[0] + Rigidbody.get(parentRigidbody).getPosY());
            dotProductResults[1] = otherTriangle.getNormalX()[i] * (x[1] + Rigidbody.get(parentRigidbody).getPosX()) + otherTriangle.getNormalY()[i] * (y[1] + Rigidbody.get(parentRigidbody).getPosY());
            dotProductResults[2] = otherTriangle.getNormalX()[i] * (x[2] + Rigidbody.get(parentRigidbody).getPosX()) + otherTriangle.getNormalY()[i] * (y[2] + Rigidbody.get(parentRigidbody).getPosY());
            dotProductResults[3] = otherTriangle.getNormalX()[i] * otherX[0] + otherTriangle.getNormalY()[i] * otherY[0];
            dotProductResults[4] = otherTriangle.getNormalX()[i] * otherX[1] + otherTriangle.getNormalY()[i] * otherY[1];
            dotProductResults[5] = otherTriangle.getNormalX()[i] * otherX[2] + otherTriangle.getNormalY()[i] * otherY[2];
            //find the min and max of both
            double dotMin = dotProductResults[0];
            double dotMax = dotProductResults[0];
            int dotMinIndex = 0;
            int dotMaxIndex = 0;
            if (dotProductResults[1] < dotMin) {
                dotMin = dotProductResults[1];
                dotMinIndex = 1;
            }
            if (dotProductResults[1] > dotMax) {
                dotMax = dotProductResults[1];
                dotMaxIndex = 1;
            }
            if (dotProductResults[2] < dotMin) {
                dotMin = dotProductResults[2];
                dotMinIndex = 2;
            }
            if (dotProductResults[2] > dotMax) {
                dotMax = dotProductResults[2];
                dotMaxIndex = 2;
            }

            double otherDotMin = dotProductResults[3];
            double otherDotMax = dotProductResults[3];
            int otherDotMinIndex = 3;
            int otherDotMaxIndex = 3;
            if (dotProductResults[4] < otherDotMin) {
                otherDotMin = dotProductResults[4];
                otherDotMinIndex = 4;
            }
            if (dotProductResults[4] > otherDotMax) {
                otherDotMax = dotProductResults[4];
                otherDotMaxIndex = 4;
            }
            if (dotProductResults[5] < otherDotMin) {
                otherDotMin = dotProductResults[5];
                otherDotMinIndex = 5;
            }
            if (dotProductResults[5] > otherDotMax) {
                otherDotMax = dotProductResults[5];
                otherDotMaxIndex = 5;
            }
            //determine if there is an intersection on that axis. If not, the triangles are not intersecting. If so, determine that overlap and find the one of least magnitude
            if (!(dotMax > otherDotMin && dotMin < otherDotMax)) {
                intersecting = false;
                break;
            }
            else if (mod(otherTriangle.indices[(i + 1) % 3] - otherTriangle.indices[i], Rigidbody.get(otherTriangle.getParentID()).getNumPoints()) == 1 && otherTriangle.enabled[i]) {
                if (otherTriangle.partOfFace && !(Rigidbody.get(parentRigidbody).getVX() * otherTriangle.getNormalX()[i] + Rigidbody.get(parentRigidbody).getVY() * otherTriangle.getNormalY()[i] < 0.0)) {
                    intersecting = false;
                    break;
                }
                double temp = Math.abs(dotMax - otherDotMin);
                if (Math.abs(otherDotMax - dotMin) < temp) temp = Math.abs(otherDotMax - dotMin);
                if (Double.isNaN(overlap) || temp < overlap) {
                    overlap = temp;
                    if (Rigidbody.get(otherTriangle.getParentID()).isMovable()) {
                        temp = overlap * (otherTriangle.getParentMass() / (getParentMass() + otherTriangle.getParentMass())) + MTV_EPSILON;
                        MTV[0] = otherTriangle.getNormalX()[i] * temp;
                        MTV[1] = otherTriangle.getNormalY()[i] * temp;
                        pointOfContact[0] = getX()[dotMinIndex] + MTV[0];
                        pointOfContact[1] = getY()[dotMinIndex] + MTV[1];
                    }
                    else {
                        temp = overlap + MTV_EPSILON;
                        MTV[0] = otherTriangle.getNormalX()[i] * temp;
                        MTV[1] = otherTriangle.getNormalY()[i] * temp;
                        pointOfContact[0] = getX()[dotMinIndex] + MTV[0];
                        pointOfContact[1] = getY()[dotMinIndex] + MTV[1];
                    }
                }
            }
        }
        //uses the minimum overlap to calculate a minimum translation vector for penetration resolution
        //returns boolean intersecting, double[] MTV, and double[] pointOfContact if the triangle has an intersection with external sides
        //returns boolean intersecting if the triangle has an intersection without any external sides involved
        //returns boolean intersecting (but false) if the triangles do not intersect
        return(new Triplet(intersecting, MTV, pointOfContact));
    }
    public Triplet checkCollisions(Point other) {
        boolean intersecting = false;
        double[] MTV = new double[]{Double.NaN, Double.NaN};
        double[] pointOfContact = new double[]{Double.NaN, Double.NaN};
        double[] dotProductResults = new double[6];
        double overlap = Double.NaN;
        if (!partOfFace) for (int i = 0; i < 3; i = i + 1) {
            if (!(mod(indices[(i + 1) % 3] - indices[i], Rigidbody.get(parentRigidbody).getNumPoints()) == 1)) {
                continue;
            }
            double trianglePointX = x[i] + Rigidbody.get(parentRigidbody).getPosX();
            double trianglePointY = y[i] + Rigidbody.get(parentRigidbody).getPosY();
            double actualDistance = (trianglePointX - other.getX()) * (trianglePointX - other.getX()) + (trianglePointY - other.getY()) * (trianglePointY - other.getY());
            actualDistance = Math.sqrt(actualDistance);
            //to make up for what I assume to be a precision error, we deduct about 1.5 from the actual distance
            //without doing so, the dot product approach needed before would proc before the distance approach needed here, even in
            //situations where the triangle's points are physically in contact, returning the wrong answer
            if (actualDistance <= other.getSolidRadius() + 1.0) {
                intersecting = true;
                double temp = Math.abs(other.getSolidRadius() - actualDistance);
                double nX = (trianglePointX - other.getX()) / actualDistance;
                double nY = (trianglePointY - other.getY()) / actualDistance;
                if (Double.isNaN(overlap) || temp < overlap) {
                    overlap = temp;
                    if (other.isMovable()) {
                        if (Rigidbody.get(parentRigidbody).isMovable()) {
                            temp = overlap * (other.getMass() / (getParentMass() + other.getMass())) + MTV_EPSILON;
                            MTV[0] = temp * nX;
                            MTV[1] = temp * nY;
                            pointOfContact[0] = trianglePointX + MTV[0];
                            pointOfContact[1] = trianglePointY + MTV[1];
                        }
                        else {
                            temp = -overlap - MTV_EPSILON;
                            MTV[0] = temp * nX;
                            MTV[1] = temp * nY;
                            pointOfContact[0] = trianglePointX;
                            pointOfContact[1] = trianglePointY;
                        }
                    }
                    else {
                        temp = overlap + MTV_EPSILON;
                        MTV[0] = temp * nX;
                        MTV[1] = temp * nY;
                        pointOfContact[0] = trianglePointX + MTV[0];
                        pointOfContact[1] = trianglePointY + MTV[1];
                    }

                }
            }
        }
        if (!intersecting && (enabled[0] || enabled[1] || enabled[2])) {
            for (int i = 0; i < 3; i = i + 1) {
                double velocityCheck = other.getVX() * normalX[i] + other.getVY() * normalY[i];
                if (!(mod(indices[(i + 1) % 3] - indices[i], Rigidbody.get(parentRigidbody).getNumPoints()) == 1 && enabled[i])) {
                    continue;
                }
                if (partOfFace && velocityCheck >= 0.0) {
                    intersecting = false;
                    break;
                }
                //in the ortho-normal projection
                dotProductResults[0] = -normalY[i] * (x[i] + Rigidbody.get(parentRigidbody).getPosX()) + normalX[i] * (y[i] + Rigidbody.get(parentRigidbody).getPosY());
                dotProductResults[1] = -normalY[i] * (x[(i + 1) % 3] + Rigidbody.get(parentRigidbody).getPosX()) + normalX[i] * (y[(i + 1) % 3] + Rigidbody.get(parentRigidbody).getPosY());
                dotProductResults[2] = -normalY[i] * other.getX() + normalX[i] * other.getY();
                //in the normal projection
                dotProductResults[3] = normalX[i] * (x[i] + Rigidbody.get(parentRigidbody).getPosX()) + normalY[i] * (y[i] + Rigidbody.get(parentRigidbody).getPosY());
                dotProductResults[4] = normalX[i] * (x[(i + 1) % 3] + Rigidbody.get(parentRigidbody).getPosX()) + normalY[i] * (y[(i + 1) % 3] + Rigidbody.get(parentRigidbody).getPosY());
                dotProductResults[5] = normalX[i] * other.getX() + normalY[i] * other.getY();
                //ball is intersecting if both the ortho-normal and normal projections overlap between the ball and the edges points.
                double orthoDotMin = dotProductResults[0];
                double orthoDotMax = dotProductResults[1];
                if (dotProductResults[1] < dotProductResults[0]) {
                    orthoDotMax = dotProductResults[0];
                    orthoDotMin = dotProductResults[1];
                }
                double dotMin = dotProductResults[3];
                double dotMax = dotProductResults[4];
                if (dotProductResults[4] < dotProductResults[3]) {
                    dotMax = dotProductResults[3];
                    dotMin = dotProductResults[4];
                }
                if (dotProductResults[2] < orthoDotMax && dotProductResults[2] > orthoDotMin) {
                    if (partOfFace && !(dotProductResults[5] - other.getSolidRadius() > dotMin - 1.0)) break;
                    if (dotProductResults[5] - other.getSolidRadius() < dotMax && dotProductResults[5] + other.getSolidRadius() > dotMax) {
                        intersecting = true;
                        double temp = Math.abs(dotMax - dotProductResults[5] + other.getSolidRadius());
                        if (Double.isNaN(overlap) || temp < overlap) {
                            overlap = temp;
                            if (other.isMovable()) {
                                if (Rigidbody.get(parentRigidbody).isMovable()) {
                                    temp = -overlap * (other.getMass() / (other.getMass() + getParentMass())) - MTV_EPSILON;
                                    double temp2 = overlap * (getParentMass() / (other.getMass() + getParentMass())) + MTV_EPSILON;
                                    MTV[0] = normalX[i] * temp;
                                    MTV[1] = normalY[i] * temp;
                                    pointOfContact[0] = other.getX() - other.getSolidRadius() * normalX[i] + normalX[i] * temp2;
                                    pointOfContact[1] = other.getY() - other.getSolidRadius() * normalY[i] + normalY[i] * temp2;
                                }
                                else {
                                    double temp2 = overlap + MTV_EPSILON;
                                    MTV[0] = normalX[i] * temp2;
                                    MTV[1] = normalY[i] * temp2;
                                    pointOfContact[0] = other.getX() - other.getSolidRadius() * normalX[i] + normalX[i] * temp2;
                                    pointOfContact[1] = other.getY() - other.getSolidRadius() * normalY[i] + normalY[i] * temp2;
                                }
                            }
                            else {
                                temp = -overlap - MTV_EPSILON;
                                MTV[0] = normalX[i] * temp;
                                MTV[1] = normalY[i] * temp;
                                pointOfContact[0] = other.getX() - other.getSolidRadius() * normalX[i] + MTV[0];
                                pointOfContact[1] = other.getY() - other.getSolidRadius() * normalY[i] + MTV[1];
                            }
                        }
                    }
                }

            }
        }

        return(new Triplet(intersecting, MTV, pointOfContact));
    }
    private double mod(int a, int b) {
        if (a >= 0) return(a % b);
        else return((a + b) % b);
    }


    public double getArea() {
        return(area);
    }
    public double getMass() {
        return(mass);
    }
    public double getParentMass() {
        return(Rigidbody.get(parentRigidbody).getMass());
    }
    public int getParentID() {
        return(parentRigidbody);
    }
    public double getInertia() {
        return(inertia);
    }
    public double getCenterX() {
        return(centerOfMass[0]);
    }
    public double getCenterY() {
        return(centerOfMass[1]);
    }
    public double[] getX() {
        return(new double[]{x[0] + Rigidbody.get(parentRigidbody).getPosX(), x[1] + Rigidbody.get(parentRigidbody).getPosX(), x[2] + Rigidbody.get(parentRigidbody).getPosX()});
    }
    public double[] getY() {
        return(new double[]{y[0] + Rigidbody.get(parentRigidbody).getPosY(), y[1] + Rigidbody.get(parentRigidbody).getPosY(), y[2] + Rigidbody.get(parentRigidbody).getPosY()});
    }
    public double[] getNormalX() {
        return(normalX);
    }
    public double[] getNormalY() {
        return(normalY);
    }
    public boolean doesExist() {
        return(exists);
    }
    public static void setMTVEpsilon(double x) {
        MTV_EPSILON = x;
    }
}
