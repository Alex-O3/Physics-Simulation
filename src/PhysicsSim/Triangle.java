package PhysicsSim;

import java.util.ArrayList;

class Triangle { //this class is closely tied to the Polygon class

    private double[] x = new double[3];
    private double[] y = new double[3];
    private double[] centerOfMass = new double[2];
    private double area;
    //massless inertia assume mass to be =1 for calculations, such that its inertia = parentRigidbodyMass * masslessInertia
    private double masslessInertia;

    public int[] indices = new int[3];
    private Polygon parentPolygon;
    private int parentRigidbody;
    boolean exists;

    public Triangle(int backwardIndex, int parentIndex, int forwardIndex, Polygon parentPolygon, double[] pos) {
        this.parentPolygon = parentPolygon;
        parentRigidbody = parentPolygon.getParentRigidbodyID();
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
    public void calculateProperties(double parentArea) {
        x[0] = parentPolygon.getXPoints(indices[0]);
        y[0] = parentPolygon.getYPoints(indices[0]);
        x[1] = parentPolygon.getXPoints(indices[1]);
        y[1] = parentPolygon.getYPoints(indices[1]);
        x[2] = parentPolygon.getXPoints(indices[2]);
        y[2] = parentPolygon.getYPoints(indices[2]);
        double yMin = y[0];
        if (y[1] < yMin) yMin = y[1];
        if (y[2] < yMin) yMin = y[2];
        area = 0.0;
        //calculate area and normals
        for (int i = 0; i < 3; i = i + 1) {
            double trapezoidArea = 0.5 * (x[(i + 1) % 3] - x[i]) * (y[(i + 1) % 3] + y[i] - 2.0 * yMin);
            area = area + trapezoidArea;
        }
        area = Math.abs(area);
        double areaProportion = (area / parentArea);
        //side lengths
        double a = Math.sqrt((x[1] - x[0]) * (x[1] - x[0]) + (y[1] - y[0]) * (y[1] - y[0]));
        double b = Math.sqrt((x[2] - x[1]) * (x[2] - x[1]) + (y[2] - y[1]) * (y[2] - y[1]));
        double c = Math.sqrt((x[0] - x[2]) * (x[0] - x[2]) + (y[0] - y[2]) * (y[0] - y[2]));
        double d = (a * a + b * b - c * c) / (2.0 * a); //length until height along the side
        //calculate inertia
        masslessInertia = 2.0 * a * d * (a - d) + d * b * b + (a - d) * c * c; //formula derived from calculus to find the moment of y = (b / a) * x, shifted to find the right triangle's moment by parallel axis theorem
        masslessInertia = masslessInertia / (18.0 * a); //then, parallel axis theorem is used to get the moment of the whole triangle out of the two right triangles that compose it
        masslessInertia = areaProportion * masslessInertia;
    }

    public double getArea() {
        return(area);
    }
    public double getMasslessInertia() {
        return(masslessInertia);
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
    public boolean doesExist() {
        return(exists);
    }
}
