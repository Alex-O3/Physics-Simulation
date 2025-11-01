package PhysicsSim;

import java.awt.*;

/**
 * Fields that must be assigned values for proper functioning:
 * masslessInertia - the inertia of the geometry divided by mass |
 * color - the color of the geometry if it were to be drawn as a single solid color |
 * area - the area of the geometry, unsigned |
 * bottomBoundBox, topBoundBox, leftBoundBox, rightBoundBox - the maxY, minY, minX, and maxX values respectively (in that order) |
 * largestDistanceSquared, largestDistance - the greatest distance from the center of mass to any point (invariant under rotation about center of mass)
 */

abstract class GeometricType {
    private Color color = Color.black;
    double masslessInertia;
    double area;
    double bottomBoundBox;
    double topBoundBox;
    double leftBoundBox;
    double rightBoundBox;
    double largestDistanceSquared = 0.0;
    double largestDistance;
    private final int parentRigidbodyID;

    protected GeometricType(int parentRigidbodyID) {
        this.parentRigidbodyID = parentRigidbodyID;
    }

    double getMasslessInertia() {
        return (masslessInertia);
    }
    double getArea() {
        return (area);
    }
    double getLargestDistanceSquared() {
        return(largestDistanceSquared);
    }
    double getLargestDistance() {
        return(largestDistance);
    }
    abstract boolean findCollisions(GeometricType otherGeometry);
    abstract boolean findCollisions(Softbody softbody);
    abstract boolean checkForCollisionsWall();
    abstract void rotateAroundCenter(double theta);
    abstract boolean pointInside(double[] point);
    abstract double[] calculateAirResistance(double AIR_DENSITY, double DRAG_COEFFICIENT, double[] v);
    abstract Triplet getDrawInt(double shiftX, double resolutionCenterX, double pixelShiftX,
                                double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling);
    abstract Triplet getDrawDoubles(double shiftX, double resolutionCenterX, double pixelShiftX,
                             double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling);

    int getParentRigidbodyID() {
        return(parentRigidbodyID);
    }
    static int mod(int a, int b) {
        if (a >= 0) return(a % b);
        else {
            int tempResult = b - (-a % b);
            if (tempResult == b) return(0);
            else return(tempResult);
        }
    }
    void setColor(Color color) {
        this.color = color;
    }
    Color getColor() {
        return(color);
    }
}
