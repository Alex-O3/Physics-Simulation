package PhysicsSim;

import java.awt.*;
import java.util.ArrayList;

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
    final ArrayList<int[]> rememberedFacesToPass = new ArrayList<>();

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

    /**
     * Carries out the collision detection algorithm between this and another geometricType belonging to a rigidbody. Results are in the form of adding to the parent rigidbody double[2] minimum translation vector and a double[2] pointOfContact position (prior to MTV shift applied).
     * @param otherGeometry the geometry tested against this for collision detection. All implementations of GeometricType must have test for consistency.
     * @return boolean intersection result
     */
    abstract boolean findCollisions(GeometricType otherGeometry);

    /**
     * Finds the collision detection information between a solid joint (double[2] minimum translation vector, double[2] pointOfContact position prior to MTV shift -> added to parent rigidbody fields). Essentially treated as 4 sided polygons, with minor variations in behavior at the edges. The length is defined by Simulation solidJointThickness.
     * @param solidJoint the solid joint in question. Passing in a joint that is not in fact solid will result in errors.
     * @return boolean intersection result
     */
    abstract boolean findCollisions(Joint solidJoint);

    /**
     * Finds the collision detection information (double[2] minimum translation vector, double[2] pointOfContact position prior to MTV shift -> added to parent rigidbody fields) between the calling geometry and the Simulation bounds.
     * @return boolean intersection result
     */
    abstract boolean checkForCollisionsWall();

    /**
     * Rotate this body theta radians about its center of mass.
     * @param theta, in radians, the angle to rotate.
     */
    abstract void rotateAroundCenter(double theta);

    /**
     * Find whether a given point in world space is inside this geometry.
     * @param point the point in space.
     * @return boolean intersection.
     */
    abstract boolean pointInside(double[] point);

    /**
     * Calculate the effect of air resistance due to various factors on torque and force about/on the center of mass of this geometry. Mass is assumed uniform, for consistency. Present implementations adopt simplistic models that, on a rudimentary level, approximate air resistance due to an application of the drag force equation.
     * @param AIR_DENSITY
     * @param DRAG_COEFFICIENT
     * @param windSpeed velocity of the air. Assumed uniform.
     * @param dt the timestep per simulation step this runs in
     */
    abstract double[] calculateAirResistance(double AIR_DENSITY, double DRAG_COEFFICIENT, double[] windSpeed, double dt);

    /**
     *
     * @param shiftX world shift. Deprecated
     * @param resolutionCenterX center of resolution scaling
     * @param pixelShiftX display shift
     * @param shiftY world shift. Deprecated
     * @param resolutionCenterY center of resolution scaling
     * @param pixelShiftY display shift
     * @param resolutionScaling the scaling of resolution (10x -> 10x more screen space for same display size)
     * @return Triplet with encoded pixel display information about drawing the geometry in int pixel coordinates
     */
    abstract Triplet getDrawInt(double shiftX, double resolutionCenterX, double pixelShiftX,
                                double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling);
/**
 *
 * @param shiftX world shift. Deprecated
 * @param resolutionCenterX center of resolution scaling
 * @param pixelShiftX display shift
 * @param shiftY world shift. Deprecated
 * @param resolutionCenterY center of resolution scaling
 * @param pixelShiftY display shift
 * @param resolutionScaling the scaling of resolution (10x -> 10x more screen space for same display size)
 * @return Triplet with encoded pixel display information about drawing the geometry in doubles prior to collapsing to int pixel coordinates
 */
    abstract Triplet getDrawDoubles(double shiftX, double resolutionCenterX, double pixelShiftX,
                             double shiftY, double resolutionCenterY, double pixelShiftY, double resolutionScaling);

    int getParentRigidbodyID() {
        return(parentRigidbodyID);
    }
    Rigidbody getParent() {
        return Rigidbody.get(parentRigidbodyID);
    }
    double getParentMass() {
        return(Rigidbody.get(parentRigidbodyID).getMass());
    }
    double getParentCompoundMass() {
        return(Rigidbody.get(parentRigidbodyID).getCompoundMass());
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
