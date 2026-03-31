package PhysicsSim;

class ConvexPolygon { //this class is closely tied to the Polygon class
    private double[] x;
    private double[] y;
    private double[] centerOfMass = new double[2];
    private double area;

    int[] indices;
    int faceEdgeIndex = -1;
    private double[] normalX;
    private double[] normalY;
    private Polygon parentPolygon;
    private int parentRigidbody;
    private boolean exists;

    public ConvexPolygon(int[] indices, int n, Polygon parentPolygon, double[] pos) {
        this.parentPolygon = parentPolygon;
        parentRigidbody = parentPolygon.getParentRigidbodyID();
        //parent index is the first one
        this.indices = new int[n];
        System.arraycopy(indices, 0, this.indices, 0, n);
        centerOfMass[0] = pos[0];
        centerOfMass[1] = pos[1];
        exists = true;
    }
    public ConvexPolygon(Triangle seed, Polygon parentPolygon) {
        this.parentPolygon = parentPolygon;
        parentRigidbody = parentPolygon.getParentRigidbodyID();
        indices = new int[3];
        indices[0] = seed.indices[1];
        indices[1] = seed.indices[2];
        indices[2] = seed.indices[0];
        centerOfMass[0] = seed.getCenterX();
        centerOfMass[1] = seed.getCenterY();
        exists = true;
    }
    public ConvexPolygon() {
        exists = false;
    }

    public void merge(ConvexPolygon sharingPolygon, int edgePointA, int edgePointB) {
        int[] newindices = new int[indices.length + sharingPolygon.indices.length - 2];
        int indicesIndexParent = -1;
        for (int i = 0; i < indices.length; i++) {
            newindices[i] = indices[i];
            if (indices[i] == edgePointA) {
                indicesIndexParent = i;
                break;
            }
        }
        int indicesIndexSharing = -1;
        for (int i = 0; i < sharingPolygon.indices.length; i++) {
            if (sharingPolygon.indices[i] == edgePointA) {
                indicesIndexSharing = i;
                break;
            }
        }
        for (int j = 1; j < sharingPolygon.indices.length + indicesIndexSharing; j++) {
            newindices[mod(indicesIndexParent + j, newindices.length)] = sharingPolygon.indices[mod(indicesIndexSharing + j, sharingPolygon.indices.length)];
            if (newindices[mod(indicesIndexParent +j, newindices.length)] == edgePointB) {
                indicesIndexSharing = j + indicesIndexParent;
                break;
            }
        }
        for (int i = 1; i < newindices.length - indicesIndexSharing; i++) {
            newindices[mod(indicesIndexSharing + i, newindices.length)] = indices[mod(indicesIndexParent + i, indices.length)];
            if (indicesIndexSharing + i >= newindices.length) break;
        }
        indices = newindices;
    }


    public void calculateProperties(double parentArea) {
        double yMin = parentPolygon.getYPoints(0);
        x = new double[indices.length];
        y = new double[indices.length];
        normalX = new double[indices.length];
        normalY = new double[indices.length];
        for (int n = 0; n < indices.length; n++) {
            x[n] = parentPolygon.getXPoints(indices[n]);
            y[n] = parentPolygon.getYPoints(indices[n]);
            if (y[n] < yMin) yMin = y[n];
        }
        area = 0.0;
        //calculate area and normals
        for (int i = 0; i < indices.length; i = i + 1) {
            double trapezoidArea = 0.5 * (x[(i + 1) % indices.length] - x[i]) * (y[(i + 1) % indices.length] + y[i] - 2.0 * yMin);
            area = area + trapezoidArea;
            normalX[i] = -(y[(i + 1) % indices.length] - y[i]);
            normalY[i] = (x[(i + 1) % indices.length] - x[i]);
            double magnitude = Math.sqrt(normalX[i] * normalX[i] + normalY[i] * normalY[i]);
            normalX[i] = normalX[i] / magnitude;
            normalY[i] = normalY[i] / magnitude;
        }
        if (area < 0.0) for (int i = 0; i < indices.length; i++) {
            normalX[i] = -normalX[i];
            normalY[i] = -normalY[i];
        }
        area = Math.abs(area);
    }
    public void shift(double shiftX, double shiftY) {
        for (int n = 0; n < indices.length; n++) {
            x[n] = x[n] - shiftX;
            y[n] = y[n] - shiftY;
        }
        centerOfMass[0] = centerOfMass[0] - shiftX;
        centerOfMass[1] = centerOfMass[1] - shiftY;
    }
    public void rotate(double theta) {
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        for (int i = 0; i < indices.length; i = i + 1) {
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

    //past this point has yet to be changed for faces. Needs dire correction so that a "memory" system is in place.
    //Faces only collide if the relative velocity at the contact point is moving into the face AND the object has not intersected
    //the face for a handful of frames.
    public Triplet checkCollisions(ConvexPolygon otherConvex) {
        double parallelEdgeTolerance = parentPolygon.getParent().sim.PARALLEL_EDGE_TOLERANCE;

        boolean intersecting = true;
        int incidentEdgeIndex = -1;
        int referenceEdgeIndex = -1;
        double overlap = Double.NaN;
        double overlapAssociatedDot = Double.NaN;
        double[] myX = getX();
        double[] myY = getY();
        double[] otherX = otherConvex.getX();
        double[] otherY = otherConvex.getY();
        double[] selectedNormal = new double[]{Double.NaN, Double.NaN};
        boolean incidentOnOther = false;
        //use separating axis theorem to determine if it is intersecting first
        //for efficiency, also determine the prospective point of contact during this time by finding the "best" normal with the least overlap
        //and satisfiable properties (like symmetry of results for parallel edges). This is why the orthoNormalPos check is performed.

        //check my normals and the other's normal to find the ideal normal, overlap, and intersecting boolean
        for (int i = 0; i < indices.length; i++) {
            double[] myDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] myDotBoundsIndices = new int[]{-1,-1};
            double[] otherDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] otherDotBoundsIndices = new int[]{-1,-1};

            for (int myPoint = 0; myPoint < indices.length; myPoint++) {
                double dot = myX[myPoint] * normalX[i] + myY[myPoint] * normalY[i];
                if (Double.isNaN(myDotBounds[0]) || dot < myDotBounds[0]) {
                    myDotBounds[0] = dot;
                    myDotBoundsIndices[0] = myPoint;
                }
                if (Double.isNaN(myDotBounds[1]) || dot > myDotBounds[1]) {
                    myDotBounds[1] = dot;
                    myDotBoundsIndices[1] = myPoint;
                }
            }
            for (int otherPoint = 0; otherPoint < otherConvex.indices.length; otherPoint++) {
                double dot = otherX[otherPoint] * normalX[i] + otherY[otherPoint] * normalY[i];
                if (Double.isNaN(otherDotBounds[0]) || dot < otherDotBounds[0]) {
                    otherDotBounds[0] = dot;
                    otherDotBoundsIndices[0] = otherPoint;
                }
                if (Double.isNaN(otherDotBounds[1]) || dot > otherDotBounds[1]) {
                    otherDotBounds[1] = dot;
                    otherDotBoundsIndices[1] = otherPoint;
                }
            }

            double tempOverlap = myDotBounds[1] - otherDotBounds[0];
            double referenceEdgeDotMin = myX[i] * -normalY[i] + myY[i] * normalX[i];
            double referenceEdgeDotMax = myX[mod(i + 1, indices.length)] * -normalY[i] + myY[mod(i + 1, indices.length)] * normalX[i];
            if (referenceEdgeDotMax < referenceEdgeDotMin) {
                double temp = referenceEdgeDotMin;
                referenceEdgeDotMin = referenceEdgeDotMax;
                referenceEdgeDotMax = temp;
            }
            double orthoNormalPos = otherX[otherDotBoundsIndices[0]] * -normalY[i] + otherY[otherDotBoundsIndices[0]] * normalX[i];

            if ((Double.isNaN(overlap) || tempOverlap < overlap) && orthoNormalPos < referenceEdgeDotMax && orthoNormalPos > referenceEdgeDotMin) {
                overlap = tempOverlap;
                overlapAssociatedDot = otherDotBounds[0];
                incidentEdgeIndex = otherDotBoundsIndices[0];
                referenceEdgeIndex = i;
                incidentOnOther = true;
                selectedNormal[0] = normalX[i];
                selectedNormal[1] = normalY[i];
            }
            if (!(otherDotBounds[1] > myDotBounds[0] && myDotBounds[1] > otherDotBounds[0])) {
                intersecting = false;
                break;
            }
        }

        for (int i = 0; i < otherConvex.indices.length; i++) {
            double[] myDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] myDotBoundsIndices = new int[]{-1,-1};
            double[] otherDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] otherDotBoundsIndices = new int[]{-1,-1};

            for (int myPoint = 0; myPoint < indices.length; myPoint++) {
                double dot = myX[myPoint] * otherConvex.normalX[i] + myY[myPoint] * otherConvex.normalY[i];
                if (Double.isNaN(myDotBounds[0]) || dot < myDotBounds[0]) {
                    myDotBounds[0] = dot;
                    myDotBoundsIndices[0] = myPoint;
                }
                if (Double.isNaN(myDotBounds[1]) || dot > myDotBounds[1]) {
                    myDotBounds[1] = dot;
                    myDotBoundsIndices[1] = myPoint;
                }
            }
            for (int otherPoint = 0; otherPoint < otherConvex.indices.length; otherPoint++) {
                double dot = otherX[otherPoint] * otherConvex.normalX[i] + otherY[otherPoint] * otherConvex.normalY[i];
                if (Double.isNaN(otherDotBounds[0]) || dot < otherDotBounds[0]) {
                    otherDotBounds[0] = dot;
                    otherDotBoundsIndices[0] = otherPoint;
                }
                if (Double.isNaN(otherDotBounds[1]) || dot > otherDotBounds[1]) {
                    otherDotBounds[1] = dot;
                    otherDotBoundsIndices[1] = otherPoint;
                }
            }

            double tempOverlap = otherDotBounds[1] - myDotBounds[0];
            double referenceEdgeDotMin = otherX[i] * -otherConvex.normalY[i] + otherY[i] * otherConvex.normalX[i];
            double referenceEdgeDotMax = otherX[mod(i + 1, otherConvex.indices.length)] * -otherConvex.normalY[i] + otherY[mod(i + 1, otherConvex.indices.length)] * otherConvex.normalX[i];
            if (referenceEdgeDotMax < referenceEdgeDotMin) {
                double temp = referenceEdgeDotMin;
                referenceEdgeDotMin = referenceEdgeDotMax;
                referenceEdgeDotMax = temp;
            }
            double orthoNormalPos = myX[myDotBoundsIndices[0]] * -otherConvex.normalY[i] + myY[myDotBoundsIndices[0]] * otherConvex.normalX[i];

            if ((Double.isNaN(overlap) || tempOverlap < overlap) && orthoNormalPos < referenceEdgeDotMax && orthoNormalPos > referenceEdgeDotMin) {
                overlap = tempOverlap;
                overlapAssociatedDot = myDotBounds[0];
                incidentEdgeIndex = myDotBoundsIndices[0];
                referenceEdgeIndex = i;
                incidentOnOther = false;
                selectedNormal[0] = otherConvex.normalX[i];
                selectedNormal[1] = otherConvex.normalY[i];
            }
            if (!(otherDotBounds[1] > myDotBounds[0] && myDotBounds[1] > otherDotBounds[0])) {
                intersecting = false;
                break;
            }
        }

        //determine point of contact, accounting for potentially parallel edges
        double[] pointOfContact = new double[]{Double.NaN, Double.NaN};
        double[] MTV = new double[]{Double.NaN, Double.NaN};
        if (intersecting) {
            if (incidentOnOther) {
                pointOfContact = new double[]{otherX[incidentEdgeIndex], otherY[incidentEdgeIndex]};
                double ldot = otherX[mod(incidentEdgeIndex - 1, otherConvex.indices.length)] * selectedNormal[0] + otherY[mod(incidentEdgeIndex - 1, otherConvex.indices.length)] * selectedNormal[1];
                double rdot = otherX[mod(incidentEdgeIndex + 1, otherConvex.indices.length)] * selectedNormal[0] + otherY[mod(incidentEdgeIndex + 1, otherConvex.indices.length)] * selectedNormal[1];
                if (Math.abs(rdot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + otherX[mod(incidentEdgeIndex + 1, otherConvex.indices.length)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + otherY[mod(incidentEdgeIndex + 1, otherConvex.indices.length)]);
                } else if (Math.abs(ldot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + otherX[mod(incidentEdgeIndex - 1, otherConvex.indices.length)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + otherY[mod(incidentEdgeIndex - 1, otherConvex.indices.length)]);
                }
            }
            else {
                pointOfContact = new double[]{myX[incidentEdgeIndex], myY[incidentEdgeIndex]};
                double ldot = myX[mod(incidentEdgeIndex - 1, indices.length)] * selectedNormal[0] + myY[mod(incidentEdgeIndex - 1, indices.length)] * selectedNormal[1];
                double rdot = myX[mod(incidentEdgeIndex + 1, indices.length)] * selectedNormal[0] + myY[mod(incidentEdgeIndex + 1, indices.length)] * selectedNormal[1];
                if (Math.abs(rdot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + myX[mod(incidentEdgeIndex + 1, indices.length)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + myY[mod(incidentEdgeIndex + 1, indices.length)]);
                } else if (Math.abs(ldot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + myX[mod(incidentEdgeIndex - 1, indices.length)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + myY[mod(incidentEdgeIndex - 1, indices.length)]);
                }
            }

            //find minimum translation vector and distribute evenly among each body according to its share of the total system mass
            double multiplier = (otherConvex.parentPolygon.getParentCompoundMass()) / (otherConvex.parentPolygon.getParentCompoundMass() + parentPolygon.getParentCompoundMass());
            multiplier = Rigidbody.get(otherConvex.parentRigidbody).isMovable() ? multiplier : 1.0;
            multiplier = incidentOnOther ? -multiplier : multiplier;
            double MTV_EPSILON = Rigidbody.get(parentRigidbody).sim.MTV_EPSILON;
            MTV = new double[]{(overlap + MTV_EPSILON) * multiplier * selectedNormal[0], (overlap + MTV_EPSILON) * multiplier * selectedNormal[1]};

            if (otherConvex.faceEdgeIndex != -1) {
                if ((!incidentOnOther && otherConvex.faceEdgeIndex != referenceEdgeIndex) ||
                (incidentOnOther && incidentEdgeIndex != otherConvex.faceEdgeIndex
                        && mod(incidentEdgeIndex - 1, otherConvex.indices.length) != otherConvex.faceEdgeIndex)) passFace(parentPolygon.getParent(), otherConvex.parentPolygon.getParent());
            }
            else if (faceEdgeIndex != -1) {
                if ((incidentOnOther && faceEdgeIndex != referenceEdgeIndex) ||
                (!incidentOnOther && incidentEdgeIndex != faceEdgeIndex
                        && mod(incidentEdgeIndex - 1, indices.length) != faceEdgeIndex)) passFace(otherConvex.parentPolygon.getParent(), parentPolygon.getParent());
            }
        }


        //now that the ideal normal and non-shifted point of contact has been found, find the incident edge from its index
        //and which of the two options branching from that point is more parallel to the reference edge

        return new Triplet(intersecting, MTV, pointOfContact);

    }
    public Triplet checkCollisions(Circle otherCircle) {
        boolean intersecting = false;
        int incidentEdgeIndex = -1;
        int referenceEdgeIndex = -1;
        double overlap = Double.NaN;
        double[] myX = getX();
        double[] myY = getY();
        double otherX = Rigidbody.get(otherCircle.getParentRigidbodyID()).getPosX();
        double otherY = Rigidbody.get(otherCircle.getParentRigidbodyID()).getPosY();
        double radius = otherCircle.getRadius();
        double[] selectedNormal = new double[]{Double.NaN, Double.NaN};
        boolean incidentOnOther = false;

        //find ideal normal and boolean intersecting

        //check edge points first
        for (int pointIndex = 0; pointIndex < indices.length; pointIndex++) {
            double distance = (myX[pointIndex] - otherX) * (myX[pointIndex] - otherX) + (myY[pointIndex] - otherY) * (myY[pointIndex] - otherY);
            distance = Math.sqrt(distance);
            double tempOverlap = radius - distance;
            if ((!Double.isFinite(overlap) || tempOverlap < overlap) && tempOverlap > 0.0) {
                overlap = tempOverlap;
                incidentOnOther = false;
                incidentEdgeIndex = pointIndex;
                selectedNormal[0] = (myX[pointIndex] - otherX) / distance;
                selectedNormal[1] = (myY[pointIndex] - otherY) / distance;
                intersecting = true;
            }
        }

        //then the edges themselves
        if (!intersecting) {
            intersecting = true;
            boolean foundValid = false;
            for (int edgeIndex = 0; edgeIndex < indices.length; edgeIndex++) {
                double signedDistance = (otherX - myX[edgeIndex]) * normalX[edgeIndex] + (otherY - myY[edgeIndex]) * normalY[edgeIndex];
                double tempOverlap = radius - signedDistance;
                double referenceEdgeDotMin = myX[edgeIndex] * -normalY[edgeIndex] + myY[edgeIndex] * normalX[edgeIndex];
                double referenceEdgeDotMax = myX[mod(edgeIndex + 1, indices.length)] * -normalY[edgeIndex] + myY[mod(edgeIndex + 1, indices.length)] * normalX[edgeIndex];
                if (referenceEdgeDotMax < referenceEdgeDotMin) {
                    double temp = referenceEdgeDotMin;
                    referenceEdgeDotMin = referenceEdgeDotMax;
                    referenceEdgeDotMax = temp;
                }
                double orthoNormalPos = otherX * -normalY[edgeIndex] + otherY * normalX[edgeIndex];
                double dotMin = Double.NaN;
                double dotMax = Double.NaN;
                for (int pointIndex = 0; pointIndex < indices.length; pointIndex++) {
                    double dot = (myX[pointIndex] - myX[edgeIndex]) * normalX[edgeIndex] + (myY[pointIndex] - myY[edgeIndex]) * normalY[edgeIndex];
                    if (!Double.isFinite(dotMin) || dot < dotMin) dotMin = dot;
                    if (!Double.isFinite(dotMax) || dot > dotMax) dotMax = dot;
                }

                if (!(signedDistance - radius < dotMax && signedDistance + radius > dotMin)) {
                    intersecting = false;
                    break;
                }
                else if (orthoNormalPos > referenceEdgeDotMin && orthoNormalPos < referenceEdgeDotMax && signedDistance < radius && signedDistance > -radius && (!Double.isFinite(overlap) || tempOverlap < overlap)) {
                    foundValid = true;
                    overlap = tempOverlap;
                    incidentOnOther = true;
                    referenceEdgeIndex = edgeIndex;
                    selectedNormal[0] = normalX[edgeIndex];
                    selectedNormal[1] = normalY[edgeIndex];
                }
            }
            if (!foundValid && !parentPolygon.getParent().isHitbox) intersecting = false;
        }

        //then another edges check for the special case where it contains the circle
        if (!intersecting && !parentPolygon.getParent().isHitbox) {
            intersecting = true;
            for (int edgeIndex = 0; edgeIndex < indices.length; edgeIndex++) {
                double signedDistance = (otherX - myX[edgeIndex]) * normalX[edgeIndex] + (otherY - myY[edgeIndex]) * normalY[edgeIndex];
                signedDistance = -signedDistance;

                if (signedDistance < 0.0) {
                    intersecting = false;
                    break;
                }
                else if (!Double.isFinite(overlap) || signedDistance < overlap) {
                    overlap = signedDistance;
                    incidentOnOther = true;
                    referenceEdgeIndex = edgeIndex;
                    selectedNormal[0] = normalX[edgeIndex];
                    selectedNormal[1] = normalY[edgeIndex];
                }
            }
        }

        double[] pointOfContact = new double[]{Double.NaN, Double.NaN};
        double[] MTV = new double[]{Double.NaN, Double.NaN};
        if (intersecting && (incidentOnOther || incidentEdgeIndex != -1)) {
            if (incidentOnOther) {
                pointOfContact[0] = otherX - radius * selectedNormal[0];
                pointOfContact[1] = otherY - radius * selectedNormal[1];
            }
            else {
                pointOfContact[0] = myX[incidentEdgeIndex];
                pointOfContact[1] = myY[incidentEdgeIndex];
            }

            //find minimum translation vector and distribute evenly among each body according to its share of the total system mass
            double multiplier = (otherCircle.getParentCompoundMass()) / (otherCircle.getParentCompoundMass() + parentPolygon.getParentCompoundMass());
            multiplier = otherCircle.getParent().isMovable() ? multiplier : 1.0;
            multiplier = incidentOnOther ? -multiplier : multiplier;
            double MTV_EPSILON = Rigidbody.get(parentRigidbody).sim.MTV_EPSILON;
            MTV = new double[]{(overlap + MTV_EPSILON) * multiplier * selectedNormal[0], (overlap + MTV_EPSILON) * multiplier * selectedNormal[1]};

            if (faceEdgeIndex != -1) {
                if ((incidentOnOther && faceEdgeIndex != referenceEdgeIndex) ||
                        (!incidentOnOther && incidentEdgeIndex != faceEdgeIndex
                                && mod(incidentEdgeIndex - 1, indices.length) != faceEdgeIndex)) passFace(otherCircle.getParent(), parentPolygon.getParent());
            }
        }

        return new Triplet(intersecting, MTV, pointOfContact);
    }
    private void passFace(Rigidbody other, Rigidbody face) {
        if (!other.geometry.rememberedFacesToPass.contains(face.ID)) other.geometry.rememberedFacesToPass.add(face.ID);
    }
    public Triplet checkCollisions(double[] jointPosX, double[] jointPosY, double[] nX, double[] nY, double jointCompoundMass, boolean jointIsMovable) {
        double parallelEdgeTolerance = parentPolygon.getParent().sim.PARALLEL_EDGE_TOLERANCE;

        boolean intersecting = true;
        int incidentEdgeIndex = -1;
        int referenceEdgeIndex = -1;
        double overlap = Double.NaN;
        double overlapAssociatedDot = Double.NaN;
        double[] myX = getX();
        double[] myY = getY();
        double[] selectedNormal = new double[]{Double.NaN, Double.NaN};
        boolean incidentOnOther = false;
        //use separating axis theorem to determine if it is intersecting first
        //for efficiency, also determine the prospective point of contact during this time by finding the "best" normal with the least overlap
        //and satisfiable properties (like symmetry of results for parallel edges). This is why the orthoNormalPos check is performed.

        //check my normals and the other's normal to find the ideal normal, overlap, and intersecting boolean
        for (int i = 0; i < indices.length; i++) {
            double[] myDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] myDotBoundsIndices = new int[]{-1,-1};
            double[] otherDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] otherDotBoundsIndices = new int[]{-1,-1};

            for (int myPoint = 0; myPoint < indices.length; myPoint++) {
                double dot = myX[myPoint] * normalX[i] + myY[myPoint] * normalY[i];
                if (Double.isNaN(myDotBounds[0]) || dot < myDotBounds[0]) {
                    myDotBounds[0] = dot;
                    myDotBoundsIndices[0] = myPoint;
                }
                if (Double.isNaN(myDotBounds[1]) || dot > myDotBounds[1]) {
                    myDotBounds[1] = dot;
                    myDotBoundsIndices[1] = myPoint;
                }
            }
            for (int otherPoint = 0; otherPoint < 4; otherPoint++) {
                double dot = jointPosX[otherPoint] * normalX[i] + jointPosY[otherPoint] * normalY[i];
                if (Double.isNaN(otherDotBounds[0]) || dot < otherDotBounds[0]) {
                    otherDotBounds[0] = dot;
                    otherDotBoundsIndices[0] = otherPoint;
                }
                if (Double.isNaN(otherDotBounds[1]) || dot > otherDotBounds[1]) {
                    otherDotBounds[1] = dot;
                    otherDotBoundsIndices[1] = otherPoint;
                }
            }

            double tempOverlap = myDotBounds[1] - otherDotBounds[0];
            double referenceEdgeDotMin = myX[i] * -normalY[i] + myY[i] * normalX[i];
            double referenceEdgeDotMax = myX[mod(i + 1, indices.length)] * -normalY[i] + myY[mod(i + 1, indices.length)] * normalX[i];
            if (referenceEdgeDotMax < referenceEdgeDotMin) {
                double temp = referenceEdgeDotMin;
                referenceEdgeDotMin = referenceEdgeDotMax;
                referenceEdgeDotMax = temp;
            }
            double orthoNormalPos = jointPosX[otherDotBoundsIndices[0]] * -normalY[i] + jointPosY[otherDotBoundsIndices[0]] * normalX[i];

            if ((Double.isNaN(overlap) || tempOverlap < overlap) && orthoNormalPos < referenceEdgeDotMax && orthoNormalPos > referenceEdgeDotMin) {
                overlap = tempOverlap;
                overlapAssociatedDot = otherDotBounds[0];
                incidentEdgeIndex = otherDotBoundsIndices[0];
                referenceEdgeIndex = i;
                incidentOnOther = true;
                selectedNormal[0] = normalX[i];
                selectedNormal[1] = normalY[i];
            }
            if (!(otherDotBounds[1] > myDotBounds[0] && myDotBounds[1] > otherDotBounds[0])) {
                intersecting = false;
                break;
            }
        }

        for (int i = 0; i < 4; i++) {
            double[] myDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] myDotBoundsIndices = new int[]{-1,-1};
            double[] otherDotBounds = new double[]{Double.NaN, Double.NaN};
            int[] otherDotBoundsIndices = new int[]{-1,-1};

            for (int myPoint = 0; myPoint < indices.length; myPoint++) {
                double dot = myX[myPoint] * nX[i] + myY[myPoint] * nY[i];
                if (Double.isNaN(myDotBounds[0]) || dot < myDotBounds[0]) {
                    myDotBounds[0] = dot;
                    myDotBoundsIndices[0] = myPoint;
                }
                if (Double.isNaN(myDotBounds[1]) || dot > myDotBounds[1]) {
                    myDotBounds[1] = dot;
                    myDotBoundsIndices[1] = myPoint;
                }
            }
            for (int otherPoint = 0; otherPoint < 4; otherPoint++) {
                double dot = jointPosX[otherPoint] * nX[i] + jointPosY[otherPoint] * nY[i];
                if (Double.isNaN(otherDotBounds[0]) || dot < otherDotBounds[0]) {
                    otherDotBounds[0] = dot;
                    otherDotBoundsIndices[0] = otherPoint;
                }
                if (Double.isNaN(otherDotBounds[1]) || dot > otherDotBounds[1]) {
                    otherDotBounds[1] = dot;
                    otherDotBoundsIndices[1] = otherPoint;
                }
            }

            double tempOverlap = otherDotBounds[1] - myDotBounds[0];
            double referenceEdgeDotMin = jointPosX[i] * -nY[i] + jointPosY[i] * nX[i];
            double referenceEdgeDotMax = jointPosX[mod(i + 1, 4)] * -nY[i] + jointPosY[mod(i + 1, 4)] * nX[i];
            if (referenceEdgeDotMax < referenceEdgeDotMin) {
                double temp = referenceEdgeDotMin;
                referenceEdgeDotMin = referenceEdgeDotMax;
                referenceEdgeDotMax = temp;
            }
            double orthoNormalPos = myX[myDotBoundsIndices[0]] * -nY[i] + myY[myDotBoundsIndices[0]] * nX[i];

            if ((Double.isNaN(overlap) || tempOverlap < overlap) && orthoNormalPos < referenceEdgeDotMax && orthoNormalPos > referenceEdgeDotMin) {
                overlap = tempOverlap;
                overlapAssociatedDot = myDotBounds[0];
                incidentEdgeIndex = myDotBoundsIndices[0];
                referenceEdgeIndex = i;
                incidentOnOther = false;
                selectedNormal[0] = nX[i];
                selectedNormal[1] = nY[i];
            }
            if (!(otherDotBounds[1] > myDotBounds[0] && myDotBounds[1] > otherDotBounds[0])) {
                intersecting = false;
                break;
            }
        }

        //determine point of contact, accounting for potentially parallel edges
        double[] pointOfContact = new double[]{Double.NaN, Double.NaN};
        double[] MTV = new double[]{Double.NaN, Double.NaN};
        if (intersecting) {
            if (incidentOnOther) {
                pointOfContact = new double[]{jointPosX[incidentEdgeIndex], jointPosY[incidentEdgeIndex]};
                double ldot = jointPosX[mod(incidentEdgeIndex - 1, 4)] * selectedNormal[0] + jointPosY[mod(incidentEdgeIndex - 1, 4)] * selectedNormal[1];
                double rdot = jointPosX[mod(incidentEdgeIndex + 1, 4)] * selectedNormal[0] + jointPosY[mod(incidentEdgeIndex + 1, 4)] * selectedNormal[1];
                if (Math.abs(rdot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + jointPosX[mod(incidentEdgeIndex + 1, 4)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + jointPosY[mod(incidentEdgeIndex + 1, 4)]);
                } else if (Math.abs(ldot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + jointPosX[mod(incidentEdgeIndex - 1, 4)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + jointPosY[mod(incidentEdgeIndex - 1, 4)]);
                }
            }
            else {
                pointOfContact = new double[]{myX[incidentEdgeIndex], myY[incidentEdgeIndex]};
                double ldot = myX[mod(incidentEdgeIndex - 1, indices.length)] * selectedNormal[0] + myY[mod(incidentEdgeIndex - 1, indices.length)] * selectedNormal[1];
                double rdot = myX[mod(incidentEdgeIndex + 1, indices.length)] * selectedNormal[0] + myY[mod(incidentEdgeIndex + 1, indices.length)] * selectedNormal[1];
                if (Math.abs(rdot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + myX[mod(incidentEdgeIndex + 1, indices.length)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + myY[mod(incidentEdgeIndex + 1, indices.length)]);
                } else if (Math.abs(ldot - overlapAssociatedDot) <= parallelEdgeTolerance) {
                    pointOfContact[0] = 0.5 * (pointOfContact[0] + myX[mod(incidentEdgeIndex - 1, indices.length)]);
                    pointOfContact[1] = 0.5 * (pointOfContact[1] + myY[mod(incidentEdgeIndex - 1, indices.length)]);
                }
            }

            //find minimum translation vector and distribute evenly among each body according to its share of the total system mass
            double multiplier = (jointCompoundMass) / (jointCompoundMass + parentPolygon.getParentCompoundMass());
            multiplier = jointIsMovable ? multiplier : 1.0;
            multiplier = incidentOnOther ? -multiplier : multiplier;
            double MTV_EPSILON = Rigidbody.get(parentRigidbody).sim.MTV_EPSILON;
            MTV = new double[]{(overlap + MTV_EPSILON) * multiplier * selectedNormal[0], (overlap + MTV_EPSILON) * multiplier * selectedNormal[1]};
        }


        //now that the ideal normal and non-shifted point of contact has been found, find the incident edge from its index
        //and which of the two options branching from that point is more parallel to the reference edge

        return new Triplet(intersecting, MTV, pointOfContact);
    }
    static int mod(int a, int b) {
        if (a >= 0) return(a % b);
        else {
            int tempResult = b - (-a % b);
            if (tempResult == b) return(0);
            else return(tempResult);
        }
    }
    public int size() {
        return indices.length;
    }


    public double getArea() {
        return(area);
    }
    public double getCenterX() {
        return(centerOfMass[0]);
    }
    public double getCenterY() {
        return(centerOfMass[1]);
    }
    public double[] getX() {
        double[] X = new double[x.length];
        for (int n = 0; n < x.length; n++) {
            X[n] = x[n] + Rigidbody.get(parentRigidbody).getPosX();
        }
        return X;
    }
    public double[] getY() {
        double[] Y = new double[y.length];
        for (int n = 0; n < x.length; n++) {
            Y[n] = y[n] + Rigidbody.get(parentRigidbody).getPosY();
        }
        return Y;
    }
    public boolean doesExist() {
        return(exists);
    }
}
