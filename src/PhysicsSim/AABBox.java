package PhysicsSim;

class AABBox {
    public int parentID = -1;
    public int boxID;
    //0 - infinity are rigidbodies, even negative numbers are points, and odd negative numbers are softbodies
    //very important: this is different from the colliding ID convention in how the odd negative IDs are used
    public Endpoint minX;
    public Endpoint maxX;
    public Endpoint minY;
    public Endpoint maxY;
    public long mortonCode;
    public AABBox(int parentID, int boxIndex) {
        this.parentID = parentID;
        minX = new Endpoint(boxIndex, true);
        maxX = new Endpoint(boxIndex, false);
        minY = new Endpoint(boxIndex, true);
        maxY = new Endpoint(boxIndex, false);
        updateBox();
    }
    public void updateBox() {
        if (parentID >= 0) {
            minX.value = Rigidbody.get(parentID).leftBoundBox + Rigidbody.get(parentID).posX;
            maxX.value = Rigidbody.get(parentID).rightBoundBox + Rigidbody.get(parentID).posX;
            minY.value = Rigidbody.get(parentID).topBoundBox + Rigidbody.get(parentID).posY;
            maxY.value = Rigidbody.get(parentID).bottomBoundBox + Rigidbody.get(parentID).posY;
        }
        else if (Rigidbody.mod(parentID, 2) == 0) {
            int index = (-parentID / 2) - 1;
            minX.value = Point.get(index).posX - Point.get(index).getSolidRadius();
            maxX.value = Point.get(index).posX + Point.get(index).getSolidRadius();
            minY.value = Point.get(index).posY - Point.get(index).getSolidRadius();
            maxY.value = Point.get(index).posY + Point.get(index).getSolidRadius();
        }
        else if (Rigidbody.mod(parentID, 2) == 1) {
            int index = (-parentID - 1) / 2;
            minX.value = Softbody.get(index).minX;
            maxX.value = Softbody.get(index).maxX;
            minY.value = Softbody.get(index).minY;
            maxY.value = Softbody.get(index).maxY;
        }
    }

    public void findCollisions(AABBox aabb) {
        int type = 0; //0 for rigidbodies and 1 for points. Assuming no aabbs are created for softbodies nor the walls.
        int index = parentID;
        if (index < 0) {
            if (Rigidbody.mod(index, 2) == 0) {
                index = (-index / 2) - 1;
                type = 1;
            }
            else {
                index = (-index - 1) / 2;
                type = 2;
            }
        }

        int otherType = 0;
        int otherIndex = aabb.parentID;
        if (otherIndex < 0) {
            if (Rigidbody.mod(otherIndex, 2) == 0) {
                otherIndex = (-otherIndex / 2) - 1;
                otherType = 1;
            }
            else {
                otherIndex = (-otherIndex - 1) / 2;
                otherType = 2;
            }
        }
        switch (type) {
            case(0): {
                switch (otherType) {
                    case(0): {
                        Rigidbody.get(index).checkForCollisionsNarrow(Rigidbody.get(otherIndex));
                        Rigidbody.get(otherIndex).checkForCollisionsNarrow(Rigidbody.get(index));
                        break;
                    }
                    case(1): {
                        Rigidbody.get(index).checkForCollisionsNarrow(Point.get(otherIndex));
                        Point.get(otherIndex).checkForCollisionsNarrow(Rigidbody.get(index));
                        break;
                    }
                    case(2): {
                        if (Softbody.get(otherIndex).boundaryCollision) Rigidbody.get(index).checkForCollisionsNarrow(Softbody.get(otherIndex));
                        break;
                    }
                }
                break;
            }
            case(1): {
                switch (otherType) {
                    case(0): {
                        Point.get(index).checkForCollisionsNarrow(Rigidbody.get(otherIndex));
                        Rigidbody.get(otherIndex).checkForCollisionsNarrow(Point.get(index));
                        break;
                    }
                    case(1): {
                        Point.get(index).checkForCollisions(Point.get(otherIndex));
                        Point.get(otherIndex).checkForCollisions(Point.get(index));
                        break;
                    }
                    case(2): {
                        if (Softbody.get(otherIndex).boundaryCollision && Point.get(index).parentSoftbody != Softbody.get(otherIndex).ID) Point.get(index).checkForCollisionsNarrow(Softbody.get(otherIndex));
                        break;
                    }
                }
                break;
            }
            case(2): {
                if (Softbody.get(index).boundaryCollision) switch (otherType) {
                    case(0): {
                        Rigidbody.get(otherIndex).checkForCollisionsNarrow(Softbody.get(index));
                        break;
                    }
                    case(1): {
                        Point.get(otherIndex).checkForCollisionsNarrow(Softbody.get(index));
                        break;
                    }
                }
                break;
            }
        }
    }
}
