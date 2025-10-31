package PhysicsSim;
import PhysicsSim.*;

public class AABBox {
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
            minX.value = Rigidbody.get(parentID).geometry.leftBoundBox + Rigidbody.get(parentID).getPosX();
            maxX.value = Rigidbody.get(parentID).geometry.rightBoundBox + Rigidbody.get(parentID).getPosX();
            minY.value = Rigidbody.get(parentID).geometry.topBoundBox + Rigidbody.get(parentID).getPosY();
            maxY.value = Rigidbody.get(parentID).geometry.bottomBoundBox + Rigidbody.get(parentID).getPosY();
        }
        else if (parentID <= -2) {
            minX.value = Softbody.get(-parentID - 2).minX;
            maxX.value = Softbody.get(-parentID - 2).maxX;
            minY.value = Softbody.get(-parentID - 2).minY;
            maxY.value = Softbody.get(-parentID - 2).maxY;
        }
    }

    public void findCollisions(AABBox aabb) {
        int type = 0; //0 for rigidbodies and 1 for softbodies.
        int index = parentID;
        if (index <= -2) {
            type = 1;
            index = -index - 2;
        }

        int otherType = 0;
        int otherIndex = aabb.parentID;
        if (otherIndex <= -2) {
            otherType = 1;
            otherIndex = -otherIndex - 2;
        }
        switch (type) {
            case(0): {
                switch (otherType) {
                    case(0): {
                        Rigidbody.get(index).checkCollisions(Rigidbody.get(otherIndex));
                        Rigidbody.get(otherIndex).checkCollisions(Rigidbody.get(index));
                        break;
                    }
                    case(1): {
                        if (Softbody.get(otherIndex).boundaryCollision) Rigidbody.get(index).checkCollisions(Softbody.get(otherIndex));
                        break;
                    }
                }
                break;
            }
            case(1): {
                if (Softbody.get(index).boundaryCollision) if (otherType == 0) {
                    Rigidbody.get(otherIndex).checkCollisions(Softbody.get(index));
                }
                break;
            }
        }
    }
}
