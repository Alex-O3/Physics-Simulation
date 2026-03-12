package PhysicsSim;
import PhysicsSim.*;

class AABBox {
    public int parentID = -1;
    public int boxID;
    //0 - infinity are rigidbodies, -1 for solid joints
    public Joint solidJoint = null;
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
    public AABBox(Joint solidJoint, int boxIndex) {
        this.solidJoint = solidJoint;
        minX = new Endpoint(boxIndex, true);
        maxX = new Endpoint(boxIndex, false);
        minY = new Endpoint(boxIndex, true);
        maxY = new Endpoint(boxIndex, false);
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
        else if (solidJoint != null) {
            double x1 = solidJoint.parent.getPosX() + solidJoint.offsetFromCMParent[0];
            double x2 = solidJoint.connection.getPosX() + solidJoint.offsetFromCMOther[0];
            double y1 = solidJoint.parent.getPosY() + solidJoint.offsetFromCMParent[1];
            double y2 = solidJoint.connection.getPosY() + solidJoint.offsetFromCMOther[1];
            double lineThickness = Simulation.get(solidJoint.parent.simID).solidJointCollisionLineThickness;
            minX.value = Math.min(x1, x2) - lineThickness;
            maxX.value = Math.max(x1, x2) + lineThickness;
            minY.value = Math.min(y1, y2) - lineThickness;
            maxY.value = Math.max(y1, y2) + lineThickness;
        }
    }

    public void findCollisions(AABBox aabb) {
        int type = 0; //0 for rigidbodies and 1 for solid joints
        int index = parentID;
        if (index == -1 && solidJoint != null) {
            type = 1;
        }

        int otherType = 0;
        int otherIndex = aabb.parentID;
        if (otherIndex == -1 && aabb.solidJoint != null) {
            otherType = 1;
        }
        switch (type) {
            case(0): {
                switch (otherType) {
                    case(0): {
                        if (Rigidbody.get(index).simID != Rigidbody.get(otherIndex).simID) return;
                        if (Rigidbody.get(index).isMovable() || Rigidbody.get(index).isHitbox) Rigidbody.get(index).checkCollisions(Rigidbody.get(otherIndex));
                        if (Rigidbody.get(otherIndex).isMovable() || Rigidbody.get(otherIndex).isHitbox) Rigidbody.get(otherIndex).checkCollisions(Rigidbody.get(index));
                        break;
                    }
                    case(1): {
                        Rigidbody rigidbody = Rigidbody.get(index);
                        if (rigidbody.simID != aabb.solidJoint.parent.simID || rigidbody.simID != aabb.solidJoint.connection.simID) return;
                        if (!rigidbody.isHitbox && aabb.solidJoint.connection != rigidbody && aabb.solidJoint.parent != rigidbody) {
                            rigidbody.checkCollisions(aabb.solidJoint);
                        }
                        break;
                    }
                }
                break;
            }
            case(1): {
                if (otherType == 0) {
                    Rigidbody rigidbody = Rigidbody.get(otherIndex);
                    if (rigidbody.simID != solidJoint.connection.simID || rigidbody.simID != solidJoint.parent.simID) return;
                    if (!rigidbody.isHitbox && solidJoint.connection != rigidbody && solidJoint.parent != rigidbody) {
                        rigidbody.checkCollisions(solidJoint);
                    }
                }
                break;
            }
        }
    }
}
