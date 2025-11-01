package PhysicsSim;
import java.util.ArrayList;

 public class BVHNode {
    public final BVHTreeRoot root;
    public final BVHNode leftNode;
    public final BVHNode rightNode;
    private int startIndex;
    private int endIndex;
    public double minX = Double.NaN;
    public double maxX = Double.NaN;
    public double minY = Double.NaN;
    public double maxY = Double.NaN;
    public BVHNode(int startIndex, int endIndex, int simID) {
        this.startIndex = startIndex;
        this.endIndex = endIndex;
        root = Simulation.get(simID).BVHtrees.get(0);
        root.BVHNodes.add(this);
        if (startIndex == endIndex) {
            leftNode = null;
            rightNode = null;
        }
        else {
            int splitIndexInList = (startIndex + endIndex) / 2;
            leftNode = new BVHNode(startIndex, splitIndexInList, simID);
            rightNode = new BVHNode(splitIndexInList + 1, endIndex, simID);
        }
    }
    public void updateBounds() {
        minX = Double.NaN;
        maxX = Double.NaN;
        minY = Double.NaN;
        maxY = Double.NaN;
        for (int i = startIndex; i <= endIndex; i = i + 1) {
            AABBox aabb = root.aabbs.get(i);
            if (Double.isNaN(minX) || aabb.minX.value < minX) minX = aabb.minX.value;
            if (Double.isNaN(maxX) || aabb.maxX.value > maxX) maxX = aabb.maxX.value;
            if (Double.isNaN(minY) || aabb.minY.value < minY) minY = aabb.minY.value;
            if (Double.isNaN(maxY) || aabb.maxY.value > maxY) maxY = aabb.maxY.value;
        }
    }
    private boolean checkOverlapping(AABBox box) {
        return(box.minX.value <= maxX && box.maxX.value >= minX && box.minY.value <= maxY && box.maxY.value >= minY);
    }
    public void traverseTree(AABBox box) {
        if (startIndex == endIndex) {
            if (box.boxID < startIndex) {
                if (checkIfSameSoftbody(box.boxID, root.aabbs.get(startIndex).boxID)) return;
                root.pairs.add(box.boxID);
                root.pairs.add(root.aabbs.get(startIndex).boxID);
            }
        }
        else {
            if (leftNode.checkOverlapping(box)) leftNode.traverseTree(box);
            if (rightNode.checkOverlapping(box)) rightNode.traverseTree(box);
        }
    }
    private boolean checkIfSameSoftbody(int boxIndex1, int boxIndex2) {
        ArrayList<AABBox> aabbs = root.aabbs;
        int parentID1 = aabbs.get(boxIndex1).parentID;
        int parentID2 = aabbs.get(boxIndex2).parentID;
        if (parentID1 <= -2 && parentID2 <= -2) return(parentID1 == parentID2);
        else if (parentID1 <= -2) return(-parentID1 - 2 == Rigidbody.get(parentID2).parentSoftbody);
        else if (parentID2 <= -2) return(-parentID2 - 2 == Rigidbody.get(parentID1).parentSoftbody);
        return(false);
    }
    public static int getMostSignificantBitIndexFromLeft(long a, int startIndexFromLeft) {
        long comparison = Long.MAX_VALUE - (Long.MAX_VALUE >> 1);
        int result = 0;
        a = a << 62 - startIndexFromLeft;
        for (int i = startIndexFromLeft; i >= 0; i = i - 1) {
            if ((a & comparison) != 0) {
                result = i;
                break;
            }
            else a = a << 1;
        }
        return(result);
    }
}
