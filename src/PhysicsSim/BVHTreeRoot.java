package PhysicsSim;
import java.util.ArrayList;

class BVHTreeRoot {
    public int simID;
    public ArrayList<AABBox> aabbs = new ArrayList<>();
    public final int pow2numCells;
    private BVHNode rootNode;
    public final ArrayList<BVHNode> BVHNodes = new ArrayList<>();
    public final ArrayList<Integer> pairs = new ArrayList<>();


    public BVHTreeRoot(int simID, int powerOf2NumOfCells) {
        this.simID = simID;
        pow2numCells = powerOf2NumOfCells;
        Simulation.get(simID).BVHtrees.add(this);
    }

    public void addBox(int parentID) {
        AABBox aabb = new AABBox(parentID, aabbs.size());
        aabbs.add(aabb);
    }
    public void createTree() {
        //this uses morton codes, which "creates" a tiling where the integer x and y value of the cell
        //containing the centroid of a box are interleaved to conserve spatial locality in a single long (64-bit)
        //since there are 2^pow2numCells along each axis, each successive bit in a morton code divides the
        //space into 2 even halves of cells, alternating between along the x and the y axes

        BVHNodes.clear();

        updateBounds();

        rootNode = new BVHNode(0, aabbs.size() - 1, simID);
    }
    public void updateBounds() {
        double minX = Double.NaN;
        double maxX = Double.NaN;
        double minY = Double.NaN;
        double maxY = Double.NaN;
        for (AABBox aabb : aabbs) {
            aabb.updateBox();
            if (Double.isNaN(minX) || aabb.minX.value < minX) minX = aabb.minX.value;
            if (Double.isNaN(maxX) || aabb.maxX.value > maxX) maxX = aabb.maxX.value;
            if (Double.isNaN(minY) || aabb.minY.value < minY) minY = aabb.minY.value;
            if (Double.isNaN(maxY) || aabb.maxY.value > maxY) maxY = aabb.maxY.value;
        }

        //determine which cell in a tiling made for morton codes the centroid of each object box lies in and then generate
        //the morton code for that box
        double invCellSizeX =  (maxX - minX) / Math.pow(2, pow2numCells);
        double invCellSizeY = (maxY - minY) / Math.pow(2, pow2numCells);
        for (AABBox aabb : aabbs) {
            double[] pos = new double[]{(aabb.minX.value + aabb.maxX.value) * 0.5, (aabb.minY.value + aabb.maxY.value) * 0.5};
            int mortonX = (int) Math.floor((pos[0] - minX) / invCellSizeX);
            int mortonY = (int) Math.floor((pos[1] - minY) / invCellSizeY);
            aabb.mortonCode = interleaveInts(mortonX, mortonY);
        }

        for (BVHNode node : BVHNodes) {
            node.updateBounds();
        }
    }
    public void rebalanceTree() {
        //assumes bounds have already been updated
        quickSortAABBs(aabbs);
    }
    public void findPairs() {
        pairs.clear();
        for (AABBox aabb : aabbs) {
            rootNode.traverseTree(aabb);
        }
    }

    public static long interleaveInts(int a, int b) {
        long result = 0;
        for (int i = 0; i < 16; i = i + 1) {
            int bit1 = a & 1;
            int bit2 = b & 1;
            a = a >> 1;
            b = b >> 1;
            result |= (bit1 << (2 * i));
            result |= (bit2 << (2 * i + 1));
        }
        return(result);
    }
    private void insertionSortAABBs() {
        for (int i = 1; i < aabbs.size(); i = i + 1) {
            if (aabbs.get(i).mortonCode > aabbs.get(i - 1).mortonCode) { //here is the compare function
                int insertionIndex = 0;
                AABBox insertionValue = aabbs.get(i);
                for (int j = i - 1; j >= 0; j = j - 1) {
                    if (!(aabbs.get(i).mortonCode > aabbs.get(i - 1).mortonCode)) { //here as well, but inverted
                        insertionIndex = j + 1;
                        break;
                    }
                }
                for (int j = i; j > insertionIndex; j = j - 1) {
                    aabbs.set(j, aabbs.get(j - 1));
                    aabbs.get(j - 1).boxID = j;
                }
                aabbs.set(insertionIndex, insertionValue);
                insertionValue.boxID = insertionIndex;
            }
        }
    }

    private static void quickSortAABBs(ArrayList<AABBox> arr) {
        qsort(arr, 0, arr.size() - 1);
    }

    private static void qsort(ArrayList<AABBox> arr, int start, int end) {
        if (end <= start) return;
        int pivot = end;
        int leftRightmostIndex = start;
        AABBox temp;
        for (int j = start; j < end; j = j + 1) {
            if (arr.get(j).mortonCode > arr.get(pivot).mortonCode) { //here is where the compare function is done
                temp = arr.get(j);
                arr.set(j, arr.get(leftRightmostIndex));
                arr.get(j).boxID = j;
                arr.set(leftRightmostIndex, temp);
                arr.get(leftRightmostIndex).boxID = leftRightmostIndex;
                leftRightmostIndex += 1;
            }
        }
        temp = arr.get(leftRightmostIndex);
        arr.set(leftRightmostIndex, arr.get(pivot));
        arr.get(leftRightmostIndex).boxID = leftRightmostIndex;
        arr.set(pivot, temp);
        arr.get(pivot).boxID = pivot;
        qsort(arr, start, leftRightmostIndex - 1);
        qsort(arr, leftRightmostIndex + 1, end);
    }
}
