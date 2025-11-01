package PhysicsSim;

import java.util.ArrayList;

 public class SAPCell {
    public final int simID;
    public final int x;
    public final int y;
    public final ArrayList<AABBox> aabbs = new ArrayList<>();
    public final ArrayList<Integer> activelyChecked = new ArrayList<>();
    public final ArrayList<Integer> pairs = new ArrayList<>();
    public final ArrayList<Endpoint> endpointsX = new ArrayList<>();
    public final ArrayList<Endpoint> endpointsY = new ArrayList<>();
    public SAPCell(int simID, int x, int y) {
        this.simID = simID;
        this.x = x;
        this.y = y;
        Simulation.get(simID).sapCells.add(this);
    }
    public void addBox(int parentID) {
        AABBox aabb = new AABBox(parentID, aabbs.size());
        aabbs.add(aabb);
        endpointsX.add(aabb.minX);
        endpointsX.add(aabb.maxX);
        endpointsY.add(aabb.minY);
        endpointsY.add(aabb.maxY);
    }
    public void updateSort() {
        for (AABBox aabb : aabbs) {
            aabb.updateBox();
        }
        insertionSort(endpointsX);
        sweep();
    }
    public void sweep() {
        activelyChecked.clear();
        pairs.clear();
        for (int i = 0; i < endpointsX.size(); i = i + 1) {
            Endpoint endpoint = endpointsX.get(i);
            if (endpoint.isMin) {
                for (int j = 0; j < activelyChecked.size(); j = j + 1) {
                    double y1min = aabbs.get(endpoint.boxIndex).minY.value;
                    double y1max = aabbs.get(endpoint.boxIndex).maxY.value;
                    double y2min = aabbs.get(activelyChecked.get(j)).minY.value;
                    double y2max = aabbs.get(activelyChecked.get(j)).maxY.value;
                    if (y1min <= y2max && y1max >= y2min) {
                        if (checkIfSameSoftbody(endpoint.boxIndex, activelyChecked.get(j))) continue;

                        pairs.add(endpoint.boxIndex);
                        pairs.add(activelyChecked.get(j));
                    }
                }
                activelyChecked.add(endpoint.boxIndex);
            }
            else {
                activelyChecked.remove((Integer) endpoint.boxIndex);
            }
        }
        activelyChecked.clear();
    }

     private boolean checkIfSameSoftbody(int boxIndex1, int boxIndex2) {
        int parentID1 = aabbs.get(boxIndex1).parentID;
        int parentID2 = aabbs.get(boxIndex2).parentID;
        if (parentID1 <= -2 && parentID2 <= -2) return(parentID1 == parentID2);
        else if (parentID1 <= -2) return(-parentID1 - 2 == Rigidbody.get(parentID2).parentSoftbody);
        else if (parentID2 <= -2) return(-parentID2 - 2 == Rigidbody.get(parentID1).parentSoftbody);
        return(false);
     }
    private static void insertionSort(ArrayList<Endpoint> endpointsList) {
        for (int i = 1; i < endpointsList.size(); i = i + 1) {
            if (endpointsList.get(i).value < endpointsList.get(i - 1).value || (endpointsList.get(i).value == endpointsList.get(i - 1).value && endpointsList.get(i).isMin)) { //here is the compare function
                int insertionIndex = 0;
                Endpoint insertionValue = endpointsList.get(i);
                for (int j = i - 1; j >= 0; j = j - 1) {
                    if (!(endpointsList.get(i).value < endpointsList.get(j).value || (endpointsList.get(i).value == endpointsList.get(j).value && endpointsList.get(i).isMin))) { //here as well, but inverted
                        insertionIndex = j + 1;
                        break;
                    }
                }
                for (int j = i; j > insertionIndex; j = j - 1) {
                    endpointsList.set(j, endpointsList.get(j - 1));
                }
                endpointsList.set(insertionIndex, insertionValue);
            }
        }
    }

     private static void quickSort(ArrayList<Endpoint> endpointsList) {
         qsort(endpointsList, 0, endpointsList.size() - 1);
     }

     private static void qsort(ArrayList<Endpoint> endpointsList, int start, int end) {
         if (end <= start) return;
         int pivot = end;
         int leftRightmostIndex = start;
         Endpoint temp = null;
         for (int j = start; j < end; j = j + 1) {
             if (endpointsList.get(j).value < endpointsList.get(pivot).value) { //here is where the compare function is done
                 temp = endpointsList.get(j);
                 endpointsList.set(j, endpointsList.get(leftRightmostIndex));
                 endpointsList.set(leftRightmostIndex, temp);
                 leftRightmostIndex += 1;
             }
         }
         temp = endpointsList.get(leftRightmostIndex);
         endpointsList.set(leftRightmostIndex, endpointsList.get(pivot));
         endpointsList.set(pivot, temp);
         qsort(endpointsList, start, leftRightmostIndex - 1);
         qsort(endpointsList, leftRightmostIndex + 1, end);
     }
}
