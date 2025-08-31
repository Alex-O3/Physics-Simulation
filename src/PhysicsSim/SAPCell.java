package PhysicsSim;

import java.util.ArrayList;

 class SAPCell {
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
        insertionSort(endpointsY);
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

    private void insertionSort(ArrayList<Endpoint> endpointsList) {
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
}
