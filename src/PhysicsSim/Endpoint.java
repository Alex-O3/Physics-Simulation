package PhysicsSim;

 public class Endpoint {
    public double value;
    public final boolean isMin;
    public final int boxIndex;
    public Endpoint(int boxIndex, boolean isMin) {
        this.boxIndex = boxIndex;
        this.isMin = isMin;
    }
}
