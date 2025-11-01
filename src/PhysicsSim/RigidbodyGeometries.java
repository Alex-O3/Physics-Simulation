package PhysicsSim;

public enum RigidbodyGeometries {
    Polygon(0),
    Circle(1);

    private final int typeID;
    RigidbodyGeometries(int typeID) {
        this.typeID = typeID;
    }
}
