package PhysicsSim;

public class Material {
    protected final String name;
    protected final double density;
    protected final double restitution;
    protected final double dynamic_friction;
    protected final double static_friction;
    protected Material(String name, double density, double restitution, double dynamic_friction, double static_friction) {
        this.name = name;
        this.density = density;
        this.restitution = restitution;
        this.dynamic_friction = dynamic_friction;
        this.static_friction = static_friction;
    }
    protected void print() {
        System.out.println("Material " + name + ": " + density + " density, " + restitution + " restitution, " + dynamic_friction + " dynamic friction, " + static_friction + " static friction.");
    }
}
