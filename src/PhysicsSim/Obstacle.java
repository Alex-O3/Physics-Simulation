package PhysicsSim;
import java.awt.*;

class Obstacle extends Rigidbody {

    public Obstacle(double[] posX, double[] posY, double mass, Color color) {
        super(posX, posY, new double[]{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}, mass, color);
        super.setPosX(super.getPosX() + super.getCX());
        super.setPosY(super.getPosY() + super.getCY());
        super.setIsMovable(false);
    }
}
