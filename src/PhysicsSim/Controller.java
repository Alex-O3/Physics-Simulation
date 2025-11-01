package PhysicsSim;

import java.util.ArrayList;
import java.util.prefs.PreferenceChangeListener;

class Controller {
    private final char key;
    private final char excludeIfKey;
    private final double initialVelocityMag;
    private final double releaseVelocity;
    private final double sustainedAccelerationMag;
    private final double maxSpeed;
    private final double[] direction = new double[2];
    private final int parentID;
    private final int type;
    //type is 0 for rigidbody, 1 for softbody

    boolean lastOnGround = false;
    int countUntilOffGround = 0;
    double[] lastGroundVelocity = new double[]{0.0, 0.0};
    //at 5 steps per frame, this is 2 frames
    public double stepsUntilNotTouchingGround = 10;
    public double velocityDecrementWhenInAir = 200.0;

    public Controller(char key, double initialVelocity, double releaseVelocity, double sustainedForce, double maxSpeed, double[] direction, Rigidbody rigidbody, char excludeIfKey) {
        this.key = key;
        this.excludeIfKey = excludeIfKey;
        this.initialVelocityMag = initialVelocity;
        this.releaseVelocity = releaseVelocity;
        this.sustainedAccelerationMag = sustainedForce;
        this.maxSpeed = maxSpeed;
        double magnitude = Math.sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
        this.direction[0] = direction[0] / magnitude;
        this.direction[1] = direction[1] / magnitude;
        type = 0;
        parentID = rigidbody.ID;
        rigidbody.controllers.add(this);
    }

    public void respondToKey(double dt, ArrayList<Character> keysCache, boolean firstPress,
                             boolean release, boolean onGround, boolean touchingObject, double[] groundVelocity) {
        if (onGround) {
            lastOnGround = true;
            countUntilOffGround = 0;
            lastGroundVelocity = groundVelocity;
        }
        else if (countUntilOffGround >= stepsUntilNotTouchingGround) {
            lastOnGround = false;
            countUntilOffGround = 0;
            double magnitude = Math.sqrt(lastGroundVelocity[0] * lastGroundVelocity[0] + lastGroundVelocity[1] * lastGroundVelocity[1]);
            if (magnitude > 0.0) {
                double multiplier = Math.max(0.0, 1.0 - (velocityDecrementWhenInAir / magnitude) * dt);
                lastGroundVelocity[0] *= multiplier;
                lastGroundVelocity[1] *= multiplier;
            }
        }
        else {
            countUntilOffGround += 1;
        }
        if (keysCache.contains((Character)excludeIfKey) && keysCache.contains((Character)key) && keysCache.indexOf((Character)key) == keysCache.size() - 1) {
            keysCache.remove((Character)excludeIfKey);
        }
        if (keysCache.contains((Character)key) && !keysCache.contains((Character)excludeIfKey)) switch(type) {
            case(0): {
                double vX = Rigidbody.get(parentID).getVX() - lastGroundVelocity[0];
                double vY = Rigidbody.get(parentID).getVY() - lastGroundVelocity[1];
                double dotV = 0.0;
                if (release && releaseVelocity > 0.0 && Simulation.get(Rigidbody.get(parentID).simID).display.keyReleasedForTheFirstTime == key && keysCache.size() == 1) {
                    dotV = releaseVelocity;
                    double tdotV = vX * -direction[1] + vY * direction[0];
                    double newvX = dotV * direction[0] - tdotV * direction[1];
                    double newvY = dotV * direction[1] + tdotV * direction[0];
                    Rigidbody.get(parentID).setVX(newvX + lastGroundVelocity[0]);
                    Rigidbody.get(parentID).setVY(newvY + lastGroundVelocity[1]);
                }
                dotV = vX * direction[0] + vY * direction[1];
                if (firstPress && key == keysCache.getLast() && lastOnGround && initialVelocityMag > 0.0 && !release) {
                    double newdotV = initialVelocityMag;
                    double tdotV = vX * -direction[1] + vY * direction[0];
                    double newvX = newdotV * direction[0] - tdotV * direction[1];
                    double newvY = newdotV * direction[1] + tdotV * direction[0];
                    Rigidbody.get(parentID).setVX(newvX + lastGroundVelocity[0]);
                    Rigidbody.get(parentID).setVY(newvY + lastGroundVelocity[1]);
                }
                if (sustainedAccelerationMag > 0.0 && (!touchingObject || onGround)) {
                    double aX = Rigidbody.get(parentID).getAX();
                    double aY = Rigidbody.get(parentID).getAY();
                    double dotA = sustainedAccelerationMag;
                    if (maxSpeed > 0.0) dotA *= (1.0 - (dotV / maxSpeed));
                    double tdotA = aX * -direction[1] + aY * direction[0];
                    double newaX = dotA * direction[0] - tdotA * direction[1];
                    double newaY = dotA * direction[1] + tdotA * direction[0];
                    Rigidbody.get(parentID).setAX(newaX);
                    Rigidbody.get(parentID).setAY(newaY);
                }
                break;
            }
            default: {
                System.out.println("Controller improperly initialized.");
            }
        }
    }


}
