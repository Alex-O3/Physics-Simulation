package PhysicsSim;

/**
 *
 * @param parent
 * @param collision
 * @param normal
 * @param pointOfContact
 * @param isJointCollision
 */
public record CollisionEvent(PhysicsObject parent, PhysicsObject collision, double[] normal, double[] pointOfContact, boolean isJointCollision) {
}
