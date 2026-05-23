package PhysicsSim;

/**
 * This class
 */
public interface EventListener {
    /**
     * This method is only called by physicsObjects, not hitboxes. <br>
     * Called prior to collision resolution in a simulation step, which there may be multiple of per frame. Collisions are only registered here
     * between rigidbodies and joints, the latter of which are logged as two separate collisions with each parent of the solid joint.
     * Order of calls is not in any particular or meaningful manner.
     * @param e record containing parent (holds listener) and collided rigidbody physicsObject alongside normal, point of contact, and boolean isJoint information. The other object may be null if it is the bounds.
     */
    void collidedPre(CollisionEvent e);
    /**
     * This method is only called by physicsObjects, not hitboxes. <br>
     * Called after the collision resolution in a simulation step, which there may be multiple of per frame. Collisions are only registered here
     * between rigidbodies and joints, the latter of which are logged as two separate collisions with each parent of the solid joint.
     * Order of calls is not in any particular or meaningful manner.
     * @param e record containing parent (holds listener) and collided rigidbody physicsObject alongside normal, point of contact, and boolean isJoint information. The other object may be null if it is the bounds.
     */
    void collidedPost(CollisionEvent e);

    /**
     * This method is only called by hitboxes, not physicsObjects. <br>
     * Always called prior to the resolution process in a simulation step, which there may be multiple of per frame. The parent is the hitbox with the attached listener.
     * Order of calls is not in any particular or meaningful manner.
     * @param e record containing parent and intersecting hitbox. The other hitbox may be null if it is the bounds.
     */
    void intersected(HitboxIntersectionEvent e);
    /**
     * This method is only called by hitboxes, not physicsObjects. <br>
     * Always called prior to the resolution process in a simulation step, which there may be multiple of per frame. The hitbox always has the attached listener.
     * Collisions are only registered here for rigidbody physicsObjects and joints, the latter of which are logged as two separate intersections with each parent of the solid joint.
     * Softbody physicsObjects collisions may be handled with the getParentSoftbody() method.
     * Order of calls is not in any particular or meaningful manner.
     * @param e record containing intersecting rigidbody physicsObject and hitbox.
     */
    void intersected(PhysicsObjectIntersectionEvent e);
}
