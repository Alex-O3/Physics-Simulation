package PhysicsSim;

enum JointType {
    //Softbody is used for within a softbody and is, for nearly all intents and purposes outside of construction, the same as a spring joint
    Softbody,
    //Spring joints have spring qualities and distance constraint qualities.
    Spring,
    //Pin joints bind a location on two rigidbodies together, but allow free rotation about that point.
    Pin,
    //Weld joints apply the same effects as pin joints, but enforce an equal application of movement on all rigidbodies connected via weld joints, which form a new compound body.
    Weld,
    //Revolute joints are similar to pin joints, except for the fact that they have the ability to limit the rotation about the connection point via the angle made with the center of mass.
    Revolute
}
