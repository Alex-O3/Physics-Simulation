package PhysicsSim;

/**
 * Script is an interface to add functions that will run before or after each simulation step.
 */
public interface Script {
    /**
     * Runs before each simulation step.
     */
    void runBefore();

    /**
     * Runs after each simulation step.
     */
    void runAfter();
}
