package frc.robot.sim;

/**
 * Minimal stub of PhysicsSim used by example Robot.java so the project compiles.
 * This does not implement full physics; it's only here to satisfy references.
 */
public class PhysicsSim {
    private static final PhysicsSim INSTANCE = new PhysicsSim();

    private PhysicsSim() {}

    public static PhysicsSim getInstance() {
        return INSTANCE;
    }

    /**
     * Accept a generic object for the talon to avoid compile-time dependency on
     * CTRE classes in the stub. The real project will use the concrete type when
     * Phoenix is available on the classpath.
     */
    public void addTalonFX(Object talon, double scale, double gearing) {
        // no-op stub for compilation; a real simulation would track motor state
    }

    public void run() {
        // no-op stub for compilation
    }
}
