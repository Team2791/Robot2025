package frc.robot.commands.lift;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.subsystems.lift.Lift;

public class Elevate extends FunctionWrapper {
    /**
     * Elevate to a certain height. This command will block until the lift is at the desired height.
     *
     * @param lift   The lift subsystem
     * @param height The height to elevate to
     */
    public Elevate(Lift lift, int height) {
        this(lift, height, true);
    }

    /**
     * Elevate to a certain height. Decides whether to block or not.
     *
     * @param lift   The lift subsystem
     * @param height The height to elevate to
     */
    public Elevate(Lift lift, int height, boolean blocking) {
        super(() -> lift.elevate(height), () -> lift.atLevel(height) || !blocking, lift);
    }
}
