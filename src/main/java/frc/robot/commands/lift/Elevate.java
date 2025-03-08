package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.ElevatorConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.lift.Lift;

public class Elevate extends FunctionWrapper {
    private static Emitter.Key<Double, Lift.ReefRange> retract;

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
     * @param lift     The lift subsystem
     * @param height   The height to elevate to
     * @param blocking Whether to block until the lift is at the desired height
     */
    public Elevate(Lift lift, int height, boolean blocking) {
        super(() -> lift.elevate(height), () -> lift.atLevel(height) || !blocking, lift);
    }

    public static void registerRetract(Lift lift) {
        final Elevate instance = new Elevate(lift, 0, true);
        final CommandScheduler scheduler = CommandScheduler.getInstance();

        retract = Emitter.on(
            new Lift.ReefRange(),
            distance -> {
                boolean outside = distance >= ElevatorConstants.Range.kRetract;
                boolean scheduled = scheduler.isScheduled(instance);
                boolean zeroed = lift.atLevel(0);

                if (outside && !scheduled && !zeroed) scheduler.schedule(instance);
            }
        );
    }

    public static void disableRetract() {
        if (retract == null) return;
        Emitter.off(retract);
    }
}
