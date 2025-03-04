package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.ElevatorConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.lift.Lift;

public class Elevate extends FunctionWrapper {
    private final boolean auto;
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
     * @param lift   The lift subsystem
     * @param height The height to elevate to
     */
    public Elevate(Lift lift, int height, boolean blocking) {
        this(lift, height, blocking, false);
    }

    /**
     * Elevate to a certain height. Decides whether to block or not. If this command is managed by
     * registerRetract, it will allow, rather than block, incoming commands.
     *
     * @param lift     The lift subsystem
     * @param height   The height to elevate to
     * @param blocking Whether to block until the lift is at the desired height
     * @param auto     Whether this command is managed by registerRetract
     */
    private Elevate(Lift lift, int height, boolean blocking, boolean auto) {
        super(() -> lift.elevate(height), () -> lift.atLevel(height) || !blocking, lift);
        this.auto = auto;
    }

    //    @Override
    //    public InterruptionBehavior getInterruptionBehavior() {
    //        if (auto) return InterruptionBehavior.kCancelSelf;
    //        else return InterruptionBehavior.kCancelIncoming;
    //    }

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
        Emitter.off(retract);
    }
}
