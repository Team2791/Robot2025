package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TakeIn;
import frc.robot.commands.intake.ToDispenser;
import frc.robot.commands.lift.DispenseIn;
import frc.robot.commands.lift.Elevate;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.event.Emitter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;


public class FullIntake extends SequentialCommandGroup {
    private int stage = 0;
    private static Emitter.Key<Double, Intake.IntakeRange> key;

    /**
     * Full intake command.
     * First, run the elevator to L0 and take coral into the intake simultaneously.
     * Then, move the coral from the intake to the dispenser by simultaneously running both motors.
     * Both the lift and intake are used as command requirements.
     *
     * @param intake The intake subsystem
     * @param lift   The lift subsystem
     */
    public FullIntake(Intake intake, Lift lift) {
        addCommands(
            // reset stage
            new FunctionWrapper(() -> stage = 0),
            new ParallelCommandGroup(new Elevate(lift, 0, false), new TakeIn(intake)),
            //            new FunctionWrapper(() -> stage++), // increment stage, so we know whether we should cancel when far
            new ParallelRaceGroup(new DispenseIn(lift), new ToDispenser(intake, lift))
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming; // FullIntake will never stop unless explicit
    }

    /**
     * Register the full intake command to run when the robot is near the coral dispenser.
     *
     * @param intake The intake subsystem
     * @param lift   The lift subsystem
     */
    public static void registerNearby(Intake intake, Lift lift) {
        FullIntake instance = new FullIntake(intake, lift);
        CommandScheduler scheduler = CommandScheduler.getInstance();

        key = Emitter.on(
            new Intake.IntakeRange(),
            distance -> {
                boolean nearby = distance <= IntakeConstants.Range.kRunIntake;
                boolean scheduled = scheduler.isScheduled(instance);

                if (nearby && !scheduled) scheduler.schedule(instance);
                else if (!nearby && scheduled && instance.stage == 0) scheduler.cancel(instance);
            }
        );
    }

    /**
     * Unregister the full intake command from running when the robot is near the coral dispenser.
     */
    public static void disableNearby() {
        Emitter.off(key);
    }
}
