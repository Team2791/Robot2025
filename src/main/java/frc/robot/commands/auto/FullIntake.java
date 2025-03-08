package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TakeIn;
import frc.robot.commands.intake.ToDispenser;
import frc.robot.commands.lift.DispenseIn;
import frc.robot.commands.lift.Elevate;
import frc.robot.commands.lift.SlowBack;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.event.Emitter;
import frc.robot.event.Key;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;


public class FullIntake extends SequentialCommandGroup {
    private int stage = 0;
    private static Key<Double, Intake.IntakeRange> key;

    /**
     * Full intake command.
     * First, run the elevator to L0 and take coral into the intake simultaneously.
     * Then, move the coral from the intake to the dispenser by simultaneously running both motors.
     * Both the elevator and intake are used as command requirements.
     *
     * @param dispenser the dispenser subsystem
     * @param elevator  the elevator subsystem
     * @param intake    the intake subsystem
     */
    public FullIntake(Dispenser dispenser, Elevator elevator, Intake intake) {
        addCommands(
            new FunctionWrapper(() -> stage = 0),
            new Elevate(elevator, 0),
            new FunctionWrapper(() -> stage = 1),
            new ParallelCommandGroup(
                new SequentialCommandGroup(new TakeIn(intake), new ToDispenser(intake, elevator)),
                new SequentialCommandGroup(new DispenseIn(dispenser, elevator), new SlowBack(dispenser))
            )
        );
    }

    /**
     * Register the full intake command to run when the robot is near the coral dispenser.
     *
     * @param dispenser the dispenser subsystem
     * @param elevator  the elevator subsystem
     * @param intake    the intake subsystem
     */
    public static void registerNearby(Dispenser dispenser, Elevator elevator, Intake intake) {
        FullIntake instance = new FullIntake(dispenser, elevator, intake);
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
        if (key == null) return;
        Emitter.off(key);
    }
}
