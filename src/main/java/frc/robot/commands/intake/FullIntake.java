package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.dispenser.DispenseIn;
import frc.robot.commands.dispenser.SlowBack;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.event.EventRegistry;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Alerter;


public class FullIntake extends SequentialCommandGroup {
    private int stage = 0;
    private static boolean useNearby = true;

    /**
     * Full intake command.
     * First, run the elevator to L0.
     * Then, run the intake until the beam has broken and move it to the dispenser.
     * At the same time as above, run the dispenser to accept the coral, and pull coral back slowly
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
                .handleInterrupt(() -> new SequentialCommandGroup(
                    new Dislodge(intake, dispenser),
                    new FullIntake(dispenser, elevator, intake)
                ).schedule())
                .withTimeout(5.0),
            new FunctionWrapper(Alerter.getInstance()::rumble)
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

        EventRegistry.stationRange.register(distance -> {
            boolean nearby = distance <= IntakeConstants.Range.kRunIntake;
            boolean scheduled = scheduler.isScheduled(instance);
            boolean auto = DriverStation.isAutonomous();

            if (auto || !useNearby) return;
            if (nearby && !scheduled) scheduler.schedule(instance);
            else if (!nearby && scheduled && instance.stage == 0) scheduler.cancel(instance);
        });
    }

    /**
     * Unregister the full intake command from running when the robot is near the coral dispenser.
     */
    public static void disableNearby() {
        useNearby = false;
    }
}
