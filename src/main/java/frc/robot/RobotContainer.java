package frc.robot;

import choreo.auto.AutoChooser;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoManager;
import frc.robot.commands.align.ReefAlign;
import frc.robot.commands.dispenser.DispenseOut;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.elevator.ManualElevate;
import frc.robot.commands.intake.Dislodge;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.manipulator.FullManipulate;
import frc.robot.commands.manipulator.RunManipulator;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.algae.ManipulatorReplay;
import frc.robot.subsystems.algae.ManipulatorSim;
import frc.robot.subsystems.algae.ManipulatorSpark;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.dispenser.DispenserReplay;
import frc.robot.subsystems.dispenser.DispenserSim;
import frc.robot.subsystems.dispenser.DispenserSpark;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroReplay;
import frc.robot.subsystems.drivetrain.gyro.NavX;
import frc.robot.subsystems.drivetrain.module.ModuleReplay;
import frc.robot.subsystems.drivetrain.module.ModuleSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorReplay;
import frc.robot.subsystems.elevator.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeReplay;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intake.IntakeSpark;
import frc.robot.subsystems.photon.Camera;
import frc.robot.subsystems.photon.CameraReplay;
import frc.robot.subsystems.photon.CameraSim;
import frc.robot.subsystems.photon.Photon;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.Alerter;
import frc.robot.util.WorldSimulator;

public class RobotContainer {
    // controllers
    final CommandXboxController driverctl;
    final CommandXboxController operctl;

    // subsystems
    final Drivetrain drivetrain = new Drivetrain(
        AdvantageUtil.matchReal(NavX::new, () -> WorldSimulator.getInstance().makeGyro(), GyroReplay::new),
        AdvantageUtil.matchReal(
            ModuleSpark::new,
            (id) -> WorldSimulator.getInstance().makeModule(id),
            ModuleReplay::new
        )
    );
    final Intake intake = new Intake(
        AdvantageUtil.matchReal(IntakeSpark::new, IntakeSim::new, IntakeReplay::new)
    );
    final Dispenser dispenser = new Dispenser(
        AdvantageUtil.matchReal(DispenserSpark::new, DispenserSim::new, DispenserReplay::new)
    );
    final Elevator elevator = new Elevator(
        AdvantageUtil.matchReal(ElevatorSpark::new, ElevatorSim::new, ElevatorReplay::new)
    );
    final Photon photon = new Photon(
        drivetrain,
        AdvantageUtil.matchReal(Camera::new, CameraSim::new, CameraReplay::new)
    );
    final AlgaeManipulator manipulator = new AlgaeManipulator(
        AdvantageUtil.matchReal(ManipulatorSpark::new, ManipulatorSim::new, ManipulatorReplay::new)
    );

    // autos
    final AutoManager autoManager = new AutoManager(drivetrain);
    final AutoChooser autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("LeftAlign", new ReefAlign(drivetrain, -1));
        NamedCommands.registerCommand("Elevate0", new Elevate(elevator, 0));
        NamedCommands.registerCommand("Elevate4", new Elevate(elevator, 4));
        NamedCommands.registerCommand("DispenseOut", new DispenseOut(dispenser, elevator));

        this.driverctl = new CommandXboxController(IOConstants.Controller.kDriver);
        this.operctl = new CommandXboxController(IOConstants.Controller.kOperator);
        this.autoChooser = new AutoChooser();

        this.autoChooser.addRoutine("Default Routine", () -> this.autoManager.routine(dispenser, elevator, intake));

        configureBindings();

        SmartDashboard.putData("Chooser", autoChooser);
        Alerter.getInstance().provideControllers(driverctl, operctl);
    }

    private void configureBindings() {
        // automatically start the intake if near the coral station

        // disable for now.
        FullIntake.registerNearby(dispenser, elevator, intake);
        Elevate.registerRetract(elevator);

        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(driverctl), drivetrain));
        driverctl.start().onTrue(new FunctionWrapper(drivetrain::resetGyro));

        driverctl.x().onTrue(new Elevate(elevator, 3));
        driverctl.y().onTrue(new Elevate(elevator, 4));
        driverctl.a().onTrue(new Elevate(elevator, 1));
        driverctl.b().onTrue(new Elevate(elevator, 2));

        driverctl.rightBumper().toggleOnTrue(new ReefAlign(drivetrain, 1));
        driverctl.leftBumper().toggleOnTrue(new ReefAlign(drivetrain, -1));

        driverctl.rightTrigger().onTrue(new DispenseOut(dispenser, elevator));
        driverctl.leftTrigger().toggleOnTrue(new SequentialCommandGroup(
            new Dislodge(intake, dispenser),
            new FullIntake(dispenser, elevator, intake)
        ));

        driverctl.rightStick().onTrue(new Elevate(elevator, 0));
        driverctl.leftStick().toggleOnTrue(new FullManipulate(manipulator, drivetrain, elevator));

        operctl.axisLessThan(1, -0.1)
            .whileTrue(
                new SequentialCommandGroup(
                    new FunctionWrapper(Elevate::disableRetract),
                    new ManualElevate(elevator, true)
                )
            );

        operctl.axisGreaterThan(1, 0.1)
            .whileTrue(
                new SequentialCommandGroup(
                    new FunctionWrapper(Elevate::disableRetract),
                    new ManualElevate(elevator, false)
                )
            );

        operctl.a().onTrue(new FunctionWrapper(FullIntake::disableNearby));
        operctl.b().onTrue(new FunctionWrapper(Elevate::disableRetract));
        operctl.x().whileTrue(new Dislodge(intake, dispenser));
        operctl.y().toggleOnTrue(new RunManipulator(manipulator));
    }

    public Command getAutonomousCommand() { return autoChooser.selectedCommand(); }
}
