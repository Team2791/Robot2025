package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoManager;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.util.Alerter;

import java.io.IOException;

public class RobotContainer {
    // controllers
    final CommandXboxController driverctl;
    final CommandXboxController operctl;

    // subsystems
    final Drivetrain drivetrain = new Drivetrain(new DrivetrainSim());

    // autos
    final AutoManager autoManager = new AutoManager(drivetrain);

    public RobotContainer() throws IOException {
        this.driverctl = new CommandXboxController(IOConstants.Controller.kDriver);
        this.operctl = new CommandXboxController(IOConstants.Controller.kOperator);

        configureBindings();

        Alerter.getInstance().provideControllers(driverctl, operctl);
        CameraServer.startAutomaticCapture();
        CameraServer.removeCamera("USB Camera 0"); // fix USB Camera 0 problem, but we still need to init CamServer
    }

    private void configureBindings() {
        Command joystickDrive = new RunCommand(() -> drivetrain.drive(driverctl), drivetrain);
        drivetrain.setDefaultCommand(joystickDrive);
        driverctl.start().onTrue(new FunctionWrapper(drivetrain::resetGyro).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return autoManager.routine().cmd();
    }
}
