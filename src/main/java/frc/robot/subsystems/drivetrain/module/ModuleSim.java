package frc.robot.subsystems.drivetrain.module;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.util.PIDFController;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

public class ModuleSim extends ModuleIO {
    final SwerveModuleSimulation moduleSim;
    final SimulatedMotorController.GenericMotorController driveSim;
    final SimulatedMotorController.GenericMotorController turnSim;

    final PIDFController driveController;
    final PIDFController turnController;

    public ModuleSim(SwerveModuleSimulation moduleSim) {
        driveController = new PIDFController(
            ControlConstants.DriveMotor.kP,
            ControlConstants.DriveMotor.kI,
            ControlConstants.DriveMotor.kD,
            ControlConstants.DriveMotor.kF
        );

        turnController = new PIDFController(
            ControlConstants.TurnMotor.kP,
            ControlConstants.TurnMotor.kI,
            ControlConstants.TurnMotor.kD,
            ControlConstants.TurnMotor.kF
        );

        this.moduleSim = moduleSim;

        driveSim = moduleSim
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(MotorConstants.Neo.kCurrentLimit));

        turnSim = moduleSim
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(MotorConstants.Neo550.kCurrentLimit));

        driveController.setOutputRange(ControlConstants.DriveMotor.kMin, ControlConstants.DriveMotor.kMax);
        turnController.setOutputRange(ControlConstants.TurnMotor.kMinOut, ControlConstants.TurnMotor.kMaxOut);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void update() {
        final double drivePow = driveController.calculate(this.data.driveVelocity.in(RadiansPerSecond));
        final double turnPow = turnController.calculate(this.data.turnPosition.in(Radians));

        // https://www.chiefdelphi.com/t/sparkmax-set-vs-setvoltage/415059/2
        // correct for differing voltages cuz battery won't always be 12V
        final double driveVolts = drivePow * RobotController.getBatteryVoltage();
        final double turnVolts = turnPow * RobotController.getBatteryVoltage();

        // set the input voltage to simulate
        driveSim.requestVoltage(Volts.of(driveVolts));
        turnSim.requestVoltage(Volts.of(turnVolts));

        final Angle drivePos = moduleSim.getDriveWheelFinalPosition();
        final Angle turnPos = moduleSim.getSteerAbsoluteAngle();
        final AngularVelocity driveVel = moduleSim.getDriveWheelFinalSpeed();
        final AngularVelocity turnVel = moduleSim.getSteerAbsoluteEncoderSpeed();

        // actually update the inputs
        data.driveConnected = true;
        data.drivePosition = drivePos;
        data.driveVelocity = driveVel;
        data.driveVoltage = Volts.of(driveVolts);
        data.driveCurrent = moduleSim.getDriveMotorSupplyCurrent();

        data.turnConnected = true;
        data.turnPosition = turnPos;
        data.turnVelocity = turnVel;
        data.turnVoltage = Volts.of(turnVolts);
        data.turnCurrent = moduleSim.getSteerMotorSupplyCurrent();
    }

    public void setDesiredState(SwerveModuleState desired) {
        desired.optimize(new Rotation2d(moduleSim.getSteerAbsoluteAngle()));

        driveController.setSetpoint(desired.speedMetersPerSecond / ModuleConstants.Wheel.kRadius);
        turnController.setSetpoint(desired.angle.getRadians());

        this.data.desired = desired;
        this.data.commanded = RadiansPerSecond.of(desired.speedMetersPerSecond / ModuleConstants.Wheel.kRadius);
    }

    @Override
    public void setIdleMode(SparkBaseConfig.IdleMode mode) { }
}