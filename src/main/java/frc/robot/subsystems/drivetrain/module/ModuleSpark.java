package frc.robot.subsystems.drivetrain.module;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.Alerter;

import static edu.wpi.first.units.Units.*;

public class ModuleSpark extends ModuleIO {
    final SparkMax driveMotor;
    final SparkMax turnMotor;

    final RelativeEncoder driveEncoder;
    final SparkAbsoluteEncoder turnEncoder;

    final SparkClosedLoopController driveController;
    final SparkClosedLoopController turnController;

    final double angularOffset;

    SwerveModuleState desiredState;

    /**
     * Constructs a new SwerveModule
     *
     * @param id the module's id. [0, 3] corresponds to [fl, fr, bl, br]
     */
    public ModuleSpark(int id) {
        int driveId = switch (id) {
            case 0 -> IOConstants.Drivetrain.Drive.kFrontLeft;
            case 1 -> IOConstants.Drivetrain.Drive.kFrontRight;
            case 2 -> IOConstants.Drivetrain.Drive.kRearLeft;
            case 3 -> IOConstants.Drivetrain.Drive.kRearRight;
            default -> throw new IllegalArgumentException("Invalid module id: " + id);
        };

        int turnId = switch (id) {
            case 0 -> IOConstants.Drivetrain.Turn.kFrontLeft;
            case 1 -> IOConstants.Drivetrain.Turn.kFrontRight;
            case 2 -> IOConstants.Drivetrain.Turn.kRearLeft;
            case 3 -> IOConstants.Drivetrain.Turn.kRearRight;
            default -> throw new IllegalArgumentException("Invalid module id: " + id);
        };

        angularOffset = switch (id) {
            case 0 -> ModuleConstants.AngularOffsets.kFrontLeft;
            case 1 -> ModuleConstants.AngularOffsets.kFrontRight;
            case 2 -> ModuleConstants.AngularOffsets.kRearLeft;
            case 3 -> ModuleConstants.AngularOffsets.kRearRight;
            default -> throw new IllegalArgumentException("Invalid module id: " + id);
        };

        // initialize motors, encoders, etc.
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        driveController = driveMotor.getClosedLoopController();
        turnController = turnMotor.getClosedLoopController();

        desiredState = new SwerveModuleState();

        // apply and burn configs
        driveMotor.configure(
            SparkConfigConstants.Drivetrain.kDrive,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );
        turnMotor.configure(
            SparkConfigConstants.Drivetrain.kTurn,
            SparkConfigConstants.kResetMode,
            SparkConfigConstants.kPersistMode
        );

        // register with notifier
        Alerter.getInstance().registerSpark("Module" + driveId / 10 + "Drive", driveMotor);
        Alerter.getInstance().registerSpark("Module" + driveId / 10 + "Turn", turnMotor);
    }

    @Override
    public void update() {
        this.data.driveConnected = driveMotor.getLastError() == REVLibError.kOk;
        this.data.drivePosition = Radians.of(driveEncoder.getPosition());
        this.data.driveVelocity = RadiansPerSecond.of(driveEncoder.getVelocity());
        this.data.driveVoltage = Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
        this.data.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

        this.data.turnConnected = turnMotor.getLastError() == REVLibError.kOk;
        this.data.turnPosition = Radians.of(turnEncoder.getPosition() - angularOffset);
        this.data.turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());
        this.data.turnVoltage = Volts.of(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput());
        this.data.turnCurrent = Amps.of(turnMotor.getOutputCurrent());
    }

    public void setDesiredState(SwerveModuleState desired) {
        SwerveModuleState corrected = new SwerveModuleState(
            desired.speedMetersPerSecond,
            desired.angle.plus(new Rotation2d(angularOffset))
        );

        corrected.optimize(new Rotation2d(turnEncoder.getPosition()));

        double angular = corrected.speedMetersPerSecond / ModuleConstants.Wheel.kRadius;
        driveController.setReference(angular, ControlType.kVelocity);
        turnController.setReference(corrected.angle.getRadians(), ControlType.kPosition);

        this.data.desired = corrected;
        this.data.commanded = RadiansPerSecond.of(angular);
    }

    @Override
    public void setIdleMode(SparkBaseConfig.IdleMode mode) {
        SparkMaxConfig config = SparkConfigConstants.Drivetrain.kDrive;
        config.idleMode(mode);

        this.driveMotor.configure(config, SparkConfigConstants.kResetMode, SparkConfigConstants.kPersistMode);
    }
}

