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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.SparkConfigConstants;
import frc.robot.util.Alerter;
import frc.robot.util.SwerveUtil;

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
        super(switch (id) {
            case 0 -> IOConstants.Drivetrain.Drive.kFrontLeft / 10;
            case 1 -> IOConstants.Drivetrain.Drive.kFrontRight / 10;
            case 2 -> IOConstants.Drivetrain.Drive.kRearLeft / 10;
            case 3 -> IOConstants.Drivetrain.Drive.kRearRight / 10;
            default -> throw new IllegalArgumentException("Invalid module id: " + id);
        });

        int driveId = moduleId * 10;
        int turnId = driveId + 5;

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
        Alerter.getInstance().registerSpark("Module" + moduleId + "Drive", driveMotor);
        Alerter.getInstance().registerSpark("Module" + moduleId + "Turn", turnMotor);
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

    /**
     * Set a desired module state
     *
     * @param driveVelocity the velocity in radians per second
     * @param turnPosition  the position in radians
     */
    @Override
    public void setStateSetpoint(double driveVelocity, double turnPosition) {
        double turnSetpoint = SwerveUtil.normalizeAngle(turnPosition + angularOffset);

        turnController.setReference(turnSetpoint, ControlType.kPosition);
        driveController.setReference(driveVelocity, ControlType.kVelocity);
    }

    @Override
    public void setIdleMode(SparkBaseConfig.IdleMode mode) {
        SparkMaxConfig config = SparkConfigConstants.Drivetrain.kDrive;
        config.idleMode(mode);

        this.driveMotor.configure(config, SparkConfigConstants.kResetMode, SparkConfigConstants.kPersistMode);
    }

    @Override
    public void driveOpenLoop(double output) {
        driveMotor.setVoltage(output);
    }

    @Override
    public void zeroTurn() {
        turnController.setReference(angularOffset, ControlType.kPosition);
    }
}

