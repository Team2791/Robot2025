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

    SwerveModuleState desiredState;

    /**
     * Constructs a new SwerveModule
     *
     * @param info data about the module, from ModuleConstants
     */
    public ModuleSpark(ModuleConstants.ModuleInfo info) {
        super(info);

        // initialize motors, encoders, etc.
        driveMotor = new SparkMax(info.driveId(), MotorType.kBrushless);
        turnMotor = new SparkMax(info.turnId(), MotorType.kBrushless);

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
        Alerter.getInstance().registerSpark("Module%dDrive".formatted(info.moduleId()), driveMotor);
        Alerter.getInstance().registerSpark("Module%dTurn".formatted(info.moduleId()), turnMotor);
    }

    @Override
    public void update() {
        this.data.driveConnected = driveMotor.getLastError() == REVLibError.kOk;
        this.data.drivePosition = Meters.of(driveEncoder.getPosition() * ModuleConstants.Wheel.kRadius);
        this.data.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity() * ModuleConstants.Wheel.kRadius);
        this.data.driveVoltage = Volts.of(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput());
        this.data.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

        this.data.turnConnected = turnMotor.getLastError() == REVLibError.kOk;
        this.data.turnPosition = Radians.of(turnEncoder.getPosition() - info.angularOffset());
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
        double turnSetpoint = SwerveUtil.normalizeAngle(turnPosition + info.angularOffset());

        turnController.setReference(turnSetpoint, ControlType.kPosition);
        driveController.setReference(driveVelocity / ModuleConstants.Wheel.kRadius, ControlType.kVelocity);
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
        turnController.setReference(info.angularOffset(), ControlType.kPosition);
    }
}

