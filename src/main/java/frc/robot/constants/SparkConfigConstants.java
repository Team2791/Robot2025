package frc.robot.constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigConstants {
    public static final class Drivetrain {
        public static final SparkMaxConfig kDrive;
        public static final SparkMaxConfig kTurn;

        static {
            kDrive = new SparkMaxConfig();
            kTurn = new SparkMaxConfig();

            // current limits
            kDrive.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);
            kTurn.smartCurrentLimit((int) MotorConstants.Neo550.kCurrentLimit);

            // position and velocity factors
            kDrive.encoder.positionConversionFactor(ModuleConstants.DriveEncoder.kPositionFactor);
            kDrive.encoder.velocityConversionFactor(ModuleConstants.DriveEncoder.kVelocityFactor);
            kTurn.absoluteEncoder.positionConversionFactor(ModuleConstants.TurnEncoder.kPositionFactor);
            kTurn.absoluteEncoder.velocityConversionFactor(ModuleConstants.TurnEncoder.kVelocityFactor);

            // voltage compensation
            kDrive.voltageCompensation(MotorConstants.kNominalVoltage);
            kTurn.voltageCompensation(MotorConstants.kNominalVoltage);

            // setup absolute encoder
            kTurn.absoluteEncoder.inverted(ModuleConstants.TurnEncoder.kInverted);
            kTurn.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

            // turn pid wrapping
            kTurn.closedLoop.positionWrappingEnabled(true);
            kTurn.closedLoop.positionWrappingMinInput(ControlConstants.TurnMotor.kMinInput);
            kTurn.closedLoop.positionWrappingMaxInput(ControlConstants.TurnMotor.kMaxInput);

            // pid constants
            kDrive.closedLoop.pidf(
                    ControlConstants.DriveMotor.kP,
                    ControlConstants.DriveMotor.kI,
                    ControlConstants.DriveMotor.kD,
                    ControlConstants.DriveMotor.kF);
            kDrive.closedLoop.outputRange(ControlConstants.DriveMotor.kMin, ControlConstants.DriveMotor.kMax);

            kTurn.closedLoop.pidf(
                    ControlConstants.TurnMotor.kP,
                    ControlConstants.TurnMotor.kI,
                    ControlConstants.TurnMotor.kD,
                    ControlConstants.TurnMotor.kF);
            kTurn.closedLoop.outputRange(ControlConstants.TurnMotor.kMinOutput, ControlConstants.TurnMotor.kMaxOutput);

            // idle mode
            kDrive.idleMode(ModuleConstants.DriveMotor.kIdleMode);
            kTurn.idleMode(ModuleConstants.TurnMotor.kIdleMode);
        }
    }

    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersistMode = PersistMode.kPersistParameters;
}
