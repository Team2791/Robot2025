package frc.robot.constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigConstants {
    public static final class Elevator {
        public static final SparkFlexConfig kLeader;
        public static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);

            // position and velocity factors
            kLeader.encoder.positionConversionFactor(ElevatorConstants.Encoder.kPositionFactor);
            kLeader.encoder.velocityConversionFactor(ElevatorConstants.Encoder.kVelocityFactor);

            // pid constants
            kLeader.closedLoop.pidf(
                ControlConstants.Elevator.kP,
                ControlConstants.Elevator.kI,
                ControlConstants.Elevator.kD,
                ControlConstants.Elevator.kF
            );

            kLeader.closedLoop.outputRange(
                ControlConstants.Elevator.kMin,
                ControlConstants.Elevator.kMax
            );

            // idle mode
            kLeader.idleMode(ElevatorConstants.Motor.kIdleMode);
            kFollower.idleMode(ElevatorConstants.Motor.kIdleMode);

            // leader-follower
            kFollower.follow(IOConstants.Elevator.kLeader, ElevatorConstants.Motor.kInvertFollower);
        }
    }

    public static final class Dispenser {
        public static final SparkMaxConfig kLeader;
        public static final SparkMaxConfig kFollower;

        static {
            kLeader = new SparkMaxConfig();
            kFollower = new SparkMaxConfig();

            // current limits
            kLeader.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);

            // idle mode
            kLeader.idleMode(DispenserConstants.Motor.kIdleMode);
            kFollower.idleMode(DispenserConstants.Motor.kIdleMode);

            // leader-follower
            kFollower.follow(IOConstants.Dispenser.kLeader, DispenserConstants.Motor.kInvertFollower);

            // beam brake
            kLeader.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
            kLeader.limitSwitch.forwardLimitSwitchEnabled(false);
            kLeader.limitSwitch.reverseLimitSwitchEnabled(false);
        }
    }

    public static final class Intake {
        public static final SparkMaxConfig kLeft;
        public static final SparkMaxConfig kRight;

        static {
            kLeft = new SparkMaxConfig();
            kRight = new SparkMaxConfig();

            // current limits
            kLeft.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);
            kRight.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);

            // position and velocity factors
            kLeft.encoder.positionConversionFactor(IntakeConstants.Encoder.kPositionFactor);
            kRight.encoder.positionConversionFactor(IntakeConstants.Encoder.kPositionFactor);
            kLeft.encoder.velocityConversionFactor(IntakeConstants.Encoder.kVelocityFactor);
            kRight.encoder.velocityConversionFactor(IntakeConstants.Encoder.kVelocityFactor);

            // idle mode
            kLeft.idleMode(IntakeConstants.Motor.kIdleMode);
            kRight.idleMode(IntakeConstants.Motor.kIdleMode);

            // beam brake
            kLeft.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
            kLeft.limitSwitch.forwardLimitSwitchEnabled(false);
            kLeft.limitSwitch.reverseLimitSwitchEnabled(false);
        }
    }

    public static final class AlgaeManipulator {
        public static final SparkMaxConfig kTurn;
        public static final SparkMaxConfig kSpin;

        static {
            kTurn = new SparkMaxConfig();
            kSpin = new SparkMaxConfig();

            // current limits
            kTurn.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);
            kSpin.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);

            // position and velocity factors
            kTurn.encoder.positionConversionFactor(AlgaeManipulatorConstants.TurnEncoder.kPositionFactor);
            kSpin.encoder.positionConversionFactor(AlgaeManipulatorConstants.SpinEncoder.kPositionFactor);
            kTurn.encoder.velocityConversionFactor(AlgaeManipulatorConstants.TurnEncoder.kVelocityFactor);
            kSpin.encoder.velocityConversionFactor(AlgaeManipulatorConstants.SpinEncoder.kVelocityFactor);

            // pid constants (turn only, spin is open loop)
            kTurn.closedLoop.pidf(
                ControlConstants.AlgaeManipulator.kP,
                ControlConstants.AlgaeManipulator.kI,
                ControlConstants.AlgaeManipulator.kD,
                ControlConstants.AlgaeManipulator.kF
            );

            kTurn.closedLoop.outputRange(
                ControlConstants.AlgaeManipulator.kMin,
                ControlConstants.AlgaeManipulator.kMax
            );

            // idle mode
            kTurn.idleMode(AlgaeManipulatorConstants.TurnMotor.kIdleMode);
            kSpin.idleMode(AlgaeManipulatorConstants.SpinMotor.kIdleMode);

            // ???
            kTurn.limitSwitch.reverseLimitSwitchEnabled(false);
            kTurn.limitSwitch.forwardLimitSwitchEnabled(false);
        }
    }

    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersistMode = PersistMode.kPersistParameters;
}
