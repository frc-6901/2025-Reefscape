package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Constants {
    public static final class ControllerConstants {
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;
        public static final double kDriveDeadband = 0.1;
      }

    public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.
    
        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(3.1).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);
    
        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    
        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the steer motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    
        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
    
        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120.0);
    
        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;
    
        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("Drive SubSystem CANivore", "./logs/example.hoot");
    
        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        // public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.48);
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3);
    
        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 0;
    
        private static final double kDriveGearRatio = 6.23;
        private static final double kSteerGearRatio = 25;
        private static final Distance kWheelRadius = Inches.of(1.75);
    
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;
    
        private static final int kPigeonId = 15;
    
        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
    
        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);
    
        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);
    
    
        // Front Left
        private static final int kFrontLeftDriveMotorId = 2;
        private static final int kFrontLeftSteerMotorId = 1;
        private static final int kFrontLeftEncoderId = 10;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.134033203125);
        private static final boolean kFrontLeftSteerMotorInverted = false;
        private static final boolean kFrontLeftEncoderInverted = false;
    
        private static final Distance kFrontLeftXPos = Inches.of(13.5);
        private static final Distance kFrontLeftYPos = Inches.of(10);
    
        // Front Right
        private static final int kFrontRightDriveMotorId = 4;
        private static final int kFrontRightSteerMotorId = 3;
        private static final int kFrontRightEncoderId = 11;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0.321044921875);
        private static final boolean kFrontRightSteerMotorInverted = false;
        private static final boolean kFrontRightEncoderInverted = false;
    
        private static final Distance kFrontRightXPos = Inches.of(13.5);
        private static final Distance kFrontRightYPos = Inches.of(-10);
    
        // Back Left
        private static final int kBackLeftDriveMotorId = 8;
        private static final int kBackLeftSteerMotorId = 7;
        private static final int kBackLeftEncoderId = 13;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.42138671875);
        private static final boolean kBackLeftSteerMotorInverted = false;
        private static final boolean kBackLeftEncoderInverted = false;
    
        private static final Distance kBackLeftXPos = Inches.of(-13.5);
        private static final Distance kBackLeftYPos = Inches.of(10);
    
        // Back Right
        private static final int kBackRightDriveMotorId = 6;
        private static final int kBackRightSteerMotorId = 5;
        private static final int kBackRightEncoderId = 12;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.31982421875);
        private static final boolean kBackRightSteerMotorInverted = false;
        private static final boolean kBackRightEncoderInverted = false;
    
        private static final Distance kBackRightXPos = Inches.of(-13.5);
        private static final Distance kBackRightYPos = Inches.of(-10);
    
    
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );
    
        /**
         * Creates a CommandSwerveDrivetrain instance.
         * This should only be called once in your robot program,.
         */
        public static CommandSwerveDrivetrain createDrivetrain() {
            return new CommandSwerveDrivetrain(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
            );
        }
    
    
        /**
         * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
         */
        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
             * @param modules               Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules
                );
            }
    
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency The frequency to run the odometry loop. If
             *                                unspecified or set to 0 Hz, this is 250 Hz on
             *                                CAN FD, and 100 Hz on CAN 2.0.
             * @param modules                 Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules
                );
            }
    
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
             *                                  unspecified or set to 0 Hz, this is 250 Hz on
             *                                  CAN FD, and 100 Hz on CAN 2.0.
             * @param odometryStandardDeviation The standard deviation for odometry calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param visionStandardDeviation   The standard deviation for vision calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param modules                   Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules
                );
            }
        }
    }
    
    public static final class VisionConstants {
        public static final String kCameraName = "Arducam_OV9281_USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(5), 0), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 20);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final double kVisionP = 0.28;
        public static final double kVisionI = 0.0;
        public static final double kVisionD = 0.0;
  }

    public static final class ElevatorConstants {
        // Motor IDs
        public static final int kRightMotorId = 20;
        public static final int kLeftMotorId = 21;

        // Limit Switch
        public static final int kLimitSwitchPort = 9;

        // Elevator Stages
        public static final int kStages = 3;
        
        // Gear Ratio
        public static final double kGearRatio = 5.0;
        public static final double kSprocketDiameter = 1.751;
        
        // Position PID Constants
        public static double kP = 21;
        public static double kI = 5;
        public static double kD = 0.1;
        public static double kG = 1.5;
        // public static double kS = 0.1;
        // public static double kV = 0.1;
        // public static double kA = 0.01;
        
        // Heights in INCHES
        public static double kHomeHeight = 0;
        public static double kL1Height = .05;
        public static double kL2Height = .1;
        public static double kL3Height = .129;
        public static double kL4Height = .15;
        public static double kCoralIntakeHeight = .034;
    }

    public static final class AlgaeIntakeConstants {
        // Motor IDs
        public static final int kPivotMotorId = 30;
        public static final int kRightMotorId = 31;
        public static final int kLeftMotorId = 32;

        // Can Bus
        public static final String kCanBusName = "rio";

        // Gear Ratios
        public static final double kPivotGearRatio = 125.0;
        public static final double kIntakeGearRatio = 36.0;
        
        // Speeds
        public static double kPivotSpeed = 0.15;
        public static double kIntakeSpeed = 0.7;
        public static double kOuttakeSpeed = -1;

        // PID Angles
        public static double kHomeAngle = 0;
        public static double kReefAngle = 35;
        public static double kProcessorAngle = 35;
        public static double kBargeAngle = 35;

        // PID Constants
        public static double kPivotP = 0.5;
        public static double kPivotI = 0;
        public static double kPivotD = 0.01;
        public static double kPivotG = 0.3;
    }

    public static final class CoralIntakeConstants {
        // Motor IDs
        public static final int kPivotMotorId = 35;
        public static final int kIntakeMotorId = 36;

        // Can Bus
        public static final String kCanBusName = "rio";

        // Gear Ratios
        public static final double kPivotGearRatio = 45.0;
        public static final double kIntakeGearRatio = 45.0;

        // Speeds
        public static double kPivotSpeed = 0.1;
        public static double kIntakeSpeed = 0.5;
        public static double kOuttakeSpeed = -1;

        // PID Angles
        public static double kHomeAngle = 0;
        public static double kL1Angle = -0.28;
        public static double kL2Angle = -0.24;
        public static double kL3Angle = -0.24;
        public static double kL4Angle = -0.3;
        public static double kIntakeAngle = -0.132;

        // PID Constants
        public static double kPivotP = 30;
        public static double kPivotI = 0.5;
        public static double kPivotD = .1;
    }

    public static final class ClimberConstants {
        // Motor ID
        public static final int kMotorId = 40;

        // Speeds
        public static double kClimbSpeed = 0.5;
        public static double kLowerSpeed = -0.5;
    }
}
