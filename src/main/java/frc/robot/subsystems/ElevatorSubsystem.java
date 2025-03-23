package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
  // Enum for elevator heights
  public enum ElevatorHeight {
    HOME(kHomeHeight),
    L1(kL1Height),
    L2(kL2Height),
    L3(kL3Height),
    L4(kL4Height),
    CORAL_INTAKE(kCoralIntakeHeight);

    private final double height;

    ElevatorHeight(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }

  private TalonFX m_rightMotor = new TalonFX(kRightMotorId, "rio");
  private TalonFX m_leftMotor = new TalonFX(kLeftMotorId, "rio");

  private final PositionVoltage PID_controller = new PositionVoltage(0).withSlot(0);

  private final DigitalInput m_limitSwitch;

  private NetworkTableEntry kPEntry;
  private NetworkTableEntry kDEntry;
  private NetworkTableEntry kGEntry;
  private NetworkTableEntry currentHeightEntry;
  private NetworkTableEntry targetHeightEntry;
  private NetworkTableEntry limitSwitchStateEntry;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // Set motor configurations
    TalonFXConfiguration m_motorConfigs = new TalonFXConfiguration();
    m_motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorConfigs.Slot0.kP = kP;
    m_motorConfigs.Slot0.kI = kI;
    m_motorConfigs.Slot0.kD = kD;
    // m_motorConfigs.Slot0.kG = kG;

    // Set current limits
    CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
    m_currentLimits.StatorCurrentLimit = 120;
    m_currentLimits.StatorCurrentLimitEnable = true;

    m_motorConfigs.Feedback.SensorToMechanismRatio = kGearRatio * Math.PI * kSprocketDiameter * kStages;

    // Apply motor configurations
    m_rightMotor.getConfigurator().apply(m_motorConfigs);
    m_rightMotor.getConfigurator().apply(m_currentLimits);

    m_leftMotor.getConfigurator().apply(m_motorConfigs);
    m_leftMotor.getConfigurator().apply(m_currentLimits);

    // Left motor follows right
    m_leftMotor.setControl(new Follower(kRightMotorId, false));

    m_limitSwitch = new DigitalInput(kLimitSwitchPort);
    
    // Reset the position to 0
    resetPosition();

    // Get the NetworkTable instance and table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Elevator Subsystem");

    // Initialize NetworkTable entries
    // kPEntry = table.getEntry("Elevator P");
    // kDEntry = table.getEntry("Elevator D");
    // kGEntry = table.getEntry("Elevator G");
    currentHeightEntry = table.getEntry("Current Height");
    // targetHeightEntry = table.getEntry("Target Height");
    limitSwitchStateEntry = table.getEntry("Limit Switch State");

    // // Set initial values for PID entries
    // kPEntry.setDouble(kP);
    // kDEntry.setDouble(kD);
    // kGEntry.setDouble(kG);

    // Tunable PID
    // initialConfig = new TalonFXConfiguration();
    // applyPIDConfigs(initialConfig);

    // Gear Ratio for correct units
    // initialConfig.Feedback.SensorToMechanismRatio = -(kSprocketDiameter * Math.PI)/(kGearRatio * kStages);
    // initialConfig.Feedback.SensorToMechanismRatio = kGearRatio * 2;

    // Apply configuration
    // m_rightMotor.getConfigurator().apply(initialConfig);
  }

  // public void applyPIDConfigs(TalonFXConfiguration talonFXConfigs) {
  //   Slot0Configs slot0Configs = talonFXConfigs.Slot0;
  //   slot0Configs.kI = kI;

  //   // slot0Configs.kP = kPEntry.getDouble(0.0);
  //   // slot0Configs.kD = kDEntry.getDouble(0.0);

  //   double newP = kPEntry.getDouble(slot0Configs.kP);
  //   double newD = kDEntry.getDouble(slot0Configs.kD);
  //   double newG = kGEntry.getDouble(slot0Configs.kG);
  //   if (newP != slot0Configs.kP || newD != slot0Configs.kD) {
  //     slot0Configs.kP = newP;
  //     slot0Configs.kD = newD;
  //     slot0Configs.kG = newG;
  //     m_rightMotor.getConfigurator().apply(initialConfig);
  //     // System.out.println("Applied New PID: P=" + newP + ", D=" + newD);
  //   }
  // }

  public void setElevatorHeight(ElevatorHeight height) {
    double inches = height.getHeight();

    // Set the motor position using the PID controller
    m_rightMotor.setControl(PID_controller.withPosition(inches).withFeedForward(kG));
  }

  public double getHeight() {
    // double rotations = m_rightMotor.getPosition().getValueAsDouble();
    // double primaryStageInches = rotations * Math.PI * kSprocketDiameter;

    // return primaryStageInches * kStages;
    return m_rightMotor.getPosition().getValueAsDouble();
  }

  public void stop() {
    m_rightMotor.set(0);
  }

  public void resetPosition() {
    m_rightMotor.setPosition(0);
  }

  public void setSpeed(double speed) {
    // m_rightMotor.set(speed);
    // m_rightMotor.setControl(new DutyCycleOut(speed).withLimitReverseMotion(m_limitSwitch.get()));
    m_rightMotor.setControl(new DutyCycleOut(speed));

  }

  @Override
  public void periodic() {
    // Apply PID configurations from NetworkTable entries
    // applyPIDConfigs(initialConfig);

    // Update current height entry
    double currentHeight = getHeight();
    currentHeightEntry.setDouble(currentHeight);

    // if (limitSwitchStateEntry.getBoolean(false) != m_limitSwitch.get())
    limitSwitchStateEntry.setBoolean(m_limitSwitch.get());
    
    if (m_limitSwitch.get()) {
      // resetPosition();
      // stop();
    }
  }
}