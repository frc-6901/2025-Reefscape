package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

import java.io.ObjectInputFilter.Config;

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

  private NetworkTableEntry kPEntry;
  private NetworkTableEntry kDEntry;
  private NetworkTableEntry kGEntry;
  private NetworkTableEntry currentHeightEntry;
  private NetworkTableEntry targetHeightEntry;
  private NetworkTableEntry resetPositionEntry;

  private TalonFXConfiguration initialConfig;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    
    m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    // motorConfigs.withNeutralMode(NeutralModeValue.Brake);
    // motorConfigs.apply(currentLimitConfigs);
    
    CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    currentLimitConfigs.StatorCurrentLimit = 100;
    currentLimitConfigs.StatorCurrentLimitEnable = true;
    
    var configurator = m_rightMotor.getConfigurator();
    configurator.apply(currentLimitConfigs);
    configurator.apply(motorConfigs);
    
    configurator = m_leftMotor.getConfigurator();
    configurator.apply(currentLimitConfigs);
    configurator.apply(motorConfigs);
    
    // Left motor follows right
    m_leftMotor.setControl(new Follower(kRightMotorId, false));
    
    // Reset the position to 0
    resetPosition();

    // Get the NetworkTable instance and table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Elevator Subsystem");

    // Initialize NetworkTable entries
    kPEntry = table.getEntry("Elevator P");
    kDEntry = table.getEntry("Elevator D");
    kGEntry = table.getEntry("Elevator G");
    currentHeightEntry = table.getEntry("Current Height");
    targetHeightEntry = table.getEntry("Target Height");
    resetPositionEntry = table.getEntry("Reset Position");

    // Set initial values for PID entries
    kPEntry.setDouble(kP);
    kDEntry.setDouble(kD);
    kGEntry.setDouble(kG);

    // Tunable PID
    initialConfig = new TalonFXConfiguration();
    applyPIDConfigs(initialConfig);

    // Gear Ratio for correct units
    // initialConfig.Feedback.SensorToMechanismRatio = -(kSprocketDiameter * Math.PI)/(kGearRatio * kStages);
    initialConfig.Feedback.SensorToMechanismRatio = kGearRatio * 2;

    // Apply configuration
    m_rightMotor.getConfigurator().apply(initialConfig);
  }

  public void applyPIDConfigs(TalonFXConfiguration talonFXConfigs) {
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kI = kI;

    // slot0Configs.kP = kPEntry.getDouble(0.0);
    // slot0Configs.kD = kDEntry.getDouble(0.0);

    double newP = kPEntry.getDouble(slot0Configs.kP);
    double newD = kDEntry.getDouble(slot0Configs.kD);
    double newG = kGEntry.getDouble(slot0Configs.kG);
    if (newP != slot0Configs.kP || newD != slot0Configs.kD) {
      slot0Configs.kP = newP;
      slot0Configs.kD = newD;
      slot0Configs.kG = newG;
      m_rightMotor.getConfigurator().apply(initialConfig);
      // System.out.println("Applied New PID: P=" + newP + ", D=" + newD);
    }
  }

  public void setElevatorHeight(ElevatorHeight height) {
    // double inches = height.getHeight();

    // double rotations = inches / (Math.PI * kSprocketDiameter);
    // m_rightMotor.setControl(PID_controller.withPosition(rotations).withFeedForward(kG));

    // // Update target height entry
    // targetHeightEntry.setDouble(inches);

    double inches = height.getHeight();

    // Convert inches to rotations
    double rotations = (inches / kStages) / (Math.PI * kSprocketDiameter);

    // Set the motor position using the PID controller
    m_rightMotor.setControl(PID_controller.withPosition(rotations).withFeedForward(kG));

    // Update target height entry
    targetHeightEntry.setDouble(inches);
  }

  public double getHeight() {
    double rotations = m_rightMotor.getPosition().getValueAsDouble();
    double primaryStageInches = rotations * Math.PI * kSprocketDiameter;

    return primaryStageInches * kStages;
  }

  public void stop() {
    m_rightMotor.set(0);
  }

  public void resetPosition() {
    m_rightMotor.setPosition(0);
  }

  public void setSpeed(double speed) {
	// m_rightMotor.set(speed);
    m_rightMotor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void periodic() {
    // Apply PID configurations from NetworkTable entries
    applyPIDConfigs(initialConfig);

    // Update current height entry
    double currentHeight = getHeight();
    currentHeightEntry.setDouble(currentHeight);

    // Check if the reset position button is pressed
    if (resetPositionEntry.getBoolean(false)) {
      resetPosition();
      resetPositionEntry.setBoolean(false); // Reset the button state
    }
    
    // m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    // m_leftMotor.setNeutralMode(NeutralModeValue.Brake);

    // Stall protection
    // if (m_rightMotor.getStatorCurrent().getValueAsDouble() > 40.0) {
    //   System.out.println("Elevator Stalled! Stopping...");
    //   stop();
    // }
  }
}