// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private NetworkTableEntry kPEntry = SmartDashboard.getEntry("Elevator P");
  private NetworkTableEntry kDEntry = SmartDashboard.getEntry("Elevator D");
  private NetworkTableEntry kVEntry = SmartDashboard.getEntry("Elevator V");

  private TalonFXConfiguration initialConfig;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);

    // Left motor follows right
    m_leftMotor.setControl(new Follower(kRightMotorId, true));

    // Read initial PID values from Shuffleboard
    SmartDashboard.putNumber("Elevator P", kP);
    SmartDashboard.putNumber("Elevator D", kD);
    SmartDashboard.putNumber("Elevator V", kV);

    // PID
    initialConfig = new TalonFXConfiguration();
    var slot0Configs = initialConfig.Slot0;
    slot0Configs.kS = kS; // Static friction voltage
    slot0Configs.kA = kA; // Acceleration feedforward
    slot0Configs.kI = kI; // Integral gain

    // Tunable PID
    applyPIDConfigs(initialConfig);

    // Motion Magic
    var pivotMotionMagicConfigs = initialConfig.MotionMagic;
    pivotMotionMagicConfigs.MotionMagicCruiseVelocity = 30;
    pivotMotionMagicConfigs.MotionMagicAcceleration = 60;

    // Gear Ratio for correct units
    initialConfig.Feedback.SensorToMechanismRatio = kGearRatio;

    // Apply configuration
    m_rightMotor.getConfigurator().apply(initialConfig);
  }

  public void applyPIDConfigs(TalonFXConfiguration talonFXConfigs) {
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = kPEntry.getDouble(0.0);
    slot0Configs.kD = kDEntry.getDouble(0.0);
    slot0Configs.kV = kVEntry.getDouble(0.0);
  }

  public void setElevatorHeight(ElevatorHeight height) {
    double inches = height.getHeight();

    // Convert inches to sensor counts
    double rotations = inches / (Math.PI * kSprocketDiameter);
    double target = rotations * kGearRatio;

    m_rightMotor.setControl(motionMagic.withPosition(target));
  }

  public double getPosition() {
    return m_rightMotor.getPosition().getValueAsDouble();
  }

  public void stop() {
    m_rightMotor.set(0);
  }

  public void periodic() {
    applyPIDConfigs(initialConfig);
    System.out.println("Elevator Position: " + getPosition());
  }
}
