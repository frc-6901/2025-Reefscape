// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  public enum IntakeAngle {
    Rest(kRestAngle),
    L1(kL1Angle),
    L2(kL2Angle),
    L3(kL3Angle),
    L4(kL4Angle),
    CORAL_INTAKE(kCoralIntakeAngle);

    private final double angle;

    IntakeAngle(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }

  private TalonFX m_PivotMotor = new TalonFX(kPivotMotorId, "rio");
  private MotionMagicVoltage m_PivotMotionMagic = new MotionMagicVoltage(0);
  private NetworkTableEntry kPivotPEntry = SmartDashboard.getEntry("Intake Pivot P");
  private NetworkTableEntry kPivotDEntry = SmartDashboard.getEntry("Intake Pivot D");
  private NetworkTableEntry kPivotVEntry = SmartDashboard.getEntry("Intake Pivot V");
  private TalonFXConfiguration initialPivotConfig;

  private TalonFX m_CoralMotor = new TalonFX(kCoralMotorId, "rio");
  // private LimitSwitch

  private TalonFX m_AlgaeRightMotor = new TalonFX(kAlgaeRightMotorId, "rio");
  private TalonFX m_AlgaeLeftMotor = new TalonFX(kAlgaeLeftMotorId, "rio");

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_PivotMotor.setNeutralMode(NeutralModeValue.Brake);
    m_CoralMotor.setNeutralMode(NeutralModeValue.Brake);
    m_AlgaeRightMotor.setNeutralMode(NeutralModeValue.Brake);
    m_AlgaeLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    // Left motor follows right
    m_AlgaeLeftMotor.setControl(new Follower(kAlgaeRightMotorId, true));

    // Read initial PID values from Shuffleboard
    SmartDashboard.putNumber("Intake Pivot P", kPivotP);
    SmartDashboard.putNumber("Intake Pivot D", kPivotD);
    SmartDashboard.putNumber("Intake Pivot V", kPivotV);

    // PID
    initialPivotConfig = new TalonFXConfiguration();
    var slot0Configs = initialPivotConfig.Slot0;
    slot0Configs.kS = kPivotS; // Static friction voltage
    slot0Configs.kA = kPivotA; // Acceleration feedforward
    slot0Configs.kI = kPivotI; // Integral gain

    // Tunable PID
    applyPIDConfigs(initialPivotConfig);
    
    // Motion Magic
    var motionMagicConfigs = initialPivotConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30;
    motionMagicConfigs.MotionMagicAcceleration = 60;

    // Gear Ratio for correct units
    initialPivotConfig.Feedback.SensorToMechanismRatio = kGearRatio;

    // Apply configuration
    m_PivotMotor.getConfigurator().apply(initialPivotConfig);
  }
  
  public void applyPIDConfigs(TalonFXConfiguration talonFXConfigs) {
      var slot0Configs = talonFXConfigs.Slot0;
  
      slot0Configs.kP = kPivotPEntry.getDouble(0.0);
      slot0Configs.kD = kPivotDEntry.getDouble(0.0);
      slot0Configs.kV = kPivotVEntry.getDouble(0.0);
    }
  
  public void setPivotAngle(IntakeAngle angle) {
    double rotations = angle.getAngle() / 360.0;
    double target = rotations * kGearRatio;

    m_PivotMotor.setControl(m_PivotMotionMagic.withPosition(target));
  }

  public void intakeCoral() {
    intakeCoral(kCoralIntakeSpeed);
  }

  public void outakeCoral() {
    intakeCoral(-kCoralIntakeSpeed);
  }

  public void intakeCoral(double speed) {
    m_CoralMotor.setControl(new DutyCycleOut(speed));
  }

  public void outakeCoral(double speed) {
    m_CoralMotor.setControl(new DutyCycleOut(speed));
  }

  public void intakeAlgae() {
    intakeAlgae(kAlgaeIntakeSpeed);
  }

  public void outakeAlgae() {
    intakeAlgae(-kAlgaeIntakeSpeed);
  }

  public void intakeAlgae(double speed) {
    m_AlgaeRightMotor.setControl(new DutyCycleOut(speed));
  }

  public void outakeAlgae(double speed) {
    m_AlgaeRightMotor.setControl(new DutyCycleOut(speed));
  }

  public double getAngle() {
    return m_PivotMotor.getPosition().getValueAsDouble() / kGearRatio * 360.0;
  }

  @Override
  public void periodic() {
    applyPIDConfigs(initialPivotConfig);
    System.out.println("Pivot Angle: " + getAngle());
  }
}
