// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.kI;
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
  private PositionVoltage m_PivotPID_Controller = new PositionVoltage(0);
  private NetworkTableEntry kPivotPEntry;
  private NetworkTableEntry kPivotDEntry;
  private NetworkTableEntry kPivotGEntry;
  private NetworkTableEntry kPivotAngleEntry;
  private TalonFXConfiguration initialPivotConfig;

  private TalonFX m_CoralMotor = new TalonFX(kCoralMotorId, "rio");
  // private LimitSwitch

  // private TalonFX m_AlgaeRightMotor = new TalonFX(kAlgaeRightMotorId, "rio");
  // private TalonFX m_AlgaeLeftMotor = new TalonFX(kAlgaeLeftMotorId, "rio");

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_PivotMotor.setNeutralMode(NeutralModeValue.Brake);
    m_CoralMotor.setNeutralMode(NeutralModeValue.Brake);
    // m_AlgaeRightMotor.setNeutralMode(NeutralModeValue.Brake);
    // m_AlgaeLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    // Left Algae motor follows right
    // m_AlgaeLeftMotor.setControl(new Follower(kAlgaeRightMotorId, true));

    // Invert coral motor
    MotorOutputConfigs intakeMotorConfig = new MotorOutputConfigs();
    intakeMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
    m_CoralMotor.getConfigurator().apply(intakeMotorConfig);
    // m_AlgaeRightMotor.getConfigurator().apply(intakeMotorConfig);

    // Get the NetworkTable instance and table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Intake Subsystem");

    // Read initial PID values from NetworkTable
    kPivotPEntry = table.getEntry("Intake Pivot P");
    kPivotDEntry = table.getEntry("Intake Pivot D");
    kPivotGEntry = table.getEntry("Intake Pivot G");

    kPivotAngleEntry = table.getEntry("Intake Pivot Angle");

    m_PivotMotor.setPosition(0);

    // Tunable PID
    initialPivotConfig = new TalonFXConfiguration();
    applyPIDConfigs(initialPivotConfig);

    // Gear Ratio
    initialPivotConfig.Feedback.SensorToMechanismRatio = kGearRatio;

    // Apply configuration
    m_PivotMotor.getConfigurator().apply(initialPivotConfig);

    m_CoralMotor.set(0);
    m_PivotMotor.set(0);
  }
  
  public void applyPIDConfigs(TalonFXConfiguration talonFXConfigs) {
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kI = kI;

    slot0Configs.kP = kPivotPEntry.getDouble(0.0);
    slot0Configs.kD = kPivotDEntry.getDouble(0.0);
    // slot0Configs.kG = kPivotGE

    double newP = kPivotPEntry.getDouble(slot0Configs.kP);
    double newD = kPivotDEntry.getDouble(slot0Configs.kD);
    double newG = kPivotGEntry.getDouble(slot0Configs.kG);
    if (newP != slot0Configs.kP || newD != slot0Configs.kD || newG != slot0Configs.kG) {
      slot0Configs.kP = newP;
      slot0Configs.kD = newD;
      slot0Configs.kG = newG;
      m_PivotMotor.getConfigurator().apply(initialPivotConfig);
    }
  }
  
  public void setPivotAngle(IntakeAngle angle) {
    double rotations = angle.getAngle() / 360.0;
    double target = rotations * kGearRatio;

    m_PivotMotor.setControl(m_PivotPID_Controller.withPosition(target));
  }

  public void setPivotSpeed(double speed) {
    m_PivotMotor.set(speed);
  }

  public void stopPivot() {
    m_PivotMotor.set(0);
    m_PivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void intakeCoral() {
    intakeCoral(kCoralIntakeSpeed);
  }

  public void outakeCoral() {
    intakeCoral(-kCoralIntakeSpeed);
  }

  public void intakeCoral(double speed) {
    // m_CoralMotor.setControl(new DutyCycleOut(speed));
    m_CoralMotor.set(speed);
  }

  public void outakeCoral(double speed) {
    // m_CoralMotor.setControl(new DutyCycleOut(speed));
    m_CoralMotor.set(speed);
  }

  public void intakeAlgae() {
    intakeAlgae(kAlgaeIntakeSpeed);
  }

  public void outakeAlgae() {
    intakeAlgae(-kAlgaeIntakeSpeed);
  }

  public void intakeAlgae(double speed) {
    // m_AlgaeRightMotor.setControl(new DutyCycleOut(speed));
    // m_AlgaeRightMotor.set(speed);
  }

  public void outakeAlgae(double speed) {
    // m_AlgaeRightMotor.setControl(new DutyCycleOut(speed));
    // m_AlgaeRightMotor.set(speed);
  }

  public double getRotations() {
    return m_PivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // System.out.println("Pivot Angle: " + getRotations());
    double rotationns = getRotations();
    kPivotAngleEntry.setDouble(rotationns);
  }

  // public void goToScoring() {
  //   if (getRotations() < .14){
  //       m_PivotMotor.set(.025);
  //   }
  // }
}
