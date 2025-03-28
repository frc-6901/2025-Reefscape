// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CoralIntakeConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralIntakeSubsytem extends SubsystemBase {
  // Enum for Algae Intake Angles
  public enum CoralIntakeAngles {
    HOME(kHomeAngle),
    L1(kL1Angle),
    L2(kL2Angle),
    L3(kL3Angle),
    L4(kL4Angle),
    INTAKE(kIntakeAngle);


    private final double angle;

    CoralIntakeAngles(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }

  private TalonFX m_pivotMotor = new TalonFX(kPivotMotorId, kCanBusName);
  private TalonFX m_intakeMotor = new TalonFX(kIntakeMotorId, kCanBusName);

  private PositionVoltage m_pivotPIDControllerRequest; 

  /** Creates a new AlgaeIntake. */
  public CoralIntakeSubsytem() {
    // Pivot Motor Configuration
    TalonFXConfiguration m_pivotMotorConfigs = new TalonFXConfiguration();
    m_pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_pivotMotorConfigs.Slot0.kP = kPivotP;
    m_pivotMotorConfigs.Slot0.kI = kPivotI;
    m_pivotMotorConfigs.Slot0.kD = kPivotD;
    m_pivotMotorConfigs.Feedback.SensorToMechanismRatio = kPivotGearRatio;

    // Pivot Current Limit Configuration
    CurrentLimitsConfigs m_pivotCurrentLimits = new CurrentLimitsConfigs();
    m_pivotCurrentLimits.StatorCurrentLimit = 120;
    m_pivotCurrentLimits.StatorCurrentLimitEnable = false;

    m_pivotMotor.getConfigurator().apply(m_pivotMotorConfigs);
    m_pivotMotor.getConfigurator().apply(m_pivotCurrentLimits);

    // Coral Intake Motor Configuration
    TalonFXConfiguration m_intkeMotorConfigs = new TalonFXConfiguration();
    m_intkeMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_intkeMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Algae Intake Current Limit Configuration
    CurrentLimitsConfigs m_intakeCurrentLimits = new CurrentLimitsConfigs();
    m_intakeCurrentLimits.StatorCurrentLimit = 40;
    m_intakeCurrentLimits.StatorCurrentLimitEnable = true;

    m_intakeMotor.getConfigurator().apply(m_intkeMotorConfigs);
    m_intakeMotor.getConfigurator().apply(m_intakeCurrentLimits);

    // Pivot PID Controller
    m_pivotPIDControllerRequest = new PositionVoltage(0).withSlot(0);

    // Reset pivot position
    resetPivotAngle();
  }

  public double getPivotAngle() {
    return m_pivotMotor.getPosition().getValueAsDouble();
  }

  public void setPivotAngle(CoralIntakeAngles angle) {
    //m_pivotMotor.setControl(m_pivotPIDControllerRequest.withPosition(angle.getAngle() - getPivotAngle()));
    m_pivotMotor.setControl(m_pivotPIDControllerRequest.withPosition(angle.getAngle()));
  }

  public void resetPivotAngle() {
    m_pivotMotor.setPosition(CoralIntakeAngles.HOME.getAngle());
  }

  public void stopPivot() {
    m_pivotMotor.setControl(new DutyCycleOut(0));
  }

  public void spinPivot(boolean reverse) {
    m_pivotMotor.setControl(new DutyCycleOut(reverse ? -kPivotSpeed : kPivotSpeed));
  }

  public void intakeCoral() {
    m_intakeMotor.setControl(new DutyCycleOut(kIntakeSpeed));
  }

  public void outtakeCoral() {
    m_intakeMotor.setControl(new DutyCycleOut(kOuttakeSpeed));
  }

  public void stopIntake() {
    m_intakeMotor.setControl(new DutyCycleOut(0));
  }

  public void stopAllMotors() {
    stopPivot();
    m_intakeMotor.setControl(new DutyCycleOut(0));
  }

  @Override
  public void periodic() {
    // Display pivot angle 
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Coral Intake");
    table.getEntry("Pivot Angle").setDouble(getPivotAngle());
  }
}
