// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(kMotorId, "rio");

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Set motor configurations
    TalonFXConfiguration m_motorConfigs = new TalonFXConfiguration();
    m_motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_motor.getConfigurator().apply(m_motorConfigs);
  }

  public void lowerClimber() {
    m_motor.setControl(new DutyCycleOut(kLowerSpeed));
  }

  public void climb() {
    m_motor.setControl(new DutyCycleOut(kClimbSpeed));
  }

  public void stop() {
    m_motor.setControl(new DutyCycleOut(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
