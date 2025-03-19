// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralL1 extends Command {
  private final ElevatorSubsystem m_elevator;
  private final CoralIntakeSubsytem m_coral;

  /** Creates a new CoralL1. */
  public CoralL1(ElevatorSubsystem elevator, CoralIntakeSubsytem coral) {
    m_elevator = elevator;
    m_coral = coral;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorHeight(ElevatorSubsystem.ElevatorHeight.L1);
    m_coral.setPivotAngle(CoralIntakeSubsytem.CoralIntakeAngles.L1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
    m_coral.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getHeight() == ElevatorSubsystem.ElevatorHeight.L1.getHeight() &&
      m_coral.getPivotAngle() == CoralIntakeSubsytem.CoralIntakeAngles.L1.getAngle();
  }
}
