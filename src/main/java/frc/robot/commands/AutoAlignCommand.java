package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import static edu.wpi.first.units.Units.*;

import static frc.robot.Constants.TunerConstants.*;

public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain m_DriveSubsystem;
    private final SwerveRequest.FieldCentric m_driveRequest;
    private final VisionSubsystem m_VisionSubsystem;

    // PID Controllers for X (forward/back), Y (sideways), and yaw (rotation)
    private final PIDController m_xPID;
    private final PIDController m_yPID;
    private final PIDController m_yawPID;

    private static final double TARGET_DISTANCE_METERS = 1; // Desired distance from the target
    private double MaxSpeed = kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public AutoAlignCommand(CommandSwerveDrivetrain m_DriveSubsystem, VisionSubsystem m_VisionSubsystem) {
        this.m_DriveSubsystem = m_DriveSubsystem;
        this.m_VisionSubsystem = m_VisionSubsystem;

        // PID Controller Tuning (Adjust for optimal performance)
        this.m_xPID = new PIDController(0.1, 0, 0.01); // Forward/Backward
        this.m_yPID = new PIDController(0.1, 0, 0.01); // Left/Right
        this.m_yawPID = new PIDController(0.03, 0, 0.01); // Rotation

        // Set deadbands and request type
        m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(m_DriveSubsystem, m_VisionSubsystem);
    }

    @Override
    public void initialize() {
        m_xPID.setSetpoint(TARGET_DISTANCE_METERS); // Move to target range
        m_yPID.setSetpoint(0); // Center on target
        m_yawPID.setSetpoint(0); // Align with tag
    }

    @Override
    public void execute() {
        if (!m_VisionSubsystem.hasTarget()) {
            m_DriveSubsystem.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()); // Stop if no target
            return;
        }

        // PID calculations
        double xCorrection = m_xPID.calculate(m_VisionSubsystem.getDistance(), TARGET_DISTANCE_METERS);
        double yCorrection = m_yPID.calculate(m_VisionSubsystem.getStrafeOffset(), 0);
        double yawCorrection = m_yawPID.calculate(m_VisionSubsystem.getYaw(), 0);
        
        m_DriveSubsystem.setControl(m_driveRequest
            .withVelocityX(xCorrection)
            .withVelocityY(yCorrection)
            .withRotationalRate(yawCorrection));
    }

    @Override
    public boolean isFinished() {
        return m_yPID.atSetpoint() && m_xPID.atSetpoint() && m_yawPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }
}
