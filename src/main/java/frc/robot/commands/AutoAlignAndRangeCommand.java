// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.*;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import static edu.wpi.first.units.Units.*;


// import static frc.robot.Constants.TunerConstants.*;

// ;
// public class AutoAlignAndRangeCommand extends Command {
//     private final CommandSwerveDrivetrain m_DriveSubsystem;
//     private final SwerveRequest.FieldCentric m_driveRequest;
//     private final VisionSubsystem m_VisionSubsystem;
//     private final PIDController m_yawPID;
//     private final PIDController m_distancePID;

//     private static final double TARGET_DISTANCE_METERS = 1;
//     private double MaxSpeed = kSpeedAt12Volts.in(MetersPerSecond);
//     private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

//     public AutoAlignAndRangeCommand(CommandSwerveDrivetrain m_DriveSubsystem, VisionSubsystem m_VisionSubsystem) {
//         this.m_DriveSubsystem = m_DriveSubsystem;
//         this.m_VisionSubsystem = m_VisionSubsystem;

//         this.m_yawPID = new PIDController(0.03, 0, 0);
//         this.m_distancePID = new PIDController(0.1, 0, .01);

//         m_driveRequest = new SwerveRequest.FieldCentric()
//         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
//         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//         addRequirements(m_DriveSubsystem, m_VisionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         m_yawPID.setSetpoint(0); // Align with tag
//         m_distancePID.setSetpoint(TARGET_DISTANCE_METERS); // Move to target range
//     }

//     @Override
//     public void execute() {
//         if (!m_VisionSubsystem.hasTarget()) {
//             m_DriveSubsystem.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
//             return;
//         }

//         double yawCorrection = m_yawPID.calculate(m_VisionSubsystem.getYaw(), 0);
//         double distanceCorrection = m_distancePID.calculate(m_VisionSubsystem.getDistance(), TARGET_DISTANCE_METERS);

//         m_DriveSubsystem.applyRequest(() -> m_driveRequest
//                 .withVelocityX(distanceCorrection);
//     }

//     @Override
//     public boolean isFinished() {
//         return m_yawPID.atSetpoint() && m_distancePID.atSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_DriveSubsystem.stop();
//     }
// }
