// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.annotation.ElementType;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CoralIntakeSubsytem.CoralIntakeAngles;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeight;
import frc.robot.commands.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverPort);
    private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final CoralIntakeSubsytem coral = new CoralIntakeSubsytem();
    public final AlgaeIntakeSubsystem algae = new AlgaeIntakeSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();
    // public final VisionSubsystem vision = new VisionSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
                                                                                                                                                                                                                                               
        // Register Named Commands
        // NamedCommands.registerCommand("coralScoreL1", new ScoreL1());
        // NamedCommands.registerCommand("coralScoreL2", new ScoreL2());
        // NamedCommands.registerCommand("coralScoreL3", new ScoreL3());
        // NamedCommands.registerCommand("coralScoreL4", new ScoreL4());
        // NamedCommands.registerCommand("keepAutonCorl", new keepAutonCoral());

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-driver.getLeftY() / 2) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY((-driver.getLeftX() / 2) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // driver.y().whileTrue(new AutoAlignCommand(drivetrain, vision));

        driver.rightTrigger().whileTrue(new RunCommand(() -> climber.climb(), climber));
        driver.leftTrigger().whileTrue(new RunCommand(() -> climber.lowerClimber(), climber));

        driver.rightTrigger().whileFalse(new RunCommand(() -> climber.stop(), climber));
        driver.leftTrigger().whileFalse(new RunCommand(() -> climber.stop(), climber));
    }

    private void configureOperatorBindings() {
        // Elevator control
        elevator.setDefaultCommand(new RunCommand(() -> {
            double speed = -operator.getLeftY() / 2;
            if (Math.abs(speed) > 0.025) {
            elevator.setSpeed(speed);
            } else {
            elevator.setSpeed(0);
            }
        }, elevator));

        // Coral Intake control
        // coral.setDefaultCommand(new RunCommand(() -> {
        //     coral.stopAllMotors();
        // }, coral));

        operator.leftTrigger().whileTrue(new InstantCommand(() -> coral.intakeCoral()));
        operator.rightTrigger().whileTrue(new InstantCommand(() -> coral.outtakeCoral()));

        operator.leftTrigger().whileFalse(new InstantCommand(() -> coral.stopIntake()));
        operator.rightTrigger().whileFalse(new InstantCommand(() -> coral.stopIntake()));

        operator.leftBumper().whileTrue(new InstantCommand(() -> coral.spinPivot(true)));
        operator.rightBumper().whileTrue(new InstantCommand(() -> coral.spinPivot(false)));

        operator.leftBumper().whileFalse(new InstantCommand(() -> coral.stopPivot()));
        operator.rightBumper().whileFalse(new InstantCommand(() -> coral.stopPivot()));

        //operator.povDown().onTrue(new InstantCommand(() -> coral.resetPivotAngle()));

        // Algae Intake control
        // algae.setDefaultCommand(new RunCommand(() -> {
        //     algae.stopAllMotors();
        // }, algae));

        operator.a().whileTrue(new InstantCommand(() -> algae.intakeAlgae()));
        operator.y().whileTrue(new InstantCommand(() -> algae.outtakeAlgae()));

        operator.a().whileFalse(new InstantCommand(() -> algae.stopIntake()));
        operator.y().whileFalse(new InstantCommand(() -> algae.stopIntake()));

        operator.x().whileTrue(new InstantCommand(() -> algae.spinPivot(true)));
        operator.b().whileTrue(new InstantCommand(() -> algae.spinPivot(false)));

        operator.x().whileFalse(new InstantCommand(() -> algae.stopPivot()));
        operator.b().whileFalse(new InstantCommand(() -> algae.stopPivot()));

        // operator.povUp().whileTrue(new RunCommand(() -> elevator.setElevatorHeight(ElevatorHeight.CORAL_INTAKE)));
        // operator.povUp().whileTrue(new ParallelCommandGroup(
        //     new RunCommand(() -> elevator.setElevatorHeight(ElevatorHeight.CORAL_INTAKE)),
        //     new RunCommand(() -> coral.setPivotAngle(CoralIntakeAngles.INTAKE))
        // ));
        operator.povUp().whileTrue(new SequentialCommandGroup(
            new InstantCommand(() -> coral.setPivotAngle(CoralIntakeAngles.HOME)),
            new ParallelCommandGroup(
                new RunCommand(() -> elevator.setElevatorHeight(ElevatorHeight.CORAL_INTAKE)),
                new RunCommand(() -> coral.setPivotAngle(CoralIntakeAngles.INTAKE))
        )));
        operator.povRight().whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> coral.resetPivotAngle()),
            new InstantCommand(() -> elevator.resetPosition())
        ));
        operator.povDown().whileTrue(new SequentialCommandGroup(
            new InstantCommand(() -> coral.setPivotAngle(CoralIntakeAngles.HOME)),
            new ParallelCommandGroup(
                new RunCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L3)),
                new RunCommand(() -> coral.setPivotAngle(CoralIntakeAngles.L3))
        )));

        //operator.povDown().whileTrue(new RunCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L1)));
        //operator.povLeft().whileTrue(new RunCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L4)));
        //operator.povRight().onTrue(new InstantCommand(() -> elevator.resetPosition()));
        // operator.povDown().whileTrue(new RunCommand(() -> coral.setPivotAngle(CoralIntakeAngles.L4)));
        operator.povLeft().whileTrue(new RunCommand(() -> coral.setPivotAngle(CoralIntakeAngles.HOME)));
        // operator.povRight().whileTrue(new RunCommand(() -> coral.setPivotAngle(CoralIntakeAngles.INTAKE)));
        //operator.povRight().whileTrue(new RunCommand(() -> coral.resetPivotAngle()));

        
        
        // operator.povDown().onTrue(new SequentialCommandGroup(
        //     // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L1))
        //     // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L1))
        // ));

        // operator.povRight().onTrue(new SequentialCommandGroup(
        //     // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L2))
        //     // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L2))
        // ));

        // operator.povLeft().onTrue(new SequentialCommandGroup(
        //     // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L3))
        //     // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L3))
        // ));

        // operator.povUp().onTrue(new SequentialCommandGroup(
        //     // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L4))
        //     // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L4))
        // ));
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        //return new PathPlannerAuto("Reefscape Auton");
        // return autoChooser.getSelected();
        // return new PathPlannerAuto("Score L1 Straight");
        return new PathPlannerAuto("Straight");

    }
}
