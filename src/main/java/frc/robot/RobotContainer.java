// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.commands.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverPort);
    private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    // public final IntakeSubsystem intake = new IntakeSubsystem();
    public final VisionSubsystem vision = new VisionSubsystem();

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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
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

        driver.y().whileTrue(new AutoAlignCommand(drivetrain, vision));
    }

    private void configureOperatorBindings() {

        // operator.rightTrigger().whileTrue(new InstantCommand(() -> intake.intakeCoral(.9)));
        // operator.leftTrigger().whileTrue(new InstantCommand(() -> intake.intakeCoral(-.7)));

        // operator.rightTrigger().whileFalse(new InstantCommand(() -> intake.intakeCoral(0)));
        // operator.leftTrigger().whileFalse(new InstantCommand(() -> intake.intakeCoral(0)));

        elevator.setDefaultCommand(new RunCommand(() -> {
            double speed = operator.getRightY() / 5;
            if (Math.abs(speed) > 0.025) {
            elevator.setSpeed(speed);
            } else {
            elevator.setSpeed(0);
            }
        }, elevator));

        operator.leftBumper().whileTrue(new InstantCommand(() -> elevator.setSpeed(.05)));

        // operator.leftBumper().whileTrue(new InstantCommand(() -> intake.setPivotSpeed(.075)));
        // operator.rightBumper().whileTrue(new InstantCommand(() -> intake.setPivotSpeed(-.5)));

        // operator.leftBumper().whileFalse(new InstantCommand(() -> intake.stopPivot()));
        // operator.rightBumper().whileFalse(new InstantCommand(() -> intake.stopPivot()));


        operator.povDown().onTrue(new SequentialCommandGroup(
            // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L1))
            // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L1))
        ));

        operator.povRight().onTrue(new SequentialCommandGroup(
            // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L2))
            // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L2))
        ));

        operator.povLeft().onTrue(new SequentialCommandGroup(
            // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L3))
            // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L3))
        ));

        operator.povUp().onTrue(new SequentialCommandGroup(
            // new InstantCommand(() -> elevator.setElevatorHeight(ElevatorHeight.L4))
            // new InstantCommand(() -> intake.setPivotAngle(IntakeSubsystem.IntakeAngle.L4))
        ));

    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        //return new PathPlannerAuto("Reefscape Auton");
        // return autoChooser.getSelected();
        // return new PathPlannerAuto("Score L1 Straight");
        return new PathPlannerAuto("Straight");

    }
}
