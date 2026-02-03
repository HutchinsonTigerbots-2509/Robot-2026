// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.autonomous.Pathplanner;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private static double MaxSpeed = 1.0 * DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick ButtonBoardA = new Joystick(1);
    private final Joystick ButtonBoardB = new Joystick(2);
    private final JoystickButton A0 = new JoystickButton(ButtonBoardA, 0); //TODO: Map out rest of button board. Also, can just use another controller

    private static final Drivetrain sDrivetrain = DrivetrainConstants.createDrivetrain();
    private static final Pathplanner sPathPlanner = new Pathplanner(sDrivetrain);
    private static final Climber sClimber = new Climber();
    private static final Feeder sFeeder = new Feeder();
    private static final Intake sIntake = new Intake();
    private static final Shooter sShooter = new Shooter();
    private static final Vision sVision = new Vision();

    public static SwerveDrivePoseEstimator eSwerveEstimator;
    private static SendableChooser<Command> autoSelect = new SendableChooser<Command>();
    private static final boolean isCompetition = false; // Change this to true when at a competition!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    private static SlewRateLimiter Slewer1 = new SlewRateLimiter(2.0);
    private static SlewRateLimiter Slewer2 = new SlewRateLimiter(2.0);

    private static Field2d field2d = new Field2d();
    // private static boolean allianceBlue = DriverStation.MatchDataSender();  //TODO: I don't know how reliable this is.

    public RobotContainer() {
        configureBindings();
        SmartDashboard.putData("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {
        // sDrivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     sDrivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // To get rid of chattering while stationary we should set minimum inputs where the robot will idle if under those.
        sDrivetrain.setDefaultCommand(
            sDrivetrain.applyRequest(() ->  
                drive.withVelocityX((Slewer1.calculate(calculateFieldX(joystick)) * MaxSpeed) * 0.5)
                    .withVelocityY((Slewer2.calculate(calculateFieldY(joystick)) * MaxSpeed) * 0.5)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            sDrivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.leftTrigger().onTrue(new InstantCommand(() -> sShooter.shootUnload(sFeeder, sVision)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> sShooter.shootCancel(sFeeder)));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> sIntake.intakeForward()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> sIntake.intakeZero()));

        // VVVV Generated bindings VVVV

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // joystick.a().whileTrue(new RunCommand(() -> sShooter.shootnum())).onFalse(new InstantCommand(() -> sShooter.shootZero()));
        // joystick.x().onTrue(new InstantCommand(() -> sShooter.shootincrement()));
        // joystick.b().onTrue(new InstantCommand(() -> sShooter.shootresetincrement()));
        // joystick.y().whileTrue(new InstantCommand(() -> sShooter.shootrecall()));

        // joystick.a().whileTrue(new RunCommand(() -> sIntake.intakenum())).onFalse(new InstantCommand(() -> sIntake.intakeZero()));
        // joystick.x().onTrue(new InstantCommand(() -> sIntake.intakeincrement()));
        // joystick.b().onTrue(new InstantCommand(() -> sIntake.intakeresetnum()));
        // joystick.y().whileTrue(new InstantCommand(() -> sIntake.intakerecall()));

        sDrivetrain.registerTelemetry(logger::telemeterize); //TODO: Might also be the cause of the signal logger still going
    }

    public static void driveVision(double vx, double vy, double vOmega) {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vOmega));
    }

    public static void driveIdle() {
        final var idle = new SwerveRequest.Idle();
        sDrivetrain.applyRequest(() -> idle);
    }

    public static void driveBrake() {
        sDrivetrain.applyRequest(() -> brake);
    }

    public static double calculateFieldX(CommandXboxController controller) {
        double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() /*+ fieldOffset*/);
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);
        double tX = -controller.getLeftY();
        double tY = -controller.getLeftX();
        double fieldX = ((tX * cos) + (tY * sin));
        return fieldX;
    }

    public static double calculateFieldY(CommandXboxController controller) {
        double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() /*+ fieldOffset*/);
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);
        double tX = -controller.getLeftY();
        double tY = -controller.getLeftX();
        double fieldY = ((tY  * cos) - (tX * sin));
        return fieldY;
    }

    // VVVV Generated autonomous VVVV

    // public Command getAutonomousCommand() {
    //     // Simple drive forward auton
    //     final var idle = new SwerveRequest.Idle();
    //     return Commands.sequence(
    //         // Reset our field centric heading to match the robot
    //         // facing away from our alliance station wall (0 deg).
    //         sDrivetrain.runOnce(() -> sDrivetrain.seedFieldCentric(Rotation2d.kZero)),
    //         // Then slowly drive forward (away from us) for 5 seconds.
    //         sDrivetrain.applyRequest(() ->
    //             drive.withVelocityX(0.5)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //         )
    //         .withTimeout(5.0),
    //         // Finally idle for the rest of auton
    //         sDrivetrain.applyRequest(() -> idle)
    //     );
    // }       
    
    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }
    
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("TestAuto");
    }
    
    public static Pose2d getPose() {
        Pose2d pos = eSwerveEstimator.getEstimatedPosition();
        return pos;
    }

    public static void resetPose(Pose2d pos) {
        sDrivetrain.resetPose(pos);
        setGyro(pos.getRotation().getDegrees());
        eSwerveEstimator.resetPose(pos);  
    }
    
    public static ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds speeds;
        speeds = sDrivetrain.getKinematics().toChassisSpeeds(getModuleStates());
        return speeds;
    }
            
    public static void driveRobotRelative(ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double vOmega = speeds.omegaRadiansPerSecond;
        if (DriverStation.isAutonomous()) {
            sDrivetrain.applyRequest(() -> drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vOmega));
        }
    }
            
    public static void setGyro(Double pos) {
        sDrivetrain.getPigeon2().setYaw(pos);
    }
        
    public static SwerveDriveKinematics getKinematics() {
        SwerveDriveKinematics kinematics = sDrivetrain.getKinematics();
        return kinematics;
    }
        
    public static Rotation2d getRotation2d() {
        Rotation2d pos3 = sDrivetrain.getPigeon2().getRotation2d();
        return pos3;
    }
        
    public static SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition FL = sDrivetrain.getModule(0).getPosition(false);
        SwerveModulePosition FR = sDrivetrain.getModule(1).getPosition(false);
        SwerveModulePosition RL = sDrivetrain.getModule(2).getPosition(false);
        SwerveModulePosition RR = sDrivetrain.getModule(3).getPosition(false);
        SwerveModulePosition[] ModPositions = new SwerveModulePosition[] {FL, FR, RL, RR};
        return ModPositions;
    }
        
    public static SwerveModuleState[] getModuleStates() {
        SwerveModuleState FL = sDrivetrain.getModule(0).getCurrentState();
        SwerveModuleState FR = sDrivetrain.getModule(1).getCurrentState();
        SwerveModuleState RL = sDrivetrain.getModule(2).getCurrentState();
        SwerveModuleState RR = sDrivetrain.getModule(3).getCurrentState();
        SwerveModuleState[] ModStates = new SwerveModuleState[] {FL, FR, RL, RR};
        return ModStates;
    }

    public static void buildAutoChooser() {
        autoSelect.setDefaultOption("Do Nothing", AutoBuilder.buildAuto("Do Nothing"));
        List<String> options = AutoBuilder.getAllAutoNames();
        if (isCompetition) {
            for (String n : options) {
                if(n.startsWith("c") && n != "Do Nothing")
                autoSelect.addOption(n, AutoBuilder.buildAuto(n));
            };
        }
        else {
            for (String n : options) {
                if(n != "Do Nothing")
                autoSelect.addOption(n, AutoBuilder.buildAuto(n));
            };
        }
    }
}
