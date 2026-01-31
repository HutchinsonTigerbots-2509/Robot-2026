// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.autonomous.Pathplanner;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    private static double MaxSpeed = 1.0 * DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final static SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final Drivetrain sDrivetrain = DrivetrainConstants.createDrivetrain();

    public static boolean fieldOriented = true;

    public static SlewRateLimiter Slewer1 = new SlewRateLimiter(2.0);  // Creates our Slew Limiter which makes our drivetrain slowly accelorate, Please Ignore spelling <3
    public static SlewRateLimiter Slewer2 = new SlewRateLimiter(2.0);

    public RobotContainer() {
        configureBindings();

        // boolean isCompetition = false; // Change this to true when at a competition

        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        //     (stream) -> isCompetition
        //     ? stream.filter(auto -> auto.getName().startsWith("comp"))
        //     : stream
        // );

        // SmartDashboard.putData("Auto Chooser", autoChooser);
        // autoChooser = AutoBuilder.buildAutoChooser();

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // sDrivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     sDrivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        sDrivetrain.setDefaultCommand(
            sDrivetrain.applyRequest(() ->  
                drive.withVelocityX((Slewer1.calculate(calculateFieldX(joystick)) * MaxSpeed) * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY((Slewer2.calculate(calculateFieldY(joystick)) * MaxSpeed) * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        ); // Drive counterclockwise with negative X (left)

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            sDrivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

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

        // joystick.leftTrigger().onTrue(new RunCommand(() -> Shooter.shootpositive1())).onFalse(new InstantCommand(() -> Shooter.shootzero()));
        // joystick.rightTrigger().onTrue(new RunCommand(() -> Shooter.shootpositive95())).onFalse(new InstantCommand(() -> Shooter.shootzero()));

        // // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(new RunCommand(() -> Shooter.shootpositive875())).onFalse(new InstantCommand(() -> Shooter.shootzero()));
        // joystick.rightBumper().onTrue(new RunCommand(() -> Shooter.shootpositive8())).onFalse(new InstantCommand(() -> Shooter.shootzero()));

        // joystick.a().whileTrue(new RunCommand(() -> Shooter.shootpositive5())).onFalse(new InstantCommand(() -> Shooter.shootzero()));
        // joystick.b().whileTrue(new RunCommand(() -> Shooter.shootpositive575())).onFalse(new InstantCommand(() -> Shooter.shootzero()));
        // joystick.x().whileTrue(new RunCommand(() -> Shooter.shootpositive65())).onFalse(new InstantCommand(() -> Shooter.shootzero()));
        // joystick.y().whileTrue(new RunCommand(() -> Shooter.shootpositive725())).onFalse(new InstantCommand(() -> Intake.intakezero()));
        // // joystick.y().onTrue(new RunCommand(() -> Shooter.shootpositive725())).onFalse(new InstantCommand(() -> Intake.intakezero()));

        // // joystick.a().whileTrue(new RunCommand(() -> Shooter.shoot())).onFalse(new InstantCommand(() -> Shooter.shootzero()));
        // // joystick.x().onTrue(new InstantCommand(() -> Shooter.shootincrement()));
        // // joystick.b().onTrue(new InstantCommand(() -> Shooter.shootresetincrement()));
        // joystick.x().whileTrue(new RunCommand(() -> Intake.intakepositive())).onFalse(new InstantCommand(() -> Intake.intakezero()));
        joystick.leftBumper().whileTrue(new RunCommand(() -> Intake.intakenegative25())).onFalse(new InstantCommand(() -> Intake.intakezero()));
        joystick.rightBumper().whileTrue(new RunCommand(() -> Intake.intakenegative5())).onFalse(new InstantCommand(() -> Intake.intakezero()));
        joystick.leftTrigger().whileTrue(new RunCommand(() -> Intake.intakenegative75())).onFalse(new InstantCommand(() -> Intake.intakezero()));
        joystick.rightTrigger().whileTrue(new RunCommand(() -> Intake.intakenegative1())).onFalse(new InstantCommand(() -> Intake.intakezero()));

        sDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public static double calculateFieldX(CommandXboxController controller) {

        double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() /*+ fieldOffset*/);
        //SmartDashboard.putNumber("gyro", gyro);
        double cos = Math.cos(gyro);
        //SmartDashboard.putNumber("Cos", cos);
        double sin = Math.sin(gyro);
        //SmartDashboard.putNumber("Sin", sin);

        double tX = -controller.getLeftY(); // The X in FRC means forwards and backwards
        double tY = -controller.getLeftX();

        double fieldX = ((tX * cos) + (tY * sin));

        return fieldX;
    }

    public static double calculateFieldY(CommandXboxController controller) {

        double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() /*+ fieldOffset*/);
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);

        double tX = -controller.getLeftY(); // The X in FRC means forwards and backwards
        double tY = -controller.getLeftX();

        double fieldY = ((tY  * cos) - (tX * sin));
            
        //SmartDashboard.putNumber("fieldY", fieldY);

        return fieldY;
    }

    // VVVV Generated autonomous VVVV

    // public Command getAutonomousCommand() {
    //     // Simple drive forward auton
    //     final var idle = new SwerveRequest.Idle();
    //     return Commands.sequence(
    //         // Reset our field centric heading to match the robot
    //         // facing away from our alliance station wall (0 deg).
    //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //         // Then slowly drive forward (away from us) for 5 seconds.
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(0.5)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //         )
    //         .withTimeout(5.0),
    //         // Finally idle for the rest of auton
    //         drivetrain.applyRequest(() -> idle)
    //     );
    // }

    
    //              -----Autonomous-----            

    public static final Pathplanner sPathPlanner = new Pathplanner(sDrivetrain);
    public static SwerveDrivePoseEstimator eSwerveEstimator;
    
    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }
    
    public static void PathplannerDriveSwerve(ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double vOmega = speeds.omegaRadiansPerSecond;
        if (DriverStation.isAutonomous()) {
            sDrivetrain.applyRequest(() -> drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vOmega));
        }   
    }
    
    public Command getAutonomousCommand() {
        // return Pathplanner.getAutonomousCommand();
        return new PathPlannerAuto("TestAuto");
    }
    
    public static Pose2d getPose() {
        Pose2d pos0 = eSwerveEstimator.getEstimatedPosition();
        return pos0;
    }

    public static void resetPose(Pose2d pos1) {
        sDrivetrain.resetPose(pos1);
        setGyro(pos1.getRotation().getDegrees());
        eSwerveEstimator.resetPose(pos1);  
    }
    
    public static ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds speed1;
        speed1 = sDrivetrain.getKinematics().toChassisSpeeds(getModuleStates());
        return speed1;
    }
            
    public static void driveRobotRelative(ChassisSpeeds speed2) {
        // RobotContainer.PathplannerDriveSwerve(speed2);
    }
            
    public static void setGyro(Double pos2) {
        sDrivetrain.getPigeon2().setYaw(pos2);
    }
        
    public SwerveDriveKinematics getKinematics() {
        SwerveDriveKinematics kinematics = sDrivetrain.getKinematics();
        return kinematics;
    }
        
    public static Rotation2d getRotation2d() {
        Rotation2d pos3 = sDrivetrain.getPigeon2().getRotation2d();
        return pos3;
    }
        
    //** Returns the List of SwerveModulePositions in ( F:LR  R:LR ) order */
    public static SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition FL = sDrivetrain.getModule(0).getPosition(false);
        SwerveModulePosition FR = sDrivetrain.getModule(1).getPosition(false);
        SwerveModulePosition RL = sDrivetrain.getModule(2).getPosition(false);
        SwerveModulePosition RR = sDrivetrain.getModule(3).getPosition(false);
        SwerveModulePosition[] ModPositions = new SwerveModulePosition[] {FL, FR, RL, RR};
        return ModPositions;
    }
        
    //** Returns the list of SwerveModuleStates in ( F:LR  R:LR ) order */
    public static SwerveModuleState[] getModuleStates() {
        SwerveModuleState FL = sDrivetrain.getModule(0).getCurrentState();
        SwerveModuleState FR = sDrivetrain.getModule(1).getCurrentState();
        SwerveModuleState RL = sDrivetrain.getModule(2).getCurrentState();
        SwerveModuleState RR = sDrivetrain.getModule(3).getCurrentState();
        SwerveModuleState[] ModStates = new SwerveModuleState[] {FL, FR, RL, RR};
        return ModStates;
    }

}
