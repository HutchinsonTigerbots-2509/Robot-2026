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
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.autonomous.Pathplanner;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.feederhopper.FeederHopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private static double MaxSpeed = 1.0 * DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final static SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick ButtonBoardA = new Joystick(1);
    private final Joystick ButtonBoardB = new Joystick(2);
    private final JoystickButton A1 = new JoystickButton(ButtonBoardA, 1);
    private final JoystickButton A2 = new JoystickButton(ButtonBoardA, 2);
    private final JoystickButton A3 = new JoystickButton(ButtonBoardA, 3);
    private final JoystickButton A4 = new JoystickButton(ButtonBoardA, 4);
    private final JoystickButton A5 = new JoystickButton(ButtonBoardA, 5);
    private final JoystickButton A6 = new JoystickButton(ButtonBoardA, 6);
    private final JoystickButton A7 = new JoystickButton(ButtonBoardA, 7);
    private final JoystickButton A8 = new JoystickButton(ButtonBoardA, 8);
    private final JoystickButton A9 = new JoystickButton(ButtonBoardA, 9);
    private final JoystickButton A10 = new JoystickButton(ButtonBoardA, 10);
    private final JoystickButton A11 = new JoystickButton(ButtonBoardA, 11);
    private final JoystickButton A12 = new JoystickButton(ButtonBoardA, 12);
    private final JoystickButton B10 = new JoystickButton(ButtonBoardB, 10);
    private final JoystickButton B11 = new JoystickButton(ButtonBoardB, 11);
    private final JoystickButton B12 = new JoystickButton(ButtonBoardB, 12);

    public static final Drivetrain sDrivetrain = DrivetrainConstants.createDrivetrain();
    private static final Climber sClimber = new Climber();
    private static final FeederHopper sFeederHopper = new FeederHopper();
    private static final Intake sIntake = new Intake();
    private static final Shooter sShooter = new Shooter();
    private static final Vision sVision = new Vision();
    private static final Pathplanner sPathPlanner = new Pathplanner(sDrivetrain);

    public static SwerveDrivePoseEstimator eSwerveEstimator;
    public static Pose2d eVisionPose2d;
    private static SendableChooser<Command> autoSelect = new SendableChooser<Command>();
    private static final boolean isCompetition = false; // Change this to true when at a competition!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    private static SlewRateLimiter Slewer1 = new SlewRateLimiter(2.0);
    private static SlewRateLimiter Slewer2 = new SlewRateLimiter(2.0);

    public static Field2d field2d = new Field2d();
    public static double fieldOffset = 0;
    private static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    public static boolean magicBool;
    public static double magicNum;

    public static double turn1;
    public double turn2;

    public RobotContainer() {
        configureBindings();
        buildAutoChooser();
        ApplyStart();
        SmartDashboard.putData("Auto Chooser", AutoBuilder.buildAutoChooser());
        SmartDashboard.putData(autoSelect);
        SmartDashboard.putNumber("MaxSpeed", MaxSpeed);
        SmartDashboard.putNumber("MaxAngularRate", MaxAngularRate);
        SmartDashboard.putNumber("joystickStrafeY", calculateFieldY(joystick) * MaxSpeed * 0.1);
    }

    private void configureBindings() {
        sDrivetrain.setDefaultCommand(
            sDrivetrain.applyRequest(() ->  
                drive.withVelocityX((Slewer1.calculate(calculateFieldX(joystick)) * MaxSpeed))
                    .withVelocityY((Slewer2.calculate(calculateFieldY(joystick)) * MaxSpeed))
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            sDrivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.povUp().onTrue(new InstantCommand(() -> setGyro(0.0)).andThen(new InstantCommand(() -> System.out.println("Yaw: " + sDrivetrain.getPigeon2().getYaw()))));

        // joystick.leftTrigger().whileTrue(new ParallelCommandGroup(new RunCommand(() -> sClimber.climb2()), new RunCommand(() -> sVision.visionTurnShoot()), new RunCommand(() -> sShooter.shootNumMethod()), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeederHopper.feedZero()).until(() -> sShooter.eShooter.get() < -200000).andThen(new RunCommand(() -> sFeederHopper.feederNumMethod()))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sClimber.climbZero()), new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeederHopper.feedzero())));
        // joystick.leftTrigger().whileTrue(new ParallelCommandGroup(new RunCommand(() -> sClimber.climb2()), new RunCommand(() -> sVision.visionTurnShoot()), new RunCommand(() -> sShooter.shootNumMethod(sVision.distanceToShootingSpeed())), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeederHopper.feedZero()).until(() -> sShooter.eShooter.get() < -200000).andThen(new RunCommand(() -> sFeederHopper.feederNumMethod()))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sClimber.climbZero()), new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeederHopper.feedzero())));
        joystick.leftTrigger().whileTrue(new ParallelCommandGroup(new RunCommand(() -> sFeederHopper.hopperOn()), new RunCommand(() -> strafeVision()), new RunCommand(() -> sShooter.shootNumMethod(sVision.distanceToShootingSpeed())), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeederHopper.feedZero()).until(() -> sShooter.eShooter.get() < -70000).andThen(new RunCommand(() -> sFeederHopper.feederNumMethod()))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sFeederHopper.hopperOff()), new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeederHopper.feedzero())));
        // joystick.leftTrigger().whileTrue(new ParallelCommandGroup(new RunCommand(() -> sClimber.climb2()), new RunCommand(() -> sVision.visionTurnShoot()), new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootNumMethod(-20)), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new InstantCommand(() -> sFeederHopper.feedToReverse()))).until(() -> sShooter.eShooter.get() > 100).andThen(new RunCommand(() -> sShooter.shootNumMethod(sVision.distanceToShootingSpeed())), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeederHopper.feedZero()).until(() -> sShooter.eShooter.get() < -200000).andThen(new RunCommand(() -> sFeederHopper.feederNumMethod())))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sClimber.climbZero()), new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeederHopper.feedzero())));
        // joystick.leftTrigger().whileTrue(new RunCommand(() -> sVision.visionTurnShoot()).unless(() -> sVision.correctAnglePos()).andThen(new ParallelCommandGroup(new RunCommand(() -> sShooter.shootNumMethod()), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeederHopper.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeederHopper.feederNumMethod())))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeederHopper.feedzero())));
        // A1.whileTrue(new ParallelCommandGroup(new RunCommand(() -> sClimber.climb2()), new RunCommand(() -> sShooter.shootNumMethod()), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeederHopper.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeederHopper.feederNumMethod()))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sClimber.climbZero()), new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeederHopper.feedzero())));

        RunCommand creepDrive = new RunCommand(() -> driveControllerCreep());
        creepDrive.addRequirements(sDrivetrain);
        joystick.leftBumper().toggleOnTrue(creepDrive);

        RunCommand intakeDrive = new RunCommand(() -> driveControllerCreep());
        intakeDrive.addRequirements(sDrivetrain);
        joystick.rightBumper().toggleOnTrue(new ParallelCommandGroup(intakeDrive, new RunCommand(() -> sIntake.intakeNumMethod()))).toggleOnFalse(new InstantCommand(() -> sIntake.intakeZero()));

        joystick.y().whileTrue(new RunCommand(() -> sIntake.liftIn()).until(() -> sIntake.eLift.get() < -750).andThen(new InstantCommand(() -> sIntake.liftZero()))).onFalse(new InstantCommand(() -> sIntake.liftZero()));
        joystick.a().whileTrue(new RunCommand(() -> sIntake.liftOut()).until(() -> !sIntake.wLiftMax.get()).andThen(new InstantCommand(() -> sIntake.liftZero())).andThen(new InstantCommand(() -> sIntake.eLift.reset())).andThen(new InstantCommand(() -> sIntake.modLiftCycle()))).onFalse(new InstantCommand(() -> sIntake.liftZero()));
        joystick.b().whileTrue(new RunCommand(() -> sClimber.climb2())).onFalse(new InstantCommand(() -> sClimber.climbZero()));
        joystick.x().whileTrue(new RunCommand(() -> sClimber.climbDown())).onFalse(new InstantCommand(() -> sClimber.climbZero()));

        A1.whileTrue(new InstantCommand(() -> System.out.println("A1")));
        A2.whileTrue(new InstantCommand(() -> System.out.println("A2")));
        A3.whileTrue(new InstantCommand(() -> System.out.println("A3")));
        A4.whileTrue(new InstantCommand(() -> System.out.println("A4")));
        A5.whileTrue(new InstantCommand(() -> System.out.println("A5")));
        A6.whileTrue(new InstantCommand(() -> System.out.println("A6")));
        A7.whileTrue(new InstantCommand(() -> System.out.println("A7")));
        A8.whileTrue(new InstantCommand(() -> System.out.println("A8")));
        A9.whileTrue(new InstantCommand(() -> System.out.println("A9")));
        A10.whileTrue(new InstantCommand(() -> System.out.println("A10")));
        A11.whileTrue(new InstantCommand(() -> System.out.println("A11")));
        A12.whileTrue(new InstantCommand(() -> System.out.println("A12")));
        B10.whileTrue(new InstantCommand(() -> System.out.println("B10")));
        B11.whileTrue(new InstantCommand(() -> System.out.println("B11")));
        B12.whileTrue(new InstantCommand(() -> System.out.println("B12")));

        sDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public static void driveVision(double vx, double vy, double vOmega) {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vOmega)).execute();
    }

    public void strafeVision() {
        turn2 = turn1;
        sDrivetrain.applyRequest(() -> drive.withVelocityX(calculateFieldX(joystick) * MaxSpeed * 0.1).withVelocityY(calculateFieldY(joystick) * MaxSpeed * 0.1).withRotationalRate(strafeVisionOffset())).execute();
        SmartDashboard.putNumber("joystickStrafeY", calculateFieldY(joystick) * MaxSpeed * 0.1);
    }

    public double strafeVisionOffset() {
        if ((calculateFieldY(joystick) * MaxSpeed * 0.1) < -0.25) {
            return turn2 - 1.5;
        } else if ((calculateFieldY(joystick) * MaxSpeed * 0.1) > 0.25) {
            return turn2 + 1.5;
        } else {
            return turn2;
        }
    }

    public static void driveIdle() {
        final var idle = new SwerveRequest.Idle();
        sDrivetrain.applyRequest(() -> idle).execute();
    }

    public static void driveBrake() {
        sDrivetrain.applyRequest(() -> brake).execute();
    }

    public static double getMaxSpeed() {
        return MaxSpeed;
    }

    public static double getMaxAngularRate() {
        return MaxAngularRate;
    }

    public static boolean getAllianceBlue() {
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                return true;
            } else if (alliance.get() == DriverStation.Alliance.Red) {
                return false;
            }
        }
        return magicBool;
    }

    public static double calculateFieldX(CommandXboxController controller) {
        double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() + fieldOffset);
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);
        double tX = -controller.getLeftY();
        double tY = -controller.getLeftX();
        double fieldX = ((tX * cos) + (tY * sin));
        return fieldX;
    }

    public static double calculateFieldY(CommandXboxController controller) {
        double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() + fieldOffset);
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);
        double tX = -controller.getLeftY();
        double tY = -controller.getLeftX();
        double fieldY = ((tY  * cos) - (tX * sin));
        return fieldY;
    }

    public void driveControllerCreep() {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(calculateFieldX(joystick) * MaxSpeed * 0.2)                                                                               
                .withVelocityY(calculateFieldY(joystick) * MaxSpeed * 0.2)
                .withRotationalRate((-joystick.getRightX() * MaxAngularRate) * 0.8))
                .execute();
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
    
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Start1");
    }

    // public Command getAutonomousCommand() {
    //     return autoSelect.getSelected();
    // }
    
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
            sDrivetrain.applyRequest(() -> drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vOmega)).execute();
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

    public static SendableChooser<Command> getSelection() {
        return autoSelect;
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

    public static void ApplyStart() {
        String autoName = autoSelect.getSelected().getName();
        try {
            AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
            // Pathplanner.startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
        } catch (Exception e) {
            e.printStackTrace();
            // Pathplanner.startPose2d = new Pose2d(0, 0, new Rotation2d(0));
        }
    };

    public Pathplanner getsPathplanner() {
        return sPathPlanner;
    }

    public static void namedCommands() {

        NamedCommands.registerCommand("IntakeOut", new RunCommand(() -> sIntake.liftOut()).until(() -> !sIntake.wLiftMax.get()).andThen(new InstantCommand(() -> sIntake.liftZero())).andThen(new InstantCommand(() -> sIntake.eLift.reset())).andThen(new InstantCommand(() -> sIntake.modLiftCycle())));
        // NamedCommands.registerCommand("IntakeRun", new RunCommand(() -> sIntake.intakeNumMethod()));
        NamedCommands.registerCommand("IntakeZero", new RunCommand(() -> sIntake.intakeZero()));
        NamedCommands.registerCommand("IntakeRun", new RunCommand(() -> sIntake.intakeNumMethod()).withTimeout(5).andThen(new RunCommand(() -> sIntake.intakeZero())));

    }
}
