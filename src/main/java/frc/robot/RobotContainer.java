// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.autonomous.Pathplanner;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
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
    public static final CommandXboxController joystick1 = new CommandXboxController(0);
    private final Joystick ButtonBoardA = new Joystick(1);
    private final Joystick ButtonBoardB = new Joystick(2);
    private final JoystickButton A1 = new JoystickButton(ButtonBoardA, 1); // Shootpos 4
    private final JoystickButton A2 = new JoystickButton(ButtonBoardA, 2); // Shootpos 3
    private final JoystickButton A3 = new JoystickButton(ButtonBoardA, 3); // Shootpos 2
    private final JoystickButton A4 = new JoystickButton(ButtonBoardA, 4); // Shootpos 1
    private final JoystickButton A5 = new JoystickButton(ButtonBoardA, 5); // Shoot+Feed backward
    private final JoystickButton A6 = new JoystickButton(ButtonBoardA, 6); // Lift in
    private final JoystickButton A7 = new JoystickButton(ButtonBoardA, 7); // Lift out
    private final JoystickButton A8 = new JoystickButton(ButtonBoardA, 8); // Intake in
    private final JoystickButton B1 = new JoystickButton(ButtonBoardB, 1); // Lift in emergency
    private final JoystickButton B2 = new JoystickButton(ButtonBoardB, 2); // Unused
    private final JoystickButton B3 = new JoystickButton(ButtonBoardB, 3); // Unused
    private final JoystickButton B4 = new JoystickButton(ButtonBoardB, 4); // Unused
    private final JoystickButton B5 = new JoystickButton(ButtonBoardB, 5); // Intake out
    private final JoystickButton B6 = new JoystickButton(ButtonBoardB, 6); // Unused
    private final JoystickButton B7 = new JoystickButton(ButtonBoardB, 7); // Unused

    public static final Drivetrain sDrivetrain = DrivetrainConstants.createDrivetrain();
    public static final Feeder sFeeder = new Feeder();
    public static final Intake sIntake = new Intake();
    public static final Lift sLift = new Lift();
    public static final Shooter sShooter = new Shooter();
    public static final Vision sVision = new Vision();
    public static final Pathplanner sPathPlanner = new Pathplanner(sDrivetrain);

    public static SwerveDrivePoseEstimator eSwerveEstimator;
    public static Pose2d eVisionPose2d;
    public static SendableChooser<Command> autoSelect = new SendableChooser<Command>();

    private static SlewRateLimiter Slewer1 = new SlewRateLimiter(2.0);
    private static SlewRateLimiter Slewer2 = new SlewRateLimiter(2.0);

    public static Field2d field2d = new Field2d();

    public static boolean orientationSource = true;

    public static double kLiftShootPos = 350;
    public static double kLiftMaxPos = 600;

    public RobotContainer() {
        configureBindings();
        buildAutoChooser();
        SmartDashboard.clearPersistent("Auto Chooser");
        SmartDashboard.putData("Auto Chooser", autoSelect);
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

        joystick.povUp().onTrue(new InstantCommand(() -> setOrientationSource()));

        joystick.leftTrigger().whileTrue(new ParallelCommandGroup(new RunCommand(() -> strafeVision()), new RunCommand(() -> sShooter.shootVariable(sVision.shootingSpeed())), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable(-80)))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        joystick.rightTrigger().whileTrue(new RunCommand(() -> sLift.liftInFast()).until(() -> sLift.eLift.get() > kLiftShootPos).andThen(new InstantCommand(() -> sLift.liftZero()))).onFalse(new RunCommand(() -> sLift.liftOutFast()).until(() -> !sLift.wLiftMax.get()).andThen(new InstantCommand(() -> sLift.liftZero())).andThen(new InstantCommand(() -> sLift.eLift.reset())).andThen(new InstantCommand(() -> sLift.modLiftCycle())));

        RunCommand creepDrive = new RunCommand(() -> driveControllerCreep());
        creepDrive.addRequirements(sDrivetrain);
        joystick.leftBumper().toggleOnTrue(creepDrive);

        RunCommand intakeDrive = new RunCommand(() -> driveControllerCreep());
        intakeDrive.addRequirements(sDrivetrain);
        joystick.rightBumper().toggleOnTrue(new ParallelCommandGroup(intakeDrive, new RunCommand(() -> sIntake.intakeForward()))).toggleOnFalse(new InstantCommand(() -> sIntake.intakeZero()));
        
        A1.whileTrue(new ParallelCommandGroup(new RunCommand(() -> sShooter.shootVariable(53)), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable1(-80)))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        A2.whileTrue(new ParallelCommandGroup(new RunCommand(() -> sShooter.shootVariable(75)), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable1(-80)))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        A3.whileTrue(new ParallelCommandGroup(new RunCommand(() -> sShooter.shootVariable(45)), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable1(-80)))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        A4.whileTrue(new ParallelCommandGroup(new RunCommand(() -> sShooter.shootVariable(51)), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable1(-80)))))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        A5.whileTrue(new ParallelCommandGroup(new RunCommand(() -> sShooter.shootVariable(-63)), new RunCommand(() -> sFeeder.feedReverse()))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        A6.whileTrue(new RunCommand(() -> sLift.liftIn()).until(() -> sLift.eLift.get() > kLiftMaxPos).andThen(new InstantCommand(() -> sLift.liftZero()))).onFalse(new InstantCommand(() -> sLift.liftZero()));
        A7.whileTrue(new RunCommand(() -> sLift.liftOut()).until(() -> !sLift.wLiftMax.get()).andThen(new InstantCommand(() -> sLift.liftZero())).andThen(new InstantCommand(() -> sLift.eLift.reset())).andThen(new InstantCommand(() -> sLift.modLiftCycle()))).onFalse(new InstantCommand(() -> sLift.liftZero()));
        A8.whileTrue(new RunCommand(() -> sIntake.intakeForward())).onFalse(new InstantCommand(() -> sIntake.intakeZero()));
        B1.whileTrue(new RunCommand(() -> sLift.liftInEmergency())).onFalse(new InstantCommand(() -> sLift.liftZero()));
        B2.whileTrue(new InstantCommand(() -> System.out.println("B2")));
        B3.whileTrue(new InstantCommand(() -> System.out.println("B3")));
        B4.whileTrue(new InstantCommand(() -> System.out.println("B4")));
        B5.whileTrue(new RunCommand(() -> sIntake.intakeReverse())).onFalse(new InstantCommand(() -> sIntake.intakeZero()));
        B6.whileTrue(new InstantCommand(() -> System.out.println("B6")));
        B7.whileTrue(new InstantCommand(() -> System.out.println("B7")));
        B7.onTrue(NamedCommands.getCommand("Print"));

        sDrivetrain.registerTelemetry(logger::telemeterize);
    }

    public Drivetrain getDrivetrain() {
        return sDrivetrain;
    }

    public Feeder getFeeder() {
        return sFeeder;
    }

    public Intake getIntake() {
        return sIntake;
    }

    public Lift getLift() {
        return sLift;
    }

    public Shooter getShooter() {
        return sShooter;
    }

    public Vision getVision() {
        return sVision;
    }

    public Pathplanner getPathplanner() {
        return sPathPlanner;
    }

    public static void driveVision(double vx, double vy, double vOmega) {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(vOmega)).execute();
    }

    public void strafeVision() {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(calculateFieldX(joystick) * MaxSpeed * 0.1).withVelocityY(calculateFieldY(joystick) * MaxSpeed * 0.1).withRotationalRate(strafeVisionOffset())).execute();
    }

    public double strafeVisionOffset() {
        if (Math.abs(joystick.getRightX()) > 0.25) {
            return -joystick.getRightX() * MaxAngularRate;
        } else {
            if ((calculateFieldY(joystick) * MaxSpeed * 0.1) < -0.25) {
                return sVision.turnStrafe - 1.5;
            } else if ((calculateFieldY(joystick) * MaxSpeed * 0.1) > 0.25) {
                return sVision.turnStrafe + 1.5;
            } else {
                return sVision.turnStationary;
            }
        }
    }

    public static double getMaxSpeed() {
        return MaxSpeed;
    }

    public static double getMaxAngularRate() {
        return MaxAngularRate;
    }

    public static double calculateFieldX(CommandXboxController controller) {
        double gyro = getOrientationSource();
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);
        double tX = -controller.getLeftY();
        double tY = -controller.getLeftX();
        double fieldX = ((tX * cos) + (tY * sin));
        return fieldX;
    }

    public static double calculateFieldY(CommandXboxController controller) {
        double gyro = getOrientationSource();
        double cos = Math.cos(gyro);
        double sin = Math.sin(gyro);
        double tX = -controller.getLeftY();
        double tY = -controller.getLeftX();
        double fieldY = ((tY  * cos) - (tX * sin));
        return fieldY;
    }

    public static double getOrientationSource() {
        if (orientationSource) {
            if (sVision.allianceCool()) {
                return getPose().getRotation().getRadians();
            } else {
                return getPose().getRotation().getRadians() + Math.PI;
            }
        } else {
            return Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble());
        }
    }

    public static void setOrientationSource() {
        orientationSource = false;
        setGyro(0.0);
        System.out.println("Yaw: " + sDrivetrain.getPigeon2().getYaw());
    }

    public void driveControllerCreep() {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(calculateFieldX(joystick) * MaxSpeed * 0.5)
                .withVelocityY(calculateFieldY(joystick) * MaxSpeed * 0.5)
                .withRotationalRate((-joystick.getRightX() * MaxAngularRate) * 0.8))
                .execute();
    }      

    public Command getAutonomousCommand() {
        return autoSelect.getSelected();
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

    public void buildAutoChooser() {
        autoSelect.setDefaultOption("Do Nothing", AutoBuilder.buildAuto("Do Nothing"));
        autoSelect.addOption("Right", AutoBuilder.buildAuto("Right"));
        autoSelect.addOption("Left", AutoBuilder.buildAuto("Left"));
        autoSelect.addOption("Potato", AutoBuilder.buildAuto("Potato"));
        autoSelect.addOption("IntakeTest", AutoBuilder.buildAuto("IntakeTest"));
    }

    public static void ApplyStart() {
        String autoName = autoSelect.getSelected().getName();
        try {
            AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
            Pathplanner.startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
    
        } catch (Exception e) {
            e.printStackTrace();
            Pathplanner.startPose2d = new Pose2d(0, 0, new Rotation2d(0));
        }
    };

    public static void namedCommands() {
        NamedCommands.registerCommand("LiftOut", new RunCommand(() -> sLift.liftOutFast()).until(() -> !sLift.wLiftMax.get()).andThen(new InstantCommand(() -> sLift.liftZero())).andThen(new InstantCommand(() -> sLift.eLift.reset())).andThen(new InstantCommand(() -> sLift.modLiftCycle())));
        NamedCommands.registerCommand("LiftIn450", new RunCommand(() -> sLift.liftZero()).withTimeout(2).andThen(new RunCommand(() -> sLift.liftIn()).until(() -> sLift.eLift.get() > kLiftShootPos)).andThen(new InstantCommand(() -> sLift.liftZero())));
        NamedCommands.registerCommand("ShootStart", new ParallelCommandGroup(new RunCommand(() -> sShooter.shootVariable(-63)), new RunCommand(() -> sFeeder.feedReverse())).withTimeout(1).andThen(new ParallelCommandGroup(new RunCommand(() -> driveVision(0.0, 0.0, sVision.turnStationary)), new RunCommand(() -> sShooter.shootVariable(sVision.shootingSpeed())), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable(-80)))))));
        NamedCommands.registerCommand("Shoot10", new ParallelCommandGroup(new RunCommand(() -> driveVision(0.0, 0.0, sVision.turnStationary)), new RunCommand(() -> sShooter.shootVariable(sVision.shootingSpeed())), new InstantCommand(() -> sShooter.eShooter.reset()).andThen(new RunCommand(() -> sFeeder.feedZero()).until(() -> sShooter.eShooter.get() < -80000).andThen(new RunCommand(() -> sFeeder.feedVariable(-80))))).withTimeout(6).andThen(new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero()))));
        NamedCommands.registerCommand("ShootStop", new ParallelCommandGroup(new InstantCommand(() -> sShooter.shootZero()), new InstantCommand(() -> sFeeder.feedZero())));
        NamedCommands.registerCommand("IntakeRun1", new RunCommand(() -> sIntake.intakeForward()).withTimeout(1).andThen(new InstantCommand(() -> sIntake.intakeZero())));
        NamedCommands.registerCommand("IntakeRun8", new RunCommand(() -> sIntake.intakeForward()).withTimeout(6).andThen(new InstantCommand(() -> sIntake.intakeZero())));
        NamedCommands.registerCommand("IntakeRun5", new RunCommand(() -> sIntake.intakeForward()).withTimeout(5).andThen(new InstantCommand(() -> sIntake.intakeZero())));
        NamedCommands.registerCommand("IntakeRun", new RunCommand(() -> sIntake.intakeForward()));
        NamedCommands.registerCommand("IntakeStop", new InstantCommand(() -> sIntake.intakeZero()));
        NamedCommands.registerCommand("Print", new InstantCommand(() -> System.out.println("Works!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")));
        NamedCommands.registerCommand("Delay", new RunCommand(() -> sIntake.intakeZero()).withTimeout(2).andThen(new InstantCommand(() -> sIntake.intakeZero())));
    }
}
