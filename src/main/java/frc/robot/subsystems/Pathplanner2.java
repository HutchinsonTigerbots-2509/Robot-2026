
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.io.Console;
import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Pathplanner2 extends SubsystemBase {
  /** Creates a new pathplanner. */

  Drivetrain sDrivetrain;

  static Pose2d startPose2d;
  static String defaultauto = "TestAuto";

  SwerveDrivePoseEstimator eSwerveEstimator;

  public static PIDController rotationController = new PIDController(.25, 0.1, 0);

  static SendableChooser<Command> autoSelect;

  public static RobotConfig config;

  public Pathplanner2(Drivetrain kDrivetrain) {

    sDrivetrain = kDrivetrain;
    
    sDrivetrain.getPigeon2().reset();
    autoSelect = new SendableChooser<Command>();
    startPose2d = new Pose2d(0,0, new Rotation2d(0));
    rotationController.setTolerance(2);

    try{
      config = RobotConfig.fromGUISettings(); 
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    
    try {
      AutoBuilder.configure(
        () -> getPose(), // Robot pose supplier
        resetPos2d -> resetPose(startPose2d), // Method to reset odometry (will be called if your auto has a starting pose)
        () -> getRobotRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        output -> driveRobotRelative(output), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          new PIDConstants(5, 0.5, 0.0), // Translation PID constants
          new PIDConstants(7, 0.5, 0.0) // Rotation PID constants
        ),
        RobotConfig.fromGUISettings(), // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return false; // alliance.get() == DriverStation.Alliance.Red;
          }
            return false;
        },
        sDrivetrain // Reference to this subsystem to set requirements
      );
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    buildAutoChooser();

    // Configure AutoBuilder last

    eSwerveEstimator = new SwerveDrivePoseEstimator(this.getKinematics(), getRotation2d(), getModulePositions(), startPose2d);
      
    ApplyStart();

    eSwerveEstimator.resetPose(startPose2d);

  }

  //** Returns the List of SwerveModulePositions in ( F:LR  R:LR ) order */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition FL = sDrivetrain.getModule(0).getPosition(false);
    SwerveModulePosition FR = sDrivetrain.getModule(1).getPosition(false);
    SwerveModulePosition RL = sDrivetrain.getModule(2).getPosition(false);
    SwerveModulePosition RR = sDrivetrain.getModule(3).getPosition(false);
    SwerveModulePosition[] ModPositions = new SwerveModulePosition[] {
      FL, FR, RL, RR
    };
    return ModPositions;
  }

  //** Returns the list of SwerveModuleStates in ( F:LR  R:LR ) order */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState FL = sDrivetrain.getModule(0).getCurrentState();
    SwerveModuleState FR = sDrivetrain.getModule(1).getCurrentState();
    SwerveModuleState RL = sDrivetrain.getModule(2).getCurrentState();
    SwerveModuleState RR = sDrivetrain.getModule(3).getCurrentState();
    SwerveModuleState[] ModStates = new SwerveModuleState[] {
      FL, FR, RL, RR
    };
    return ModStates;
  }
  
  public void setGyro(Double pos) {
    sDrivetrain.getPigeon2().setYaw(pos);
  }

  public Pose2d getPose() {
    Pose2d pose2d = eSwerveEstimator.getEstimatedPosition();
    return pose2d;
  }

  public void resetPose(Pose2d pos) {
    sDrivetrain.resetPose(pos);
    setGyro(pos.getRotation().getDegrees());
    eSwerveEstimator.resetPose(pos);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = sDrivetrain.getKinematics().toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speed) {
    RobotContainer.PathplannerDriveSwerve(speed);
  }

  public SwerveDriveKinematics getKinematics() {
    SwerveDriveKinematics kinematics = sDrivetrain.getKinematics();
    return kinematics;
  }

  public Rotation2d getRotation2d() {
    Rotation2d pos = sDrivetrain.getPigeon2().getRotation2d();
    return pos;
  }

  @Override
  public void periodic() {
  }

  public void ApplyStartCool() {
    String autoName = autoSelect.getSelected().toString();
    try {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
        startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
        //SmartDashboard.putString("Side", "blue");
      }
      else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).flipPath().mirrorPath().getStartingHolonomicPose().get());
        startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).flipPath().mirrorPath().getStartingHolonomicPose().get();
        //SmartDashboard.putString("Side", "red");
      }
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
   
  public void ApplyStart() {
    String autoName = autoSelect.getSelected().getName();
    try {
      AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
      startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      startPose2d = new Pose2d(0,0,new Rotation2d(0));
    }
  };


  public static Command getAutonomousCommand() {
      return autoSelect.getSelected();
  }

  public void buildAutoChooser() {
    autoSelect.setDefaultOption(defaultauto, AutoBuilder.buildAuto(defaultauto));
    List<String> options = AutoBuilder.getAllAutoNames();
    for (String n : options) {
      if(n != defaultauto)
      autoSelect.addOption(n, AutoBuilder.buildAuto(n));
    };
  }

  public void ResetPoseNextMove(Pose2d pos) {
    Pose2d resetPose;
    resetPose = pos;
    resetPose(resetPose);
  }

  public double getRotationMove(double dAngle) {
    double speed;
    double desiredAngle = dAngle;

    speed = rotationController.calculate(getRotation2d().getDegrees(), desiredAngle);

    return speed;
  }

}
