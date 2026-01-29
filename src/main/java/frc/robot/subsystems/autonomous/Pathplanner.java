// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autonomous;

import java.io.Console;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Pathplanner extends SubsystemBase {
  /** Creates a new Pathplanner. */

  Drivetrain sDrivetrain;

  SwerveDrivePoseEstimator eSwerveEstimator;
  static RobotConfig config;
  static Pose2d startPose2d;
  String defaultAuto = "TestAuto";
//   private final SendableChooser<Command> autoSelect;



  public Pathplanner(Drivetrain kDrivetrain) {

    sDrivetrain = kDrivetrain;

    startPose2d = new Pose2d(0,0, new Rotation2d(0));
    defaultAuto = new pathPla "TestAuto";
    // autoSelect = new SendableChooser<>();

    try{
      AutoBuilder.configure(
        () -> getPose(), // Robot pose supplier
        resetPose -> resetPose(startPose2d), // Method to reset odometry (will be called if your auto has a starting pose)
        () -> getRobotRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        speeds -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        RobotConfig.fromGUISettings(), // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
            return false;
        },
        sDrivetrain // Reference to this subsystem to set requirements
      );    
    } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
    }

    // buildAutoChooser();

    // ApplyStart();

    // getAutonomousCommand();

    
    eSwerveEstimator = new SwerveDrivePoseEstimator(this.getKinematics(), getRotation2d(), getModulePositions(), startPose2d);
    eSwerveEstimator.resetPose(startPose2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Pose2d getPose() {
    Pose2d pos0 = eSwerveEstimator.getEstimatedPosition();
    return pos0;
  }

  public void resetPose(Pose2d pos1) {
    sDrivetrain.resetPose(pos1);
    setGyro(pos1.getRotation().getDegrees());
    eSwerveEstimator.resetPose(pos1);  
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds speed1;
    speed1 = sDrivetrain.getKinematics().toChassisSpeeds(getModuleStates());
    return speed1;
  }

  public void driveRobotRelative(ChassisSpeeds speed2) {
    RobotContainer.PathplannerDriveSwerve(speed2);
  }

  public void setGyro(Double pos2) {
    sDrivetrain.getPigeon2().setYaw(pos2);
  }

  public SwerveDriveKinematics getKinematics() {
    SwerveDriveKinematics kinematics = sDrivetrain.getKinematics();
    return kinematics;
  }

  public Rotation2d getRotation2d() {
    Rotation2d pos3 = sDrivetrain.getPigeon2().getRotation2d();
    return pos3;
  }

  //** Returns the List of SwerveModulePositions in ( F:LR  R:LR ) order */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition FL = sDrivetrain.getModule(0).getPosition(false);
    SwerveModulePosition FR = sDrivetrain.getModule(1).getPosition(false);
    SwerveModulePosition RL = sDrivetrain.getModule(2).getPosition(false);
    SwerveModulePosition RR = sDrivetrain.getModule(3).getPosition(false);
    SwerveModulePosition[] ModPositions = new SwerveModulePosition[] {FL, FR, RL, RR};
    return ModPositions;
  }

  //** Returns the list of SwerveModuleStates in ( F:LR  R:LR ) order */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState FL = sDrivetrain.getModule(0).getCurrentState();
    SwerveModuleState FR = sDrivetrain.getModule(1).getCurrentState();
    SwerveModuleState RL = sDrivetrain.getModule(2).getCurrentState();
    SwerveModuleState RR = sDrivetrain.getModule(3).getCurrentState();
    SwerveModuleState[] ModStates = new SwerveModuleState[] {FL, FR, RL, RR};
    return ModStates;
  }

  // This is here for testing purposes
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    //return new PathPlannerAuto(defaultAuto);
    return //PathplannerAuto(defaultAuto);
  }

//   public void buildAutoChooser() {
//     autoSelect.setDefaultOption(defaultAuto, AutoBuilder.buildAuto(defaultAuto));
//     List<String> options = AutoBuilder.getAllAutoNames();
//     for (String n : options) {
//       if(n != defaultAuto)
//       autoSelect.addOption(n, AutoBuilder.buildAuto(n));
//     };
//   }

//   public void ApplyStart() {
//     String autoName = autoSelect.getSelected().getName();
//     try {
//       AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
//       startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
//     } catch (Exception e) {
//       // TODO Auto-generated catch block
//       e.printStackTrace();
//       startPose2d = new Pose2d(0,0,new Rotation2d(0));
//     }
//   };


}
