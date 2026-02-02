// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Pathplanner extends SubsystemBase {
  /** Creates a new Pathplanner. */

  Drivetrain sDrivetrain;
  static RobotConfig config;
  static Pose2d startPose2d;
  String defaultAuto = "TestAuto";

  public Pathplanner(Drivetrain kDrivetrain) {

    //TODO: Fix overrun error

    sDrivetrain = kDrivetrain;
    startPose2d = new Pose2d(0,0, new Rotation2d(0));

    try{
      AutoBuilder.configure(
        () -> RobotContainer.getPose(), // Robot pose supplier
        resetPose -> RobotContainer.resetPose(startPose2d), // Method to reset odometry (will be called if your auto has a starting pose)
        () -> RobotContainer.getRobotRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        speeds -> RobotContainer.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
    RobotContainer.eSwerveEstimator = new SwerveDrivePoseEstimator(sDrivetrain.getKinematics(), RobotContainer.getRotation2d(), RobotContainer.getModulePositions(), startPose2d);
    RobotContainer.eSwerveEstimator.resetPose(startPose2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.eSwerveEstimator.update(RobotContainer.getRotation2d(), RobotContainer.getModulePositions());
  }
}
