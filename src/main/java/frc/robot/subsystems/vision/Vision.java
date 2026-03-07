// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.feeder.FeederHopper;
import frc.robot.subsystems.shooter.Shooter;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private PIDController visionShootRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionShootDistancePID = new PIDController(5.0,0.0,0.0);
  private PIDController visionClimbRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionClimbDistancePID = new PIDController(5.0,0.0,0.0);

  private final String cameraShoot = "limelight-shoot";
  private final String cameraIntake = "limelight-intake";

  private double timestamp;

  private final double kShootDistance = 3.0; //TODO: Find correct shooting distance.
  private final double kShootDistanceTolerance = 0.5;
  private final double kShootAngleTolerance = 0.00001;

  private final double blueHubX = 4.625; //TODO: Find correct hub coordinates. These should be from the Welded Perimeter.
  private final double blueHubY = 4.050;
  private final double redHubX = 11.925;
  private final double redHubY = 4.030;

  private int[] climbTags;

  private double dir;

  public Vision() {
    visionShootRotationPID.setTolerance(0.2/41.0);
    visionShootDistancePID.setTolerance(0.0);
    visionShootRotationPID.setSetpoint(0.0);
    visionShootDistancePID.setSetpoint(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("getX", Limelight.getBotPose2d_wpiBlue(cameraShoot).getX());
    SmartDashboard.putNumber("getY", Limelight.getBotPose2d_wpiBlue(cameraShoot).getY());
    SmartDashboard.putNumber("getOmega", getShooterAngle());
    SmartDashboard.putNumber("getDifferenceX", getDifferenceX());
    SmartDashboard.putNumber("getDifferenceY", getDifferenceY());
    SmartDashboard.putNumber("getDifferenceOmega", getDifferenceOmega());
    SmartDashboard.putNumber("visionShootDistancePIDX", 10 * visionShootDistancePID.calculate(getPropX(getDifferenceX())));
    SmartDashboard.putNumber("visionShootDistancePIDY", 10 * visionShootDistancePID.calculate(getPropY(getDifferenceY())));
    SmartDashboard.putNumber("visionShootDistancePIDOmega", -1 * visionShootDistancePID.calculate(getPropOmega(getDifferenceOmega())));
    SmartDashboard.putBoolean("shootingAnglebool", correctShootPos());
    SmartDashboard.putNumber("getAngleShoot", getAngleShoot());
    SmartDashboard.putNumber("getDistanceDifference", Math.abs(getDistanceToHub() - kShootDistance));
    SmartDashboard.putNumber("getDistanceToHub", getDistanceToHub());
    SmartDashboard.putBoolean("getCorrectAnglePos", correctAnglePos());
    SmartDashboard.putNumber("getPropX", getPropX(getDifferenceX()));
    SmartDashboard.putNumber("getPropY", getPropY(getDifferenceY()));
    SmartDashboard.putNumber("timestamp", timestamp);
    RobotContainer.eSwerveEstimator.addVisionMeasurement(RobotContainer.eVisionPose2d, timestamp);
    visionPose2dEstimator();
  }

  public void visionClimb(Climber sClimber) {
    //TODO: This method should align the robot for climbing.
  }

  // public void visionShoot(FeederHopper sFeederHopper, Shooter sShooter) {
  //   if (correctShootPos()) {
  //     RobotContainer.driveBrake();
  //     sShooter.shootUnload(sFeederHopper); // Call actual shoot method
  //   } else if (!correctShootPos()) {
  //     driveToShootPos();
  //   }
  // }

  public boolean visionShoot() {
    if (correctShootPos()) {
      RobotContainer.driveBrake();
      return true;
    } else if (!correctShootPos()) {
      driveToShootPos();
    }
    return false;
  }

  public boolean visionTurnShoot() {
    if (correctAnglePos()) {
      RobotContainer.driveBrake();
      return true;
    } else if (!correctAnglePos()) {
      turnToShootPos();
    }
    return false;
  }

  public void visionCancel(Climber sClimber, FeederHopper sFeederHopper, Shooter sShooter) {
    RobotContainer.driveIdle();
    sShooter.shootCancel(sFeederHopper);
  }

  public boolean correctShootPos() {
    if (Math.abs(getDistanceToHub() - kShootDistance) < kShootDistanceTolerance && Math.abs(getDifferenceOmega()) < kShootAngleTolerance) {
      return true;
    }
    return false;
  }

  public boolean correctAnglePos() {
    if (Math.abs(getDifferenceOmega()) < kShootAngleTolerance) {
      return true;
    }
    return false;
  }

  // private void driveToShootPos() {
  //   if (Limelight.getTV(cameraShoot)) {
  //     if (getShootingArea()) {
  //       if (correctAnglePos()) {
  //         RobotContainer.driveVision(
  //           visionShootDistancePID.calculate(getPropX(getDifferenceX())),
  //           visionShootDistancePID.calculate(getPropY(getDifferenceY())),
  //           0.0);
  //       } else {
  //         dir = 10 * getDifferenceOmega();
  //         RobotContainer.driveVision(
  //           0.0,
  //           0.0,
  //           -1 * visionShootRotationPID.calculate(getPropOmega(getDifferenceOmega())));
  //       }
  //     }
  //   } else if (!Limelight.getTV(cameraShoot)) {
  //     RobotContainer.driveVision(0.0, 0.0, spinDir(dir));
  //   }
  // }

  private void driveToShootPos() {
    RobotContainer.driveVision(
      10 * visionShootDistancePID.calculate(getPropX(getDifferenceX())),
      10 * visionShootDistancePID.calculate(getPropY(getDifferenceY())),
      -1 * visionShootRotationPID.calculate(getPropOmega(getDifferenceOmega())));
  }

  private void turnToShootPos() {
    RobotContainer.driveVision(
      0.0,
      0.0,
      -1 * visionShootRotationPID.calculate(getPropOmega(getDifferenceOmega())));
  }

  private double getDifferenceX() {
    return getDistanceToHubX() - kShootDistance * Math.cos(getAngleShoot());
  }

  private double getDifferenceY() {
    return getDistanceToHubY() - kShootDistance * Math.sin(getAngleShoot());
  }

  private double getDifferenceOmega() {
    return  getAngleShoot() - getShooterAngle();
  }

  private double getAngleShoot() {
    return Math.atan(getDistanceToHubY() / getDistanceToHubX());
  }

  private double getShooterAngle() {
    if (RobotContainer.getPose().getRotation().getRadians() < 0) {
      return RobotContainer.getPose().getRotation().getRadians() + Math.PI;
    }
    return RobotContainer.getPose().getRotation().getRadians() - Math.PI;
  }
 
  private double getDistanceToHub() {
    return Math.sqrt((getDistanceToHubY() * getDistanceToHubY()) + (getDistanceToHubX() * getDistanceToHubX()));
  }

  private double getDistanceToHubX() {
    return getHubX() - RobotContainer.getPose().getX();
  }

  private double getDistanceToHubY() {
    return getHubY() - RobotContainer.getPose().getY();
  }

  private double getHubX() {
    if (RobotContainer.getAllianceBlue()) {
      return blueHubX;
    } else if (!RobotContainer.getAllianceBlue()){
      return redHubX;
    }
    return RobotContainer.magicNum;
  }

  private double getHubY() {
    if (RobotContainer.getAllianceBlue()) {
      return blueHubY;
    } else if (!RobotContainer.getAllianceBlue()){
      return redHubY;
    }
    return RobotContainer.magicNum;
  }

  private double getPropX(double x) {
    return x / blueHubX;
  }

  private double getPropY(double y) {
    return y / blueHubY;
  }

  private double getPropOmega(double o) {
    return o / (Math.PI/4);
  }

  private void visionPose2dEstimator() {
    if (Limelight.getTV(cameraShoot)) {
      RobotContainer.eVisionPose2d = Limelight.getBotPose2d_wpiBlue(cameraShoot);
      timestamp = Limelight.getBotPoseEstimate(cameraShoot, "botpose_wpiblue", false).timestampSeconds;
    } else if (Limelight.getTV(cameraIntake)) {
      RobotContainer.eVisionPose2d = Limelight.getBotPose2d_wpiBlue(cameraIntake);
      timestamp = Limelight.getBotPoseEstimate(cameraIntake, "botpose_wpiblue", false).timestampSeconds;
    }
  }
}
