// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private PIDController visionShootRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionShootDistancePID = new PIDController(5.0,0.0,0.0);

  private final String cameraShoot = "limelight-shoot";
  private final String cameraIntake = "limelight-intake"; 

  private double timestamp = 0; 

  // private double kSinFactor = 3.54307644 * Math.pow(10, 11);

  private final double kShootDistance = 3.0; //TODO: Find correct shooting distance.
  private final double kShootDistanceTolerance = 0.5;
  private final double kShootAngleTolerance = 0.000001;

  private final double blueHubX = 4.625; //4.572;
  private final double blueHubY = 4.050; //4.08;
  private final double redHubX = 11.925;
  private final double redHubY = 4.030;

  public Vision() {
    visionShootRotationPID.setTolerance(0.2/41.0);
    visionShootDistancePID.setTolerance(0.0);
    visionShootRotationPID.setSetpoint(0.0);
    visionShootDistancePID.setSetpoint(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("getX", RobotContainer.getPose().getX());
    SmartDashboard.putNumber("getY", RobotContainer.getPose().getY());
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
    SmartDashboard.putNumber("shootingSpeed", distanceToShootingSpeed());
    // RobotContainer.eSwerveEstimator.addVisionMeasurement(RobotContainer.eVisionPose2d, timestamp);
    visionPose2dEstimator();
    RobotContainer.turn1 = turnToShootPos1();
  }

  public double distanceToShootingSpeed() {
    // return Math.pow(((getDistanceToHub() * 39.701 - 9) / 31), 2) + 40;
    // return ((getDistanceToHub() * 39.701) + kSinFactor - 1.57) / (kSinFactor * Math.sin(5.0469822681 * Math.pow(10, -7))); 
    // return ((Math.log((182.84605 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244;
    // double v = (((Math.log((182.84605 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244) * 0.98;
    double v = (((Math.log((183.75 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244) * 0.98;
    if (v < 63) {
      return v;
    } else {
      return 63;
    }
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
      // RobotContainer.driveBrake();
      return true;
    } else if (!correctAnglePos()) {
      turnToShootPos();
    }
    return false;
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

  private double turnToShootPos1() {
    return (-1.0 * visionShootRotationPID.calculate(getPropOmega(getDifferenceOmega())));
  }

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

  // private double getShooterAngle() {
  //   if (RobotContainer.getPose().getRotation().getRadians() < 0) {
  //     return RobotContainer.getPose().getRotation().getRadians() + Math.PI;
  //   }
  //   return RobotContainer.getPose().getRotation().getRadians() - Math.PI;
  // }

  private double getShooterAngle() {
    if (RobotContainer.getAllianceBlue()) {
      if (RobotContainer.getPose().getRotation().getRadians() < 0) {
        return RobotContainer.getPose().getRotation().getRadians() + Math.PI;
      }
      return RobotContainer.getPose().getRotation().getRadians() - Math.PI;
    } else if (!RobotContainer.getAllianceBlue()) {
      return RobotContainer.getPose().getRotation().getRadians();
    }
    return RobotContainer.magicNum;
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
      RobotContainer.eSwerveEstimator.addVisionMeasurement(RobotContainer.eVisionPose2d, timestamp);
    } else if (Limelight.getTV(cameraIntake)) {
      RobotContainer.eVisionPose2d = Limelight.getBotPose2d_wpiBlue(cameraIntake);
      timestamp = Limelight.getBotPoseEstimate(cameraIntake, "botpose_wpiblue", false).timestampSeconds;
      RobotContainer.eSwerveEstimator.addVisionMeasurement(RobotContainer.eVisionPose2d, timestamp);
    }
  }
}
