// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
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

  private final double kShootDistance = 0.0; //TODO: Find correct shooting distance.
  private final double kShootDistanceTolerance = 0.0;
  private final double kShootAngleTolerance = 0.0;
  private final double kShootHigher = kShootDistance + kShootDistanceTolerance;
  private final double kShootLower = kShootDistance - kShootDistanceTolerance;

  private final double blueHubX = 182.11; //TODO: Find correct hub coordinates. These should be from the Welded Perimeter.
  private final double blueHubY = 158.34;
  private final double redHubX = 469.11;
  private final double redHubY = 158.84;

  private int[] climbTags;

  public Vision() {
    visionShootRotationPID.setTolerance(0.2/41.0);
    visionShootDistancePID.setTolerance(0.0);
    visionShootRotationPID.setSetpoint(0.0);
    visionShootDistancePID.setSetpoint(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("xvelo", velox());
    // SmartDashboard.putNumber("yvelo", veloy);
  }

  public double visionHP(double hp) { // This method returns the proportion of how far away we are from the April tag horizontally in degrees relative to the cameras field of view.
    return hp/41.0;
  }

  public double visionVP(double vp) { // This method returns the proportion of how far away we are from the April tag vertically in degrees relative to the cameras field of view.
    return vp/28.1;
  }

  public double velox() {
    // return visionShootRotationPID.calculate(visionHP(Limelight.getTX(cameraShoot)));
    return visionShootRotationPID.calculate(visionHP(Limelight.getTX(cameraShoot))) * RobotContainer.getMaxAngularRate();
  }

  public void turnToTag() {
    if (Limelight.getTV(cameraShoot)) {
      RobotContainer.driveVision(0.0, 0.0, visionShootRotationPID.calculate(visionHP(Limelight.getTX(cameraShoot))));
    }
  }

  public void driveToTag() {
    if (visionShootRotationPID.atSetpoint()) {
      RobotContainer.driveVision(-1.0, 0.0, 0.0);
    }
    else {
      turnToTag();
    }
  }

  // VVVVV Comp vision methods below VVVVV

  public void visionClimb(Climber sClimber) {
    //TODO: This method should align the robot for climbing.
  }

  public void visionShoot(FeederHopper sFeederHopper, Shooter sShooter) {
    if (correctShootPos()) {
      RobotContainer.driveBrake();
      // sShooter.shootUnload(sFeeder); // Call actual shoot method
    } else if (!correctShootPos()) {
      driveToShootPos();
    }
  }

  public void visionCancel(Climber sClimber, FeederHopper sFeederHopper, Shooter sShooter) {
    RobotContainer.driveIdle();
    sShooter.shootCancel(sFeederHopper);
  }

  private boolean correctShootPos() {
    if (getDistanceToHub() < kShootHigher && getDistanceToHub() > kShootLower && getDifferenceOmega() < kShootAngleTolerance) {
      return true;
    }
    return false;
  }
  
  private void driveToShootPos() {
    if (getShootingArea()) {
      RobotContainer.driveVision(
        visionShootDistancePID.calculate(getPropX(getDifferenceX())), 
        visionShootDistancePID.calculate(getPropY(getDifferenceY())), 
        visionShootDistancePID.calculate(getPropOmega(getDifferenceOmega())));
    }
    RobotContainer.driveVision(0.0, 0.0, RobotContainer.getMaxAngularRate());
  }
  
  private double getDifferenceX() {
    return kShootDistance * Math.cos(getAngleShoot());
  }

  private double getDifferenceY() {
    return kShootDistance * Math.sin(getAngleShoot());
  }

  private double getDifferenceOmega() {
    return Limelight.getBotPose2d(cameraShoot).getRotation().getRadians() - getAngleShoot();
  }

  private double getAngleShoot() {
    return Math.atan(getDistanceToHubY() / getDistanceToHuBX());
  }

  private boolean getShootingArea() {
    if (RobotContainer.getAllianceBlue()) {
      if (Limelight.getBotPose2d(cameraShoot).getX() < getHubX()) {
        return true;
      } else {
        return false;
      }
    } else if (!RobotContainer.getAllianceBlue()){
      if (Limelight.getBotPose2d(cameraShoot).getX() > getHubX()) {
        return true;
      } else {
        return false;
      }
    }
    return RobotContainer.magicBool;
  }
 
  private double getDistanceToHub() {
    return Math.sqrt((getDistanceToHubY() * getDistanceToHubY()) / (getDistanceToHuBX() * getDistanceToHuBX()));
  }

  private double getDistanceToHuBX() {
    return Limelight.getBotPose2d(cameraShoot).getX() - getHubX();
  }

  private double getDistanceToHubY() {
    return Limelight.getBotPose2d(cameraShoot).getY() - getHubY();
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
    return o / 41.0;
  }
}
