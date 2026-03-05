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

  private final double kShootDistance = 3.0; //TODO: Find correct shooting distance.
  private final double kShootDistanceTolerance = 0.0;
  private final double kShootAngleTolerance = 0.2;
  private final double kShootHigher = kShootDistance + kShootDistanceTolerance;
  private final double kShootLower = kShootDistance - kShootDistanceTolerance;

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
    SmartDashboard.putNumber("visionShootDistancePIDX", visionShootDistancePID.calculate(getPropX(getDifferenceX())));
    SmartDashboard.putNumber("visionShootDistancePIDY", visionShootDistancePID.calculate(getPropY(getDifferenceY())));
    SmartDashboard.putNumber("visionShootDistancePIDOmega", visionShootDistancePID.calculate(getPropOmega(getDifferenceOmega())));

    SmartDashboard.putNumber("getAngleShoot", getAngleShoot());
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
    // if (Limelight.getTV(cameraShoot)) {
      if (getShootingArea()) {
        dir = getDifferenceOmega();
        // RobotContainer.driveVision(
        //     visionShootDistancePID.calculate(getPropX(getDifferenceX())),
        //     visionShootDistancePID.calculate(getPropY(getDifferenceY())),
        //     visionShootDistancePID.calculate(getPropOmega(getDifferenceOmega())));
        RobotContainer.driveVision(0.0, 0.0, 10 * getDifferenceOmega());
      }
    // } else if (!Limelight.getTV(cameraShoot)) {
    //   RobotContainer.driveVision(0.0, 0.0, spinDir(dir));
    // }
    // }
  }
  
  private double getDifferenceX() {
    return kShootDistance * Math.cos(getAngleShoot());
  }

  private double getDifferenceY() {
    return kShootDistance * Math.sin(getAngleShoot());
  }

  // private double getDifferenceOmega() {
  //   return Limelight.getBotPose2d_wpiBlue(cameraShoot).getRotation().getRadians() - getAngleShoot();
  // }

  private double getDifferenceOmega() {
    return  getAngleShoot() - getShooterAngle();
  }

  // private double getAngleShoot() {
  //   return Math.atan(getDistanceToHubY() / getDistanceToHuBX());
  // }

  private double getAngleShoot() {
    return Math.atan(getDistanceToHubX() / getDistanceToHubY());
  }

  private double getShooterAngle() {
    if (Limelight.getBotPose2d_wpiBlue(cameraShoot).getRotation().getRadians() < 0) {
      return Limelight.getBotPose2d_wpiBlue(cameraShoot).getRotation().getRadians() + Math.PI;
    }
    return Limelight.getBotPose2d_wpiBlue(cameraShoot).getRotation().getRadians() - Math.PI;
  }

  private boolean getShootingArea() {
    if (RobotContainer.getAllianceBlue()) {
      if (Limelight.getBotPose2d_wpiBlue(cameraShoot).getX() < getHubX()) {
        return true;
      } else {
        return false;
      }
    } else if (!RobotContainer.getAllianceBlue()){
      if (Limelight.getBotPose2d_wpiBlue(cameraShoot).getX() > getHubX()) {
        return true;
      } else {
        return false;
      }
    }
    return RobotContainer.magicBool;
  }
 
  private double getDistanceToHub() {
    return Math.sqrt((getDistanceToHubY() * getDistanceToHubY()) + (getDistanceToHubX() * getDistanceToHubX()));
  }

  private double getDistanceToHubX() {
    return getHubX() - Limelight.getBotPose2d_wpiBlue(cameraShoot).getX();
  }

  private double getDistanceToHubY() {
    return getHubY() - Limelight.getBotPose2d_wpiBlue(cameraShoot).getY();
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

  private double spinDir(double n) {
    if (n > 0) {
      return RobotContainer.getMaxAngularRate();
    }
    return 0 - RobotContainer.getMaxAngularRate();
  }
}
