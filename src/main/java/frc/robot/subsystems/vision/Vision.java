// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private PIDController visionShootRotationPID = new PIDController(5.0,0.0,0.0);

  private final String cameraShoot = "limelight-shoot";
  private final String cameraRight = "limelight-intake"; 

  private double timestamp = 0; 

  private final double kShootAngleTolerance = 0.01;

  private final double blueHubX = 4.625594;
  private final double blueHubY = 4.034536;
  private final double redHubX = 11.915394;
  private final double redHubY = 4.034536;


  public Vision() {
    visionShootRotationPID.setTolerance(0.2/41.0);
    visionShootRotationPID.setSetpoint(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterSpeed", distanceToShootingSpeed());
    visionPose2dEstimator();
    RobotContainer.turn1 = getRotationOutput();
    SmartDashboard.putNumber("correctAnglePos", Math.abs(getDifferenceOmega()));
    SmartDashboard.putBoolean("RotatedCheck", getRotatedCheck());
    SmartDashboard.putNumber("turn1", RobotContainer.turn1);
  }

  public boolean allianceCool() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }
    return true;
  }

  public double distanceToShootingSpeed() {
    // return Math.pow(((getDistanceToHub() * 39.701 - 9) / 31), 2) + 40;
    // return ((getDistanceToHub() * 39.701) + 3.54307644 * Math.pow(10, 11) - 1.57) / (3.54307644 * Math.pow(10, 11) * Math.sin(5.0469822681 * Math.pow(10, -7))); 
    // return ((Math.log((182.84605 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244;
    // double v = (((Math.log((182.84605 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244) * 0.98;
    // double v = (((Math.log((183.75 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244) * 0.98;
    double v = (((Math.log((183.75 / (getDistanceToHub() * 39.701)) - 1)) - 6.41678) / -0.133244) * 0.90;
    if (v < 63) {
      return v;
    } else {
      return 63;
    }
  }

  public boolean getRotatedCheck() {
    if ((RobotContainer.calculateFieldY(RobotContainer.joystick1) * RobotContainer.getMaxSpeed() * 0.1) < -0.225) {
      if (Math.abs(getDifferenceOmega()) < 0.3) {
        return true;
      } else {
        return false;
      }
    } else if ((RobotContainer.calculateFieldY(RobotContainer.joystick1) * RobotContainer.getMaxSpeed() * 0.1) > 0.225) {
      if (Math.abs(getDifferenceOmega()) < 0.3) {
        return true;
      } else {
        return false;
      }
    } else {
      if (Math.abs(getDifferenceOmega()) < 0.1) {
        return true;
      } else {
        return false;
      }
    }
  }

  private double getRotationOutput() {
    return (-1.0 * visionShootRotationPID.calculate(getPropOmega(getDifferenceOmega())));
  }

  private double getDifferenceOmega() {
    return  getDesiredAngle() - getCurrentAngle();
  }

  private double getDesiredAngle() {
    return Math.atan(getDistanceToHubY() / getDistanceToHubX());
  }

  private double getCurrentAngle() {
    if (allianceCool()) {
      if (RobotContainer.getPose().getRotation().getRadians() < 0) {
        return RobotContainer.getPose().getRotation().getRadians() + Math.PI;
      }
      return RobotContainer.getPose().getRotation().getRadians() - Math.PI;
    } else {
      return RobotContainer.getPose().getRotation().getRadians();
    }
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
    if (allianceCool()) {
      return blueHubX;
    } else {
      return redHubX;
    }
  }

  private double getHubY() {
    if (allianceCool()) {
      return blueHubY;
    } else {
      return redHubY;
    }
  }

  private double getPropOmega(double o) {
    return o / (Math.PI/4);
  }

  private void visionPose2dEstimator() {
    if (Limelight.getTV(cameraShoot)) {
      RobotContainer.eVisionPose2d = Limelight.getBotPose2d_wpiBlue(cameraShoot);
      timestamp = Limelight.getBotPoseEstimate(cameraShoot, "botpose_wpiblue", false).timestampSeconds;
      RobotContainer.eSwerveEstimator.addVisionMeasurement(RobotContainer.eVisionPose2d, timestamp);
    } else if (Limelight.getTV(cameraRight)) {
      RobotContainer.eVisionPose2d = Limelight.getBotPose2d_wpiBlue(cameraRight);
      timestamp = Limelight.getBotPoseEstimate(cameraRight, "botpose_wpiblue", false).timestampSeconds;
      RobotContainer.eSwerveEstimator.addVisionMeasurement(RobotContainer.eVisionPose2d, timestamp);
    }
  }
}
