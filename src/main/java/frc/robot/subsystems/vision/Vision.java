// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private PIDController visionShootRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionShootDistancePID = new PIDController(5.0,0.0,0.0);
  private PIDController visionClimbRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionClimbDistancePID = new PIDController(5.0,0.0,0.0);

  private String cameraShoot = "limelight-shoot";
  private String cameraIntake = "limelight-intake";

  private int[] allTags = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
  private int[] allianceTags;
  private int[] shootTags;
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

  public void visionClimb() {
    //TODO: This method should align the robot for climbing.
  }

  public void visionShoot(Feeder sFeeder, Shooter sShooter) {
    //TODO: This method should align the robot for shooting.
  }

  public void visionTowards() {
    while (Limelight.getTV(cameraShoot)) {
      if (visionShootRotationPID.atSetpoint()) {

      }
    }
    RobotContainer.driveIdle();
  }

  public double visionHP(double hp) { // This method returns the proportion of how far away we are from the April tag horizontally in degrees relative to the cameras field of view.
    return hp/41.0;
  }

  public double visionVP(double vp) { // This method returns the proportion of how far away we are from the April tag vertically in degrees relative to the cameras field of view.
    return vp/28.1;
  }

  public double velox() {
    // return visionShootRotationPID.calculate(visionHP(Limelight.getTX(cameraShoot)));
    return calculateOutput("r", visionShootRotationPID.calculate(visionHP(Limelight.getTX(cameraShoot))) * RobotContainer.getMaxAngularRate());
  }

  public double calculateOutput(String d, double output) {
    switch(d) {
      case "l": 
        if (output < RobotContainer.getMaxSpeed()) {
          return output;
        }
        else {
          return RobotContainer.getMaxSpeed();
        }
      case "r":
        if (output < RobotContainer.getMaxAngularRate()) {
          return output;
        }
        else {
          return RobotContainer.getMaxAngularRate();
        }
    }
    return 0.0;
  }

  public void turnToTag() {
    if (Limelight.getTV(cameraShoot)) {
      RobotContainer.driveVision(0.0, 0.0, calculateOutput("r", visionShootRotationPID.calculate(visionHP(Limelight.getTX(cameraShoot)))));
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
}
