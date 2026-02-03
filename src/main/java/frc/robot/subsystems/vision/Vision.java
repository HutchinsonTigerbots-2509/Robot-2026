// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private PIDController visionDrivePID = new PIDController(5.0,0.0,0.0);
  private String cameraShoot = "cameraShoot";
  private String cameraIntake = "cameraIntake";

  public Vision() {
    visionDrivePID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionDrivePID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionDrivePID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionDrivePID.setTolerance(0.0); //TODO: Find acceptable tolerance.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isAligned(String Case) { // This method will return true if the robot is aligned
    switch (Case) {
      case "climb":
        if (visionDrivePID.atSetpoint()) { // Get camera distance and angle then compare to tolerance
          return true;
        }
        else {
          return false;
        }
      case "shoot":
        if (visionDrivePID.atSetpoint()) {
          return true;
        }
        else {
          return false;
        }
      case "load":
        if (visionDrivePID.atSetpoint()) {
          return true;
        }
        else {
          return false;
        }
    }
    return false;
  }

  public void visionClimb() {
    Limelight.SetFiducialIDFiltersOverride(cameraShoot, new int[]{1, 2, 3, 4});
    boolean hasTarget = Limelight.getTV(cameraShoot);
    if (hasTarget) {

    }
    //TODO: This method should align the robot for climbing.
  }

  public void visionShoot() {
    //TODO: This method should align the robot for shooting.
  }

  public void visionLoad() {
    //TODO: This method should align the robot for loading.
  }
}
