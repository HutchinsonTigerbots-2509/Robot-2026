// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private PIDController visionShootRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionShootDistancePID = new PIDController(5.0,0.0,0.0);
  private PIDController visionClimbRotationPID = new PIDController(5.0,0.0,0.0);
  private PIDController visionClimbDistancePID = new PIDController(5.0,0.0,0.0);

  private String cameraShoot = "shoot";
  private String cameraIntake = "intake";

  private int[] allTags = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
  private int[] allianceTags;
  private int[] shootTags;
  private int[] climbTags;

  public Vision() {
    SmartDashboard.putNumber("horizontal offset", Limelight.getTX(cameraShoot));
    visionShootRotationPID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionShootRotationPID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionShootDistancePID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionShootDistancePID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionClimbRotationPID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionClimbRotationPID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionClimbDistancePID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionClimbDistancePID.setTolerance(0.0); //TODO: Find acceptable tolerance.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void visionClimb() {
    //TODO: This method should align the robot for climbing.
  }
  
  // RobotCentric vision drive
  public void visionShoot(Feeder sFeeder, Shooter sShooter) {
    Limelight.SetFiducialIDFiltersOverride(cameraShoot, shootTags);
    visionShootRotationPID.setSetpoint(0.0); //TODO: Find acceptable setpoint. If we shoot from a set angle, this can be moved to the constructor.
    visionShootDistancePID.setSetpoint(0.0); //TODO: Find acceptable setpoint. If we shoot from a set distance, this can be moved to the constructor.
    boolean loop = true;
    while (loop) {
      if (Limelight.getTV(cameraShoot)) {
        if (visionShootRotationPID.atSetpoint()) {
          if (visionShootDistancePID.atSetpoint()) {
            sShooter.shootUnload(sFeeder);
            loop = false;
          }
          else { // This probably won't work.
            RobotContainer.driveVision(driveOutput(visionShootRotationPID.calculate(Limelight.getTargetPose3d_CameraSpace("cameraShoot").getZ())), 0.0, 0.0);
          }
        }
        else {
          RobotContainer.driveVision(0.0, 0.0, driveOutput(visionShootRotationPID.calculate(Limelight.getTX(cameraShoot))));
        }
      }
      else {
        RobotContainer.driveVision(0.0, 0.0, -0.1); // Spins the robot until it finds a shoot April tag
      }
    }
  }

  public double driveOutput(double output) { // Not sure if I need sperate ones for direction and rotation.
    output = output * 1.0; //TODO: Find ratio for distance : drivetrain power
      return output;
  }

  public void visionDriveTowards() {
    if (Limelight.getTV(cameraShoot)) {
      RobotContainer.driveVision(0.0, 0.0, -1 * driveOutput(visionShootRotationPID.calculate(Limelight.getTX(cameraShoot))));
    }
    else {
      RobotContainer.driveBrake();
    }
  }
}
