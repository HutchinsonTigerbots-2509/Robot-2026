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

  private PIDController visionTestPID = new PIDController(5.0,0.0,0.0);

  private String cameraShoot = "limelight-shoot";
  private String cameraIntake = "limelight-intake";

  private int[] allTags = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
  private int[] allianceTags;
  private int[] shootTags;
  private int[] climbTags;

  public Vision() {
    visionShootRotationPID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionShootRotationPID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionShootDistancePID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionShootDistancePID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionClimbRotationPID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionClimbRotationPID.setTolerance(0.0); //TODO: Find acceptable tolerance.
    visionClimbDistancePID.setIZone(0.0); //TODO: Find acceptable IZone.
    visionClimbDistancePID.setTolerance(0.0); //TODO: Find acceptable tolerance.

    
    visionTestPID.setIZone(0.0);
    visionTestPID.setTolerance(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void visionClimb() {
    //TODO: This method should align the robot for climbing.
  }

  public void visionShoot(Feeder sFeeder, Shooter sShooter) {
    //TODO: This method should align the robot for shooting.
  }

  public void driveOutput(double vx, double vy, double vOmega) {
    vx = vx * RobotContainer.MaxAngularRate;
    RobotContainer.driveVision(0.0, 0.0, driveOutput(visionTestPID.calculate(Limelight.getTX(cameraShoot))));
  }

  public void visionTowards() {
    while (Limelight.getTV(cameraShoot)) {
      
    }
    RobotContainer.driveIdle();
  }
}
