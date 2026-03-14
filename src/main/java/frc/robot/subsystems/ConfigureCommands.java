// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.autonomous.Pathplanner;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.feederhopper.FeederHopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class ConfigureCommands extends SubsystemBase {
  /** Creates a new Commands. */

  private static Climber sClimber;
  private static FeederHopper sFeederHopper;
  private static Intake sIntake;
  private static Shooter sShooter;
  private static Vision sVision;

  public ConfigureCommands(Climber kClimber, FeederHopper kFeederHopper, Intake kIntake, Shooter kShooter, Vision kVision) {
    sClimber = kClimber;
    sFeederHopper = kFeederHopper;
    sIntake = kIntake;
    sShooter = kShooter;
    sVision = kVision;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void createCommands() {
    // TODO: simply configureBindings() by moving commands here

    RunCommand climbUp = new RunCommand(() -> sClimber.climb2());
    RunCommand climbDown = new RunCommand(() -> sClimber.climbDown());
    InstantCommand climbStop = new InstantCommand(() -> sClimber.climbZero());
    RunCommand feedStart = new RunCommand(() -> sFeederHopper.feedNumMethod(-80));
    RunCommand feedReverse = new RunCommand(() -> sFeederHopper.feedReverse());
    InstantCommand feedStop = new InstantCommand(() -> sFeederHopper.feedzero());
    RunCommand hopperOn = new RunCommand(() -> sFeederHopper.hopperOn());
    InstantCommand hopperOff = new InstantCommand(() -> sFeederHopper.hopperOn());

  }
}
