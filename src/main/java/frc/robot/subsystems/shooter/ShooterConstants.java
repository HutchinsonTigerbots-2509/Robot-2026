// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class ShooterConstants {

    public static final int kShootAMotorId = 11;
    public static final int kShootBMotorId = 12;

    public static PIDController shootPID = new PIDController(5.0,0.0,0.0); //Tune this.
}
