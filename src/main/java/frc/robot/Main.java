// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {

    //    **** LEGEND ****   //
    //     k - Constant      //
    //     s - Subsystem     //
    //     m - Motor         //
    //     e - Encoder       //
    //     w - Switch        //
    //     j - Joystick      //

    /* TODO:
     * 
     * Improve autos
     * Integrate the feeder into correctAnglePos() which should be called during shooting
     * Clean up and comment code
     * Torque readings from intake and lift motors for jams and overdrive
     * 
     */

  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
