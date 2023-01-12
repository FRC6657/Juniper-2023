// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class CAN {

    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kBackRight = 3;
    public static final int kBackLeft = 4;
    public static final int kArm = 5;
    public static final int kLeftClaw = 6;
    public static final int kRightClaw = 7;
    public static final int kFlipper = 8;
    public static final int kPigeon = 5;
  }

  public static class DriveConstants {

    //ENCODERS YAY FUN 
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kFalconToMeters = Math.PI * kWheelDiameter / 2048 * 10.71; //10.71:1 gearbox


  }
}
