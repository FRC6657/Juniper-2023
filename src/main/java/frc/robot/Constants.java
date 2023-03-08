package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class CAN {

    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kBackRight = 3;
    public static final int kBackLeft = 4;
    public static final int kPivot = 5;
    public static final int kPigeon = 6;
    public static final int kPCM = 7;
    public static final int kLeftClaw = 8;
    public static final int kRightClaw = 9;
    public static final int kArm = 13;

  }

  public static class DriveConstants {

    //ENCODERS YAY FUN 
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kFalconToMeters = (1.0/2048) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71) ; //10.71:1 gearbox
    
    public static final double kS = 0.13305;
    public static final double kV = 2.2876;
    public static final double kA = 0.31596;

    public static double[] kDrivetrainCharacterization = {kS, kV, kA};

  }

  public static class VisionConstants {
    
    public static final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(0.043, -0.203, 0.589065),
      new Rotation3d()
    );
  }

  public static class PivotConstants {  
    
    public static final double throughboreOffset = 0.7648;
    
    public static enum SETPOINTS {

      INTAKE(-12), 
      ZERO(0),
      CARRY(30), 
      SCORE(50),
      SUBSTATION(55),
      START(66);

     public final int angle;

      private SETPOINTS(int angle) {
        this.angle = angle;
      }

    }

  }

}

