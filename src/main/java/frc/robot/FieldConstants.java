// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

    public static final double kFieldLength = Units.inchesToMeters(651.25);
    public static final double kFieldWidth = Units.inchesToMeters(315.5);

    //Location in classroom
    // public final static AprilTag testTag = new AprilTag(2, 
    //     new Pose3d(
    //         (kFieldLength / 2),
    //         (kFieldWidth / 2),
    //         Units.inchesToMeters(45),
    //         new Rotation3d(0,0, Math.PI/2)
    //     )
    // );
    
    public final static AprilTag tag1 = new AprilTag(1, 
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))
    );

    //Commented out for the sake of class testing, location on field correct
    public final static AprilTag tag2 = new AprilTag(2, 
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))
    );

    public final static AprilTag tag3 = new AprilTag(3, 
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))
    );

    public final static AprilTag tag4 = new AprilTag(4,
        new Pose3d(
              Units.inchesToMeters(636.96),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d(0.0, 0.0, Math.PI))
    );

    public final static AprilTag tag5 = new AprilTag(5, 
        new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d())
    );

    public final static AprilTag tag6 = new AprilTag(6, 
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d())
    );

    public final static AprilTag tag7 = new AprilTag(7, 
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d())
    );

    public final static AprilTag tag8 = new AprilTag(8, 
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d())
    );
    


    public static ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>(){
        {
            //add(testTag);
            add(tag1);
            add(tag2);
            add(tag3);
            add(tag4);
            add(tag5);
            add(tag6);
            add(tag7);
            add(tag8);
        }
    };
}