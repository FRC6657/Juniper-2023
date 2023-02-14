package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  
  private Command m_autonomousCommand;
  private RobotContainer mRobotContainer;
  private PhotonCamera OV5647;
  private AprilTagFieldLayout mTags = new AprilTagFieldLayout(FieldConstants.aprilTagList, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
  private ArrayList<Pair<PhotonCamera, Transform3d>> mCamList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private PhotonPoseEstimator mPoseEstimator;

  private Pose3d mLastPose = new Pose3d();

  @Override
  public void robotInit() {

    Logger logger = Logger.getInstance();
    mRobotContainer = new RobotContainer();

       // Record metadata
       logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
       logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
       logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
       logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
       logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
       switch (BuildConstants.DIRTY) {
         case 0:
           logger.recordMetadata("GitDirty", "All changes committed");
           break;
         case 1:
           logger.recordMetadata("GitDirty", "Uncomitted changes");
           break;
         default:
           logger.recordMetadata("GitDirty", "Unknown");
           break;
       }

       logger.addDataReceiver(new WPILOGWriter(""));
       logger.addDataReceiver(new NT4Publisher());

    logger.start();


    var instance = NetworkTableInstance.getDefault();

    if(RobotBase.isSimulation()){
      instance.stopServer();
      instance.setServer("10.0.0.5");
      instance.startClient4("photonvision");
    }

    OV5647 = new PhotonCamera(instance, "OV5647");
    mCamList.add(new Pair<PhotonCamera, Transform3d>(OV5647, new Transform3d()));

    mPoseEstimator = new PhotonPoseEstimator(mTags, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, OV5647, new Transform3d());

    Logger.getInstance().recordOutput("Apriltags",
      FieldConstants.testTag.pose
  );

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    mRobotContainer.stopAll().schedule();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    mPoseEstimator.setReferencePose(mLastPose);

    double currentTime = Timer.getFPGATimestamp();

    Optional<EstimatedRobotPose> result = mPoseEstimator.update();

    Pair<Pose3d, Double> estimation;

    if (result.isPresent()) {
      estimation = new Pair<Pose3d, Double>(result.get().estimatedPose, currentTime - result.get().timestampSeconds);
    } else {
      estimation = new Pair<Pose3d, Double>(mLastPose, 0.0);
    }

    Logger.getInstance().recordOutput("Vision Estimate", estimation.getFirst());

    mLastPose = estimation.getFirst();

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

