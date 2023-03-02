package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  
  private Command m_autonomousCommand;
  private RobotContainer mRobotContainer;

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

    m_autonomousCommand = mRobotContainer.getAutonomousCommand();
      
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

