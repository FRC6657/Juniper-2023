package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Pistons;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);
  private CommandXboxController mTesting = new CommandXboxController(2);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Pistons pistons = new Pistons();
  private final Claw claw = new Claw();


  public RobotContainer() {

    configureBindings();
  
  }

 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftY(), 0.1) * 3, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.1) * 3 , 
        ()-> deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1) * 3, 
        false));

        mTesting.x().whileTrue(
          new InstantCommand(
            pistons::extend,
            pistons
          )
        );

        mTesting.y().whileTrue(
          new InstantCommand(
            pistons::retract,
            pistons
          )
        );

        //Operator

      mOperator.x().whileTrue(
        new InstantCommand(
          claw::intake,
          claw
        )).whileFalse(
          new InstantCommand(
            claw::stop,
            claw
          )
      );

      mOperator.y().whileTrue(
        new InstantCommand(
          claw::outtake,
          claw
        )).whileFalse(
          new InstantCommand(
            claw::stop,
            claw
          )
      );

      //Cube - extend, wheels spin
      mOperator.b().whileTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            pistons::extend,
            pistons
          ),
          new InstantCommand(
            claw::intake,
            claw
          )
        )
      );

      mOperator.x().whileTrue(
        new InstantCommand(
          arm::retract,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );
      
      mOperator.y().whileTrue(
        new InstantCommand(
          arm::extend,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );

      //Driver

      // Resets the gyro, specifically for when field relative driving is on
      mDriver.x().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain)
      );

      }
    }