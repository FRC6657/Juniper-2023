// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriverControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final Drivetrain m_Drivetrain;
  private final DoubleSupplier pXInput; 
  private final DoubleSupplier pYInput;
  private final DoubleSupplier pZInput; 


  public DriverControl(Drivetrain drivetrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier zInput) {
    m_Drivetrain = drivetrain;
    pXInput = xInput;
    pYInput = yInput;
    pZInput = zInput;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Drivetrain.Drive(pXInput.getAsDouble(), pYInput.getAsDouble(), pZInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_Drivetrain.Drive(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
