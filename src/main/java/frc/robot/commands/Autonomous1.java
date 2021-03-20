/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArcadeDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous1 extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous1.
   */
  public Autonomous1(ArcadeDrive driveBase) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super( new DriveDistance(driveBase, Constants.auto1Distance, Constants.auto1Speed));

  }
}
