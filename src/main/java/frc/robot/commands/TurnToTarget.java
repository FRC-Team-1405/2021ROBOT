/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends SequentialCommandGroup {
  /**
   * Creates a new TurnToTarget.
   */
  
  public TurnToTarget(Shooter shooter, ArcadeDrive drivebase, Limelight limelight) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super( 
      // new FunctionalCommand( ()-> { if(!shooter.hasTarget() && shooter.tracking){
      //                                     shooter.turnToGoal(drivebase);
      //                                   }},
      //                             ()-> { },
      //                             (interrupted) -> {},
      //                             ()-> { return (shooter.turretTurnIsComplete() || !shooter.tracking); } ),

          new FunctionalCommand( ()-> {if(shooter.tracking){
                                        limelight.setPipeline((byte) 7);
                                        limelight.setLED((byte) 3);
                                        shooter.turnTurret();
                                      }},
                                 ()-> {if(shooter.turretTurnIsComplete() && shooter.tracking){
                                        shooter.turnTurret();
                                      } },
                                 (interrupted) -> {},
                                 () -> {return !shooter.tracking;} )
    );
  };
}
