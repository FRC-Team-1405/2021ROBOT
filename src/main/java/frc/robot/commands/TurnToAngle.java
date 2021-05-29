// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.SwerveDriveBase;

/** 
 * Turn until the sensor angle in degrees is less than 3.0 
*/
public class TurnToAngle extends DriveByAngle {
    public TurnToAngle(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier speedLimit, DoubleSupplier sensor, SwerveDriveBase drive){
        super(forward, strafe, speedLimit, sensor, drive);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(sensor.getAsDouble()) < 3.0;
    }
}
