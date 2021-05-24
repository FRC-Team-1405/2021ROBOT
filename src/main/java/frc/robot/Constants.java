/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants { 
    public final static int PDP = 0;    

    public final static class SwerveBase {
        public final static int driveFrontLeft = 1;
        public final static int driveFrontRight = 2;
        public final static int driveBackLeft = 3;
        public final static int driveBackRight = 4; 

        public final static int azimuthFrontLeft = 21;
        public final static int azimuthFrontRight = 22;
        public final static int azimuthBackLeft = 23;
        public final static int azimuthBackRight = 24; 

        public final static Translation2d frontLeftLocation = new Translation2d(0.330, -0.279);
        public final static Translation2d frontRightLocation = new Translation2d(0.330, -0.279);
        public final static Translation2d backLeftLocation = new Translation2d(-0.330, -0.279);
        public final static Translation2d backRightLocation = new Translation2d(-0.330, 0.279); 

        public final static double maxAcceleration = 1; 
        public final static double maxSpeed = 1;  
        public final static double maxAngularAccelerartion = Math.PI; 
        public final static double maxAngularSpeed = Math.PI;  
    }

    public static class Hood {
        public final static int maxError = 100;
        public final static int maxAngle = 36000;
        public final static int TrenchToTarget = 850;
        public final static int LoadingToTrench = 750;
    }

    public final static int intakeTalon = 5;
    public final static int intakeDeploy = 6;
    public final static int indexerid = 7;
    public final static int triggerid = 8;
    public final static int shooterMaster = 9;
    public final static int shooterSlave = 10; 
    public final static int shooterAngle = 11;

    public final static int leftScissor = 12;
    public final static int rightScissor = 13;

    public final static int swerveIntakeTalon = 15; 
    public final static int swerveIntakeDeployTalon = 16; 

    public final static int leftActuatorId = 1;
    public final static int rightActuatorId = 2;

    public final static double deadBand = 0.033; 
    public final static double swerveDeadBand = 0.033;
    public final static double scissorDeadband = 0.2;

    public final static int leftFrontSwitchid = 0;
    public final static int rightFrontSwitchid = 1;
    public final static int leftBackSwitchid = 2;
    public final static int rightBackSwitchid = 3;

    //Led strip length 
    public final static int ledLength =20; 

    public final static int pilot = 0;
    public final static int operator = 1; 
    public final static int demoController = 5;

    public final static int lidarBufferSize = 20;

    public final static double auto1Speed = -0.25;
    public final static double auto1Distance = 0.5;//distance in meters

    // meters
    public final static double robotWidth = 6.9342;
    public final static double goalX = 2.404364;

    public class ShooterConstants {
        public static final int acceptableError = 250;
    } 

     //TrajectoryDrive constants
    public static final double kTrackwidthMeters = 0.5461;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
	public static final double maxVelocity = 1;
	public static final double maxAcceleration = 0.5;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static int maxFlywheelError = 250;
    public static int closeFire = 1;
    public static int farFire = 4;
    public static int closeFire2 = 2;
    public static int farFire2 = 3;

    public static class TurnPID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class DriveStraightPID {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
    }

    public static class IntakeConstants {
        public final static int DEPLOY_POSITION = -1500;
        public final static int RETRACT_POSITION = -250;
        public static double SPEED = -0.8;
    }

    public static class VelocityConversions{
        public final static double SensorTimePerSec = 10;
        public final static double WheelCircumference = (6 * Math.PI);
        public final static double SensorUnitsPerRotation = 2048; 
        public final static double SwerveSensorUnitsPerRotation = 2048; 
        public final static double SwerveWheelCircumference = (4 * Math.PI); 
        public final static double DriveBaseReduction = 8.68;
        public final static double InchesPerMeter = 39.37; 
        public final static double ScaleFactor = 1.1; 
        public final static double SwerveMetersPerSecondToSensorVelocity = ((SensorUnitsPerRotation * (1 / SwerveWheelCircumference) * InchesPerMeter)) * 6 / SensorTimePerSec;                                  
        public final static double SensorVelocityToMetersPerSecond = (1.0 / SensorUnitsPerRotation) * (1.0 / DriveBaseReduction) * WheelCircumference * (1.0 / InchesPerMeter); 
        public final static double SwerveSensorVelocityToMetersPerSecond = (((1.0 / SwerveSensorUnitsPerRotation) * SwerveWheelCircumference * (1 / InchesPerMeter)) / 6) * SensorTimePerSec;  
        public final static double SensorToMetersCIMBot = 1/SensorUnitsPerRotation * WheelCircumference * (1.0 / InchesPerMeter) / 2.0; 
        public final static double MetersToSensor = (1.0 / SensorVelocityToMetersPerSecond);
        public final static double FVelocityToMetersPerSecond = (SensorTimePerSec * WheelCircumference)/(SensorUnitsPerRotation * DriveBaseReduction * InchesPerMeter); 
        public final static double MetersPerSecondToVelocity = 1.0/FVelocityToMetersPerSecond; 


    }
    public class VelocityPID{
        public final static double VelocityP = 0.5;
        public final static double VelocityI = 0;
        public final static double VelocityD = 0;
    }

    public class LimelightConfig {
        public final static byte DrivePipeline   = 0;
        public final static byte TargetPipeline  = 3;
        public final static byte BallPipelien    = 9;
    }
}
