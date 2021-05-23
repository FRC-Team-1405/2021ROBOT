package frc.robot.lib;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomPIDController extends PIDController {

    public CustomPIDController(String name, double kp, double ki, double kd) {
        super(kp, ki, kd);

        SmartDashboard.putData(this);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("PIDController");
      builder.addDoubleProperty("p", this::getP, this::setP);
      builder.addDoubleProperty("i", this::getI, this::setI);
      builder.addDoubleProperty("d", this::getD, this::setD);
      builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
      builder.addDoubleProperty("position", this::getPositionError, null);
      builder.addDoubleProperty("velocity", this::getVelocityError, null);
    }
  
}
