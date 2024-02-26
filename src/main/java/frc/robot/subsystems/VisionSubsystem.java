package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    final I2C.Port i2cPort0 = I2C.Port.kOnboard;//i2c port 
    final I2C.Port i2cPort1 = I2C.Port.kMXP;//i2c port on the mxp

    final ColorSensorV3 sensor0 = new ColorSensorV3(i2cPort0);
    final ColorSensorV3 sensor1 = new ColorSensorV3(i2cPort1);



    @Override
    public void periodic() {
        Color detectedColor = sensor0.getColor();
        //Printing out test values for detecting note intake
        SmartDashboard.putNumber("GREEN", detectedColor.green);
        SmartDashboard.putNumber("BLUE", detectedColor.blue);
        SmartDashboard.putNumber("RED", detectedColor.red);
        SmartDashboard.putNumber("IR", sensor0.getIR());
        SmartDashboard.putNumber("Proximity", sensor0.getProximity());

        SmartDashboard.putString("COLOR DETECTED", detectedColor.toHexString());

    }



}
