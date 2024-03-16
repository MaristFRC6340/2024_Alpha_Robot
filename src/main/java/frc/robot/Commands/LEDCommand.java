package frc.robot.Commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDPattern;
import frc.robot.subsystems.LEDSubsystem.LEDState;;

public class LEDCommand extends Command{
    
    LEDPattern pattern;
    LEDSubsystem leds;
    double timeout;
    double startTimeNano = 0;
    public LEDCommand(LEDPattern pattern, LEDSubsystem leds){
        this.pattern = pattern;
        this.leds = leds;
        this.timeout = pattern.restTime;
        addRequirements(leds);
    }
    @Override
    public void initialize(){
        startTimeNano = System.nanoTime();
        leds.setColors(pattern.getCurrentLEDState());
    }
    @Override
    public void execute(){
        double timeElapsed = (System.nanoTime()-startTimeNano)/1000000.0;
        if(timeElapsed>timeout){
            leds.setColors(pattern.getCurrentLEDState());
            startTimeNano = System.nanoTime();
        }


    }
    @Override
    public boolean isFinished(){
        return false;
    }


    

}