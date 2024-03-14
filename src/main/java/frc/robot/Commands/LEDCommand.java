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


    public static LEDPattern blueFlashing(){
        LEDState state = new LEDState(new Color[]{new Color(0,0,255), new Color(0,0,0)});
        return new LEDPattern(0, new LEDState[]{state});
    }
    public static LEDPattern redFlashing(){
        LEDState state = new LEDState(new Color[]{new Color(255,0,0), new Color(255,0,0)});
        return new LEDPattern(0, new LEDState[]{state});
    }
    //note detected
    public static LEDPattern orange(){
        LEDState state = new LEDState(new Color[]{new Color(255, 165, 0)});
        return new LEDPattern(0, new LEDState[]{state});
    }

    public static LEDPattern ambientManatee(){
        LEDState state = new LEDState(new Color[]{new Color(255,255,255), new Color(99,194,210), new Color(24,75,89), new Color(13,50,63)});
        return LEDPattern.shiftPattern(state, 1);
    }

}