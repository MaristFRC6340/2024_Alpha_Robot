package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.LEDCommand;
import frc.robot.Constants.LEDConstants;

import java.util.Arrays;
import java.util.Random;

public class LEDSubsystem extends SubsystemBase{

    AddressableLED ledStrip1;
    AddressableLED ledStrip2;

    AddressableLEDBuffer ledBuffer;

    public LEDSubsystem(int length){
        ledStrip1 = new AddressableLED(LEDConstants.pwmId);
        //ledStrip2 = new AddressableLED(LEDConstants.pwmId2);

        ledBuffer = new AddressableLEDBuffer(LEDConstants.stripLength);
        ledStrip1.setLength(ledBuffer.getLength());
        //ledStrip2.setLength(ledBuffer.getLength());


        // Set the data
        ledStrip1.setData(ledBuffer);
        ledStrip1.start();
        // ledStrip2.setData(ledBuffer);
        // ledStrip2.start();
    }

    /**
     * Takes in a LED State Object and updates the led strip to match the state
     * @param colors
     */
    public void setColors(LEDState ledState){
        //Iterate through the led state object, and set the led colors in the buffer 
        //Double end contition to ensure index doesn't go out of bounds for ledState and ledBuffer
        for (var i = 0; i < ledState.length && i < ledBuffer.getLength() ; i++) {
          //Get the color at that index in the ledState object
            Color c = ledState.getColorList()[i];
            //set the color to the buffer
            ledBuffer.setRGB(i, (int)(255*c.red), (int)(255*c.green), (int)(255*c.blue));
         }
         //Send the buffer data to the stirp
         ledStrip1.setData(ledBuffer);

    }
   public static class LEDPattern{
        public double restTime;

        private LEDState[] states;
        private int curIndex = 0;


        public LEDPattern(double restTime, LEDState[] states){
            this.restTime=restTime;
            this.states=states;

        }

        public LEDState getCurrentLEDState(){
            LEDState currentState = states[curIndex];
            curIndex++;
            if(curIndex>=states.length)curIndex = 0;
            return currentState;
        }

        public static LEDPattern shiftPattern(LEDState shiftState, double restTime){
            LEDState[] states = new LEDState[shiftState.length];
            LEDState curState = shiftState;
            for(int i = 0; i < shiftState.length; i++){
                states[i] =curState;
                curState=curState.shift(1);
            }
            return new LEDPattern(restTime, states);
        }

        public static LEDPattern flashing(LEDState colorState, double restTime){
            LEDState[] states = new LEDState[2];
            states[0]=colorState;
            states[1] = new LEDState(colorState.length).fill(new Color(0,0,0));
            return new LEDPattern(restTime, states);
        }



    }




   public static class LEDState{
        public int length;
        Color[] leds;
        Random rand;


        public LEDState(int length){
            this.length = length;
            rand = new Random();
            leds = new Color[length];

        }
        public LEDState(Color[] leds){
            this.leds = leds;
            rand = new Random();
            length = leds.length;
        }


        public Color[] getColorList(){
            return leds;
        }
        /**
         * Fills the LEDState with te specified color starting at start and ending at end
         * @param start the first index to be filed with the color
         * @param end - the last index to be filled with the color
         * @param color - the color to set the leds to 
         * @return the LEDStateObject so command can be chained
         */
        public LEDState fill(int start, int end, Color color){
            return fillAlternating(start, end, new Color[]{color});
        }
        /**
         * Overloaded Shortcut to fill(int, int, color)
         * @param start
         * @param color
         * @return
         */
        public LEDState fill(int start, Color color){
            return fill(start, length, color);
        }
        /**
         * Overloaded Shortcut to fill (int, int, color)
         * @param color
         * @return
         */
        public LEDState fill(Color color){
            return fill(0,length, color);
        }
        /**
         * Puts a repeating pattern of colors onto the strip starting at index start and endining at index end inclusive; the colors parameter holds hte colors in the order they will be inserted
         * @param start the first index to start the color pattern at
         * @param end   the last index to have the color pattern
         * @param colors the color pattern to repeat through the strip
         * @return
         */
        public LEDState fillAlternating(int start, int end, Color[] colors){
            int counter = 0;
            for(int i = start; i < end; i++){
                if(counter >= colors.length)counter=0;//Wrapping
                leds[i]=colors[counter];
                counter++;
            }

            return this;
        }
        /**
         * Overloaded method for fillAlternating
         * @param start
         * @param colors
         * @return
         */
        public LEDState fillAlternating(int start, Color[] colors){
            return fillAlternating(start, length, colors);
        }
        /**
         * Overloaded method for fill Alternating
         * @param colors
         * @return
         */
        public LEDState fillAlternating(Color[] colors){
            return fillAlternating(0, length, colors);
        }

        /**
         * Randomly filss the ledState with colors from a color pallete
         */
        public LEDState fillRandom(int start, int end, Color[] pallete){
            int num = rand.nextInt(0,pallete.length);
            for(int i = start; i < end; i++){
                leds[i]=pallete[num];
                num = rand.nextInt(0,pallete.length);
            }
            return this;
        }  
        /**
         * Shifts the LEDS by the specificed amount of spaces; wrapping enabled
         * @param shiftBy
         * @return
         */
        public LEDState shift(int shiftBy){
            Color[] temp = new Color[length];

            for(int i = 0; i < length; i++){
                int newIndex = i+shiftBy;
                if(newIndex >=length){
                    newIndex-=length;
                }
                temp[newIndex] = leds[i];
            }

            
            
            return new LEDState(temp);
            
        }

    }
    



}