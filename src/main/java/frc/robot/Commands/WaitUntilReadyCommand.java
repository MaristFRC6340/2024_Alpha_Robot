package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;

public class WaitUntilReadyCommand extends Command{

	BooleanSupplier shoulderUp;
	
	NetworkTable limTable;
	NetworkTableEntry ty;
	NetworkTableEntry tx;
	NetworkTableEntry ledMode;
	public WaitUntilReadyCommand(BooleanSupplier shoulderUp) {

		this.shoulderUp = shoulderUp;
				
		limTable = NetworkTableInstance.getDefault().getTable("limelight");
		tx = limTable.getEntry("tx");
		ty = limTable.getEntry("ty");
		ledMode = limTable.getEntry("ledMode");
	}
	
	double lastTY;
	@Override
	public void initialize() {
		ledMode.setDouble(3.0); //ON
		lastTY = ty.getDouble(0);
	}
	
	
	
	
	/**
	 * TY increases as the robot moves towards the april tag, and decreases as the robot moves away from the april tag
	 * If the current TY is greater than a given number, while the last TY is less than the given number, the robot
	 * must have been moving forward for the TY to increase past a treshhold. The opposite is also true, that if the
	 * current TY is less than a given value and the lasty TY is greater than the given value, the robot must have
	 * passed the value while moving backwards. This logic allows us to create two distinct lines: One that will end this
	 * command when the robot passes it moving forwards, and another distinct line that will end this command when the robot
	 * passes it moving backwards. Thus, it will always end the command between the two markers, in the sweetspot. 
	 * This logic is here to deal with the case in which the robot does not have a frame of the limelight in which
	 * ty is within the given range (between the forward and backward thresholds). It will return true as soon as it sees
	 * the april tag on the other side of that line.
	 * 
	 * Sorry for the paragraph, wanted to spill my thoughts before I forgot how this works. This command should be 
	 * followed in a sequential command group immediately by a LaunchNoteCommand to launch the note the instant this
	 * command ends.
	 * 
	 * This method will be run every loop, so lastTY is updated properly
	 * 
	 * In terms of constants, I would start with kCloseForward and kCloseBackward being +/- 1 of the current value
	 * of ty for close shots. The same can be done with kFarForward and kFarBackward and the current value of ty
	 * for far shots. A good starting value of both TX tolerances would be 1, but this can be tuned depending on 
	 * how accurate our target locking pid is. Lower/Raise these values until the robot is consistantly making shots
	 * while target locking is active.
	 * 
	 * In terms of implementing target locking, just copy our drive command. In the execute, put an if statement for
	 * whether or not tx and ty exists, similar to what you see here. If they don't, drive normally with rightX for
	 * rotational speed. If they do exist, use the aim to april tag command math for rot speed.
	 * 
	 * 
	 * @return
	 */
	@Override
	public boolean isFinished() {
		double currTY = ty.getDouble(0);
		if(tx.exists() && ty.exists()) {
			if(shoulderUp.getAsBoolean()) {
				if(currTY > LimelightConstants.kCloseForwardThreshold &&
						lastTY <LimelightConstants.kCloseForwardThreshold &&
						Math.abs(tx.getDouble(0)) < LimelightConstants.kCloseTXTolerance) 
				{
					return true;
				}
				else if (currTY < LimelightConstants.kCloseBackwardThreshold && 
						lastTY > LimelightConstants.kCloseBackwardThreshhold && 
						Math.abs(tx.getDouble(0))<LimelightConstants.kCloseTXTolerance)
						 {
					return true;
				}
			}
			else {
				if(currTY > LimelightConstants.kFarForwardThreshold &&
						lastTY <LimelightConstants.kFarForwardThreshold &&
						Math.abs(tx.getDouble(0)) < LimelightConstants.kFarTXTolerance)
				{
					return true;
				}
				else if (currTY < LimelightConstants.kFarBackwardThreshold && 
						lastTY > LimelightConstants.kFarBackwardThreshold && 
						Math.abs(tx.getDouble(0))<LimelightConstants.kFarTXTolerance)
						 {
					return true;
				}
			}
			lastTY = currTY;
		}
		return false;
	}
	
}