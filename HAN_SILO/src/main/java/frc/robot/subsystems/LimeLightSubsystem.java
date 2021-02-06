package frc.robot.subsystems;

import frc.robot.vision.LimeTargetInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem stores the last target coordinates and allows for easy control over the LED
 */
public class LimeLightSubsystem extends SubsystemBase {

    /**
     * The table that contains all controls and outputs for the Limelight
     */
    private NetworkTable limeTable;
    /**
     * How off the X axis (target coordinates) the target is from the crosshair in degrees
     */
    private NetworkTableEntry limeTX;
    /**
     * How off the Y axis (target coordinates) the target is from the crosshair in degrees
     */
    private NetworkTableEntry limeTY;
    /**
     * How much of the image the target covers (0%-100%)
     */
    private NetworkTableEntry limeArea;
    /**
     * The horizontal (width) sidelength of the rough bounding box (0-320 pixels)
     */
    private NetworkTableEntry limeHoriz;
    /**
     * The vertical (length) sidelength of the rough bounding box (0-320 pixels)
     */
    private NetworkTableEntry limeVert;
    /**
     * Indicates if the Limelight has a target. 1 for yes, 0 for no.
     */
    private NetworkTableEntry limeValidTargets;
    /**
     * Control for LED. 1 is off, 2 is blink, 3 is on, and 0 is default for pipeline
     */
    private NetworkTableEntry limeLED;

    /**
     * The last target that was reported by the Limelight
     */
    private LimeTargetInfo lastTarget = LimeTargetInfo.empty;

    /**
     * Keeps track of how many systems are requesting the LED. Each system should be turning off the LED when they are done.
     * e.g. VisionRotateCommand turns the LED on while the command is active and then turns it off when deactivated.
     */
    private int ledQueries = 0;

    public LimeLightSubsystem() {
        //Set tables for easy getting
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeTX = limeTable.getEntry("tx");
        limeTY = limeTable.getEntry("ty");
        limeArea = limeTable.getEntry("ta");
        limeHoriz = limeTable.getEntry("thor");
        limeVert = limeTable.getEntry("tvert");
        limeValidTargets = limeTable.getEntry("tv");
        limeLED = limeTable.getEntry("ledMode");

        //Keep the light off so we don't blind unfortunate spectators
        disableLED(false);
    }

    @Override
    public void periodic() {
        //Report target when one is valid
        if(hasTarget()){
            lastTarget = new LimeTargetInfo(limeTX.getDouble(0), limeTY.getDouble(0), Timer.getFPGATimestamp(), 
                                            limeArea.getDouble(0), limeVert.getDouble(0), limeHoriz.getDouble(0));
        }

        UpdateSmartDashboard();
    }

    /**
     * Updates the Vis_HasTarget and Vis_TargetPos SmartDashboard entries
     */
    private void UpdateSmartDashboard(){
        SmartDashboard.putBoolean("Vis_HasTarget", hasTarget());
        SmartDashboard.putString("Vis_TargetPos", hasTarget() ? lastTarget.getY()+", "+lastTarget.getZ() 
                                                                : "N/A");
    }

    /**
     * Gets the last target reported by the Limelight
     * @return The last target reported by the Limelight
     */
    public LimeTargetInfo getLastTarget(){
        return lastTarget;
    }

    /**
     * Checks if the Limelight has any valid targets
     * @return Whether or not the Limelight has any valid targets
     */
    public boolean hasTarget(){
        return limeValidTargets.getDouble(0) > 0;
    }

    /**
     * Turns on the LED
     */
    public void enableLED(){
        limeLED.setNumber(3);
        ledQueries++;
    }

    /**
     * Notes that your system is done with the LED. If all systems are done with the LED, it is turned off
     */
    public void disableLED(){
        disableLED(true);
    }

    /**
     * Notes that your system is done with the LED. If all systems are done with the LED, it is turned off
     * @see disableLED()
     * @param trackQuery Whether or not to actually track the system that queried the disable. Setting to false typically forces the LED to turn off.
     */
    private void disableLED(boolean trackQuery){
        if(trackQuery){
            ledQueries--;
        }

        if(ledQueries <= 0){
            //limeLED.setNumber(1);
            limeLED.setNumber(3);
        }
    }
}
