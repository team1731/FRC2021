/*
    Limelight 2+ Specs:

    RES: 320 x 240 pixels
    FOV: 59.6 x 49.7 degrees
*/

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
     * The current vision pipeline the Limelight is set to
     */
    private NetworkTableEntry limePipeline;
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

    private NetworkTableEntry[] limeRawX = new NetworkTableEntry[3];
    private NetworkTableEntry[] limeRawY = new NetworkTableEntry[3];
    private NetworkTableEntry[] limeRawArea = new NetworkTableEntry[3];

    /**
     * The last target that was reported by the Limelight
     */
    private LimeTargetInfo[] lastTarget = new LimeTargetInfo[] {
        LimeTargetInfo.empty,
        LimeTargetInfo.empty,
        LimeTargetInfo.empty
    };

    /**
     * Keeps track of how many systems are requesting the LED. Each system should be turning off the LED when they are done.
     * e.g. VisionRotateCommand turns the LED on while the command is active and then turns it off when deactivated.
     */
    private int ledQueries = 0;

    /**
     * The different configured pipelines on the Limelight
     */
    public enum DetectionMode {
        PowerPort,
        PowerCells
    }

    /**
     * The current Limelight pipeline
     */
    private DetectionMode currDetectionMode = DetectionMode.PowerPort;

    public LimeLightSubsystem() {
        //Set tables for easy getting
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        limePipeline = limeTable.getEntry("pipeline");
        limeTX = limeTable.getEntry("tx");
        limeTY = limeTable.getEntry("ty");
        limeArea = limeTable.getEntry("ta");
        limeHoriz = limeTable.getEntry("thor");
        limeVert = limeTable.getEntry("tvert");
        limeValidTargets = limeTable.getEntry("tv");
        limeLED = limeTable.getEntry("ledMode");
        for(int i = 0; i < 3; i++){
            limeRawX[i] = limeTable.getEntry("tx"+i);
            limeRawY[i] = limeTable.getEntry("ty"+i);
            limeRawArea[i] = limeTable.getEntry("ta"+i);
        }

        limePipeline.setNumber(2);

        //Keep the light off so we don't blind unfortunate spectators
        disableLED(false);
    }

    @Override
    public void periodic() {
        //Report target when one is valid
        if(hasTarget()){
            if(currDetectionMode == DetectionMode.PowerPort){
                lastTarget[0] = new LimeTargetInfo(limeTX.getDouble(0), limeTY.getDouble(0), 
                                                limeArea.getDouble(0), limeVert.getDouble(0), limeHoriz.getDouble(0), 
                                                Timer.getFPGATimestamp());
            } else if(currDetectionMode == DetectionMode.PowerCells){
                for(int i = 0; i < lastTarget.length; i++){
                    lastTarget[i] = new LimeTargetInfo(limeRawX[i].getDouble(0), limeRawY[i].getDouble(0), 
                                                        limeRawArea[i].getDouble(0), 0, 0,
                                                        Timer.getFPGATimestamp());
                }
            }
        }

        UpdateSmartDashboard();
    }

    /**
     * Updates the Vis_HasTarget and Vis_TargetPos SmartDashboard entries
     */
    private void UpdateSmartDashboard(){
        SmartDashboard.putBoolean("Vis_HasTarget", hasTarget());
        SmartDashboard.putString("Vis_TargetPos", hasTarget() ? lastTarget[0].getY()+", "+lastTarget[0].getZ() 
                                                                : "N/A");
    }

    /**
     * Gets the last target reported by the Limelight
     * @return The last target reported by the Limelight
     */
    public LimeTargetInfo getLastPortPos(){
        return lastTarget[0];
    }

    /**
     * Checks if the Limelight has any valid targets
     * @return Whether or not the Limelight has any valid targets
     */
    public boolean hasTarget(){
        return limeValidTargets.getDouble(0) > 0;
    }

    /**
     * Sets the Limelight pipeline
     */
    public void SetDetectionMode(DetectionMode detectionMode){
        currDetectionMode = detectionMode;

        switch(detectionMode){
            case PowerPort:
                limePipeline.setNumber(2);
                break;
            case PowerCells:
                limePipeline.setNumber(1);
                break;
        }
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

        if(ledQueries <= 0 && trackQuery){
            limeLED.setNumber(0);
        }
    }
}