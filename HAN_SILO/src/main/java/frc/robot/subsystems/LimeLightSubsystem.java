package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.vision.LimeTargetInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Example code for limelight
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
*/

/**
 * This subsystem stores the last target coordinates and allows for easy control over the LED
 */
public class LimeLightSubsystem extends SubsystemBase {

    private NetworkTable limeTable;
    private NetworkTableEntry limeTX;
    private NetworkTableEntry limeTY;
    private NetworkTableEntry limeArea;
    private NetworkTableEntry limeHoriz;
    private NetworkTableEntry limeVert;
    private NetworkTableEntry limeValidTargets;
    private NetworkTableEntry limeLED;

    private LimeTargetInfo lastTarget = LimeTargetInfo.empty;

    private int ledQueries = 0;

    public LimeLightSubsystem() {
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeTX = limeTable.getEntry("tx");
        limeTY = limeTable.getEntry("ty");
        limeArea = limeTable.getEntry("ta");
        limeHoriz = limeTable.getEntry("thor");
        limeVert = limeTable.getEntry("tvert");
        limeValidTargets = limeTable.getEntry("tv");
        limeLED = limeTable.getEntry("ledMode");

        disableLED(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vis_HasTarget", hasTarget());
        if(hasTarget()){
            lastTarget = new LimeTargetInfo(limeTX.getDouble(0), limeTY.getDouble(0), Timer.getFPGATimestamp(), 
                                            limeArea.getDouble(0), limeVert.getDouble(0), limeHoriz.getDouble(0));
            SmartDashboard.putString("Vis_TargetPos", lastTarget.getY()+", "+lastTarget.getZ());
        } else {
            SmartDashboard.putString("Vis_TargetPos", "N/A");
        }
    }

    public LimeTargetInfo getLastTarget(){
        return lastTarget;
    }

    public boolean hasTarget(){
        return limeValidTargets.getDouble(0) > 0;
    }

    public void enableLED(){
        limeLED.setNumber(3);
        ledQueries++;
    }

    public void disableLED(){
        disableLED(true);
    }

    private void disableLED(boolean trackQuery){
        if(trackQuery){
            ledQueries--;
        }

        if(ledQueries <= 0){
            limeLED.setNumber(1);
        }
    }
}
