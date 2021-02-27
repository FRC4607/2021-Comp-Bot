package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import frc.robot.Constants;

public class ShifterSubsystem extends SubsystemBase {
    private final DoubleSolenoid mShifter;
    private boolean mHighGear = false;

    public ShifterSubsystem(DoubleSolenoid shifter) {
        mShifter = shifter;
        mHighGear = false;
        mShifter.set(kForward);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("low Gear", mHighGear);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void lowGear() {
        mShifter.set(kForward);
        mHighGear = false;
    }

    public void highGear() {
        mShifter.set(kReverse);
        mHighGear = true;
    }
    
    public void switchGears() {
        if (mHighGear) {
            lowGear();
        }
        else {
            highGear();
        }
    }

    public static ShifterSubsystem create() {
        return new ShifterSubsystem(new DoubleSolenoid(Constants.DRIVETRAIN.LOW_GEAR_SOLENOID_ID, Constants.DRIVETRAIN.HIGH_GEAR_SOLENOID_ID));
    }
}
