package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import frc.robot.Constants;

public class IntakePneumaticsSubsystem extends SubsystemBase {
    private final DoubleSolenoid mArms;
    private boolean mExtended = false;

    public IntakePneumaticsSubsystem(DoubleSolenoid shifter) {
        mArms = shifter;
        mExtended = false;
        mArms.set(kReverse);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private void extend() {
        mArms.set(kForward);
        mExtended = true;
    }

    private void retract() {
        mArms.set(kReverse);
        mExtended = false;
    }
    
    public void toggleIntake() {
        if (mExtended) {
            retract();
        }
        else {
            extend();
        }
    }

    public static IntakePneumaticsSubsystem create() {
        return new IntakePneumaticsSubsystem(new DoubleSolenoid(Constants.INTAKE.UP_SOLENOID_ID, Constants.INTAKE.DOWN_SOLENOID_ID));
    }
}

