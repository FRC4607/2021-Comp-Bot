package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendClimber extends CommandBase{
    private final ClimberSubsystem mClimber;
    private final IntakeSubsystem mIntake;

    public ExtendClimber(ClimberSubsystem climber, IntakeSubsystem intake) {
        mClimber = climber;
        mIntake = intake;
        addRequirements(mClimber, mIntake);
    }

    @Override
    public void initialize() {
        //CPA 04 03 2021 Commmand the Clmber to Unlock
        /*if(DriverStation.getInstance().getMatchTime()>??){
            mClimber.unlock();
        }*/
        mClimber.unlock();
        mIntake.retract();
        //mClimber.toggle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
