package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeHoldClosed extends Command {

    public final Intake intake;

    public IntakeHoldClosed(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(Constants.Intake.HOLD_CLOSED_POWER);
    }

    @Override
    public void execute() {
      
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
