package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Stretcher.StretcherSubsystem;


public class GroundIntakeSequence extends Command {
    enum IntakeState {
        //STRETCH,
        INTAKE,
        STOP
    }
    
    private IntakeState state;
    private ImprovedCommandXboxController driveController = RobotContainer.driveController;

    /**This works as an emergency continue button, in case the sensor is <b>not</b> working properly. */
    private Button m_toggleButton;
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    StretcherSubsystem stretcher = StretcherSubsystem.getInstance();

    public GroundIntakeSequence(Button toggleButton) {
        addRequirements(intaker, stretcher);
        m_toggleButton = toggleButton;
    }

    @Override
    public void initialize() {
        stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
        intaker.setRPS(0);
        state = IntakeState.INTAKE;
    }

    @Override
    public void execute() {
        switch (state) {
            // case STRETCH:
            //     stretch();
            //     break;
            case INTAKE:
                intake();
                break;
            case STOP:
                break;
        }
    }

    // private void stretch() {
    //     stretcher.setPosition(Constants.StretcherConstants.ExtendedPosition);
    //     intaker.setRPS(Constants.IntakerConstants.IntakingRPS / 2.);
    //     state = IntakeState.INTAKE;
    // }

    private void intake() {
        stretcher.setPosition(Constants.StretcherConstants.ExtendedPosition);
        intaker.setRPS(Constants.IntakerConstants.IntakingRPS);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.stop();
        stretcher.setPosition(Constants.StretcherConstants.RetractedPosition);
        state = IntakeState.STOP;
    }

    @Override
    public boolean isFinished() {
        return state == IntakeState.STOP;
    }
}
