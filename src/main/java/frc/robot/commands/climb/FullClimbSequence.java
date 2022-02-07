package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArm;
import static frc.robot.Constants.ClimbConstants.*;

public class FullClimbSequence extends SequentialCommandGroup{
    public FullClimbSequence(ClimbArm arm){
        addCommands(
            //first bar
            new ClimbPull(arm),

            //move to second bar
            new ExtendClimbArm(HOOK_RELEASE_DISTANCE, arm),
            new DropArmForward(arm),
            new ExtendClimbArm(FULL_EXTENSION_DISTANCE, arm),
            new RetractIntoBar(arm),
            new ClimbPull(arm),

            //move to third bar
            new ExtendClimbArm(HOOK_RELEASE_DISTANCE, arm),
            new DropArmForward(arm),
            new ExtendClimbArm(FULL_EXTENSION_DISTANCE, arm),
            new RetractIntoBar(arm),
            new ClimbPull(arm)
        ); 
    }
}
