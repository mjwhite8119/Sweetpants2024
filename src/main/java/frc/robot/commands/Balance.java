package frc.robot.commands;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Balance extends Command {
    private Drivetrain drivetrain;
    public double maxSpeed = 1;
    
    public Balance(Drivetrain drivetrain, double p, double i, double d) {
        this.addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }
    
    @Override
    public void initialize() {
        drivetrain.halt();

        // for(TalonFX fx : new TalonFX[] { drivetrain.leftLeader, drivetrain.rightLeader }) {
        //     // fx.selectProfileSlot(0, 0);
        //     // fx.set(0);

        //     // Todo: feed in measured value
        // }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.halt();
    }
}
