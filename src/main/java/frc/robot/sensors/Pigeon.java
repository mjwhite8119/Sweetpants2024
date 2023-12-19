package frc.robot.sensors;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;

/**
 * Utility class responsible for the gyro
 */
public class Pigeon{
    private Pigeon2 m_pigeon;

    public Pigeon(){
        m_pigeon = new Pigeon2(Constants.CANBusIDs.kPigeonIMU);
    }

    public double getYaw(){
        return m_pigeon.getYaw().getValue();
    }

    public void resetGyro(){
        m_pigeon.setYaw(0);
    }
}
