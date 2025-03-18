package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.RobotState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.AbsoluteEncoder;

public class CANArmSubsystem extends SubsystemBase{
    //Neo's connected to the chassis, moves the arm(height) high - ArmHost is the one with the long chain, needs negative power
    private final SparkMax ArmHost;
    private final SparkMax ArmSlave;

    // Set the arm encoder (connected to ArmHost)
    private final AbsoluteEncoder ArmInitial;
    public double ArmHomeBase;

    private final PIDController ArmPID = new PIDController(ArmConstants.Arm_P, ArmConstants.Arm_I, ArmConstants.Arm_D);

    public CANArmSubsystem() {
        ArmPID.setTolerance(0.1);
        // Initialize sensor for resetting encoder;
       
        // Initialize arm motors;
        ArmHost = new SparkMax(ArmConstants.Left_Back_ID, MotorType.kBrushless);
        ArmSlave = new SparkMax(ArmConstants.Right_Back_ID, MotorType.kBrushless);
        // Initializze arm encoder;
        ArmInitial = ArmHost.getAbsoluteEncoder();

        // Set Start CAN timeout,should(??????) only be called once, so a ong imeout is fine
        ArmHost.setCANTimeout(250);
        ArmSlave.setCANTimeout(250);

        //set nominal voltage - 12v allows higher speed, less consistency
        //we will try for 10 volts,  tis mean it will usually stay at ten, consistently
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(ArmConstants.NOMINAL_VOLTAGE);
        config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);

        //set short slave to follow long host, but invert
        config.follow(ArmHost, true);
        ArmSlave.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //sets the arm goal to where it currently sits, and passes that to the PID
        
        setArmGoal(0);
            ArmHomeBase = getCurrentArmPOS();
            SmartDashboard.putNumber("Arm Home Base", ArmHomeBase);

    }
    
    // finds and sets the setpont for your arm movement
    public void setArmGoal(double ArmGoal){
        SmartDashboard.putNumber("Arm Position Demand", (ArmHomeBase + ArmGoal));
        ArmPID.setSetpoint(ArmGoal + ArmHomeBase);
    }

    public double getCurrentArmPOS() {
        // Return the position from the arm encoder;
        return ArmInitial.getPosition();
    }

    @Override
    public void periodic(){
        // If the sensor is triggered, reset the arm encoder;
//        if (sensor.get()) {
//            ArmInitial.setPosition(0);
//        }

        double currentArmPOS = getCurrentArmPOS();
        double armPower = ArmPID.calculate(currentArmPOS);
        double armVoltage = ArmHost.getOutputCurrent();

        // Put SmartDashboard entries;
        SmartDashboard.putNumber("NEO Volt",armVoltage);
        SmartDashboard.putNumber("High Arm POS", currentArmPOS);
        SmartDashboard.putNumber("High Arm Power", armPower);

        // If the robot is enabled, write power to the arm;
        if (RobotState.isEnabled()){
            ArmHost.set(-0.25*armPower);
            
        } 

            
    }
}
