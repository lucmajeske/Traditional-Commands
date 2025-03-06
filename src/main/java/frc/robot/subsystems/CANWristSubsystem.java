package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import com.revrobotics.AbsoluteEncoder;

public final class CANWristSubsystem extends SubsystemBase{
    //Neo's Connected to the head
    private final SparkMax WristHost;
    private final SparkMax WristSlave;
    // create new encoder for wrist movement (connected to wrist host)
    private final AbsoluteEncoder WristInitial;
    private final DigitalInput sensor;
    private double encOffset;
    
    // create Wrist PID controller
    private final PIDController WristPID = new PIDController(WristConstants.Wrist_P, WristConstants.Wrist_I, WristConstants.Wrist_D);
        // wrist subsystem constructor, runs on boot
        public CANWristSubsystem(){
            sensor = new DigitalInput(0);

            //create motors for wrist movement
            WristHost = new SparkMax(WristConstants.Right_Front_ID, MotorType.kBrushless);
            WristSlave = new SparkMax(WristConstants.Left_Front_ID, MotorType.kBrushless);
            //create an encoder to track the host
            WristInitial = WristHost.getAbsoluteEncoder();
            encOffset = 0;

            //Set CAN Timeout - only runs once 
            WristHost.setCANTimeout(250);
            WristSlave.setCANTimeout(250);

            //set nominal voltage - 12v allows higher speed, less consistency
            //we will try for 10 volts,  tis mean it will usually stay at ten, consistently
            SparkMaxConfig config = new SparkMaxConfig();
            config.voltageCompensation(WristConstants.NOMINAL_VOLTAGE);
            config.smartCurrentLimit(WristConstants.CURRENT_LIMIT);

            //set WristSlave to follow WristHost, Inverting
            config.follow(WristHost, true);
            WristSlave.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            setWristGoal(getCurrentWristPOS());
        }

        //Finds the setpoint fr wrist movement
        public void setWristGoal(double WristGoal){
            SmartDashboard.putNumber("Wrist Position Demand", WristGoal);
            WristPID.setSetpoint(WristGoal);
        }

        public double getCurrentWristPOS(){
            return WristInitial.getPosition();
        }
      
        @Override
        public void periodic() {
            if (sensor.get())
                encOffset += getCurrentWristPOS();

            double currentWristPOS = getCurrentWristPOS();
            double WristPower = WristPID.calculate(currentWristPOS);
            double wristVoltage = WristHost.getOutputCurrent();
            SmartDashboard.putNumber("NEO Wrist Volt", wristVoltage);
            
            SmartDashboard.putNumber("High Wrist POS", currentWristPOS);
            SmartDashboard.putNumber("High Wrist Power", WristPower);
            if (RobotState.isEnabled()){
                WristHost.set(-WristPower);
            }
    }
}
