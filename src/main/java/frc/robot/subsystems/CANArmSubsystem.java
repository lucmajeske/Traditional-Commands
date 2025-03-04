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
    //Neo's connected to the chassis, moves the arm(height) high - LongHost is the one with the long chain, needs negative power
    private final SparkMax LongHost;
    private final SparkMax ShortSlave;
    // sset the Arm encoder (connected to LongHost)
    private final AbsoluteEncoder ArmInitial;

    private final PIDController ArmPID = new PIDController(ArmConstants.Arm_P, ArmConstants.Arm_I, ArmConstants.Arm_D);

        public CANArmSubsystem(){
            //create motors for arm host movement
            LongHost = new SparkMax(ArmConstants.Left_Back_ID, MotorType.kBrushless);
            ShortSlave = new SparkMax(ArmConstants.Right_Back_ID,MotorType.kBrushless);
            //create new encoder for arm movement
            ArmInitial =  LongHost.getAbsoluteEncoder();

            //Set Start CAN timeout,should(??????) only be called once, so a ong imeout is fine
            LongHost.setCANTimeout(250);
            ShortSlave.setCANTimeout(250);

            //set nominal voltage - 12v allows higher speed, less consistency
            //we will try for 10 volts,  tis mean it will usually stay at ten, consistently
            SparkMaxConfig config = new SparkMaxConfig();
            config.voltageCompensation(ArmConstants.NOMINAL_VOLTAGE);
            config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);

            //set short slave to follow long host, but invert
            config.follow(LongHost, true);
            ShortSlave.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        // finds and sets the setpont for your arm movement
        public void setArmGoal(double ArmGoal){
            SmartDashboard.putNumber("Arm Position Demand", ArmGoal);
            ArmPID.setSetpoint(ArmGoal);
            
        }
        public double getCurrentArmPOS(){
            return ArmInitial.getPosition();

        }


            @Override
            public void periodic(){
                double currentArmPOS = getCurrentArmPOS();
                double armPower = ArmPID.calculate(currentArmPOS);
                
                SmartDashboard.putNumber("High Arm POS", currentArmPOS);
                SmartDashboard.putNumber("High Arm Power", armPower);
                if (RobotState.isEnabled()){
                    LongHost.set(-armPower);
                }


            }




        }
