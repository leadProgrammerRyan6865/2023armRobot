// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalInput;

import org.photonvision.PhotonUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
//import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SerialPort;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
*/	
 public class Robot extends TimedRobot
 {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // Master power
  double DRIVE_POWER = 0.6;
  double INTAKE_POWER = 0.6;
  // Gear ratio - gear1/gear2 * gearbox
  double armRatio = 500;
  double handRatio = (54/12)*(16);

  Timer autoTimer = new Timer();
  ///CHANGE RIGHT AND LEFT PORTS TO 13 & 12
  ///
  ///
  XboxController xBox1 = new XboxController(0);  //drive controller
  XboxController xBox2 = new XboxController(1);
  WPI_TalonSRX left = new WPI_TalonSRX(1);
  WPI_TalonSRX leftF = new WPI_TalonSRX(14);  // left follow motor
  WPI_TalonSRX right = new WPI_TalonSRX(0);  // right motor
  WPI_TalonSRX rightF = new WPI_TalonSRX(15); // right follow motor
  //gyro
  //AHRS navx = new AHRS(SerialPort.Port.kUSB);

  // Intake/Outake
  WPI_TalonSRX intakeSrx = new WPI_TalonSRX(10);   //Intake motor
  // Intake backstop
  DigitalInput gamePiece = new DigitalInput(0);

  MotorControllerGroup leftM = new MotorControllerGroup(left, leftF);
  MotorControllerGroup rightM = new MotorControllerGroup(right, rightF);


  DifferentialDrive diffDrive = new DifferentialDrive (leftM, rightM);

  // Arm init
  // Falcons 
  TalonFX armFx = new TalonFX(4); // FX W/Encoder - only moved for intake
  TalonFX handFx = new TalonFX(5); // FX W/Encoder - used for positioning game pieces

  // Arm control values
  final double ARM_EXTENDED_POSITION = -45;
  final double HAND_PICKUP_POSITION = -75;
  final double ARM_RETRACTED_POSITION = -5;
  final double HAND_RETRACTED_POSITION = 0;
  int armPos = 1;

  // Vision init
  PhotonCamera sideCamera = new PhotonCamera("photonvision");
  boolean targetSeen = false;

  // Methods for arm - input in degrees
  public void moveArm(double position) 
  {
    armFx.set(TalonFXControlMode.Position, (int)((2048/360)*position*armRatio));
  }


  public void moveHand(double position)
  {
    handFx.set(TalonFXControlMode.Position, (int)((2048/360)*position*handRatio));
  }


  public boolean handOut()
  {
    // If current position of hand is less than (closer to parellel than) the 70 degree clearance
    if (handFx.getSelectedSensorPosition()<(-70*handRatio*(2048/360)))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  public boolean armIn ()
  {
    // If current position of arm is greater than (closer to perpendicular than) the 85 degree clearance
    if (armFx.getSelectedSensorPosition()<-6*armRatio*(2048/360))
    {
      return true;
    }
    else
    {
      return false;
    }
  }


  // Checker to see if desired position is reached (for both hand and arm)
  public boolean handOnTarget(double position)
  {    
    boolean handOnTarget = ((position*handRatio*(2048/360))-handFx.getSelectedSensorPosition())<100;
    return handOnTarget;
  }

  public boolean armOnTarget(double position)
  {    
    boolean armOnTarget = ((position*armRatio*(2048/360))-armFx.getSelectedSensorPosition())<100;
    return armOnTarget; 
  }
  // Methods to ensure the arm and forearm (hand) hold position SEPARATE from target position.
  public void holdArm()
  {
    if (Math.abs(armFx.getSelectedSensorPosition()-armFx.getClosedLoopTarget()) < 100)
      {
        //keep current target
        armFx.set(TalonFXControlMode.Position, armFx.getClosedLoopTarget());
      }
      else
      {
        // Set target to current position if not on target
        armFx.set(TalonFXControlMode.Position, armFx.getSelectedSensorPosition());
      }
  }

  public void holdHand()
  {
    if (Math.abs(handFx.getSelectedSensorPosition()-handFx.getClosedLoopTarget()) < 1000)
      {
        handFx.set(TalonFXControlMode.Position, handFx.getClosedLoopTarget());
      }
      else
      {
        handFx.set(TalonFXControlMode.Position, handFx.getSelectedSensorPosition());
      }
  }

  public void extendArm ()
  {
    moveHand(HAND_PICKUP_POSITION);
    if (handOut())
    {
      moveArm(ARM_EXTENDED_POSITION);
    }
  }

  public void retractArm()
  {
    if (handOut())
    {
      moveArm(ARM_RETRACTED_POSITION);
      if (armIn())
      {
        moveHand(HAND_RETRACTED_POSITION);
      }
    }
  }

  public void placeAt(double hand_position)
  {
    if (!armIn())
    {
      retractArm();
    }
    else
    {
      moveHand(hand_position);
    }
  }
/* Main arm control ahead 
   Input desired angle for bicep and forearm (arm and hand) 
   Uses logic to determine whether or not certain parts are clear to move.
   Includes reajustment (if the forearm gets too low / close to the inner frame) */ 
  /* 
   public void armTo(double bicep_position, double forearm_position)
  {
    if (armIn() && !armOnTarget(bicep_position))
    {
      if (handOnTarget(forearm_position))
      {
        moveArm(bicep_position);
      }
      else
      {
        moveHand(forearm_position);
        holdArm();
      }
    }
    else if (armOnTarget(bicep_position))
    {
      if (!handOut())
      {
        moveHand(forearm_position);
      }
      else
      {
        moveArm(bicep_position);
        moveHand(forearm_position);
      }
    }
    else 
    {
      if (handOut())
      {
        moveArm(forearm_position);
        holdHand();
      }
      else
      moveHand(-80);
    }
 }*/
  /* 
  // Gyro balance methods
  public void balance()
  {
    double currentAngle = navx.getPitch();
    double error = 0-currentAngle;
    double drivePower = -Math.min(0.015 * error, 1);
    if (Math.abs(drivePower) < 0.4)
    {
      drivePower *= Math.copySign(0.4, drivePower);
      diffDrive.tankDrive(drivePower, drivePower);
    }
  }
  
  public boolean isBalanced()
  {
    return Math.abs(0-navx.getPitch()) < 1;
  }
  /**
   * F=follow
   * This function is run when the robotfrc.robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Full Auto", kDefaultAuto);
    m_chooser.addOption("Quick", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


    leftM.setInverted(false);
    rightM.setInverted(true);

    // Gyro stuff
    //navx.calibrate();
    //navx.reset();

    // Sensor stuff
    armFx.setSelectedSensorPosition(-95*handRatio*(2048/360));
    handFx.setSelectedSensorPosition(handRatio*(2048/360));


    // FPID values - for arm control
    armFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    armFx.setSensorPhase(true);
    armFx.setInverted(true);
    armFx.config_kF(0, 0);
    armFx.config_kP(0, 0.2);
    armFx.config_kI(0, 0);
    armFx.config_kD(0, 0);
    armFx.configClosedLoopPeakOutput(0, 0.3);
    armFx.configAllowableClosedloopError(0, 0);

    handFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    handFx.setSensorPhase(true);
    handFx.setInverted(true);
    handFx.config_kF(0, 0);
    handFx.config_kP(0, 0.4);
    handFx.config_kI(0, 0);
    handFx.config_kD(0, 0);
    handFx.configClosedLoopPeakOutput(0, 0.2);
    handFx.configAllowableClosedloopError(0, 0);
  }
  @Override
  public void autonomousInit(){}
  
  @Override
  public void autonomousPeriodic() 
    {
      var result = sideCamera.getLatestResult();
      if (Math.abs(result.getBestTarget().getYaw()) < 5 )
      {
        targetSeen = true;
      }
  //vision code first
    switch (m_autoSelected) 
      {
        case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        if (autoTimer.get() < 1)
          {
            diffDrive.arcadeDrive(0.6, 0);
          }
        else if (autoTimer.get() > 5)
          {
            diffDrive.arcadeDrive(0, 0);
          }
        }
      }
    
	/** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    // Drive base code
    if (Math.abs(xBox1.getLeftY())>0.07||Math.abs(xBox1.getLeftX())>0.07)
    {
      diffDrive.arcadeDrive(xBox1.getLeftY()*DRIVE_POWER,xBox1.getLeftX()*-DRIVE_POWER);  
    }
    else
    {
      diffDrive.arcadeDrive(0, 0);
    }
    //first drivepower is forward & second drivepower is turn
    //figure out getY and getX because of button not coords.



    // Intake - stops rotation based on limiter switch.
    // Overriden value
    intakeSrx.set(0);
    if (xBox2.getLeftTriggerAxis() > 0.2 && gamePiece.get())
    {
     intakeSrx.set(1);
    }
    else if (xBox2.getRightTriggerAxis() > 0.2)
    {
      intakeSrx.set(-0.2);
    }
    else 
    {
      intakeSrx.set(0);
    }
    
    //Arm setting - controlled by separate method
    // A. floor position
    // B. low shelf
    // X. top shelf / ??middle post??
    // Rt. top post
    // Default - neutral
    if (xBox2.getAButton())
    {
      armPos = 1;
      extendArm();
    }
    else if (xBox2.getBButton())
    {
      armPos = 2;
      placeAt(-30);
    }
    else if (xBox2.getXButton())
    {
      armPos = 3;
      placeAt(-70);
    }
    else if (xBox2.getRightBumper()) 
    {
      armPos = 4;
      placeAt(-100);
    }
    else
    {
      armPos = 5;
      retractArm();
    }
    // Debug console output
    System.out.println((armFx.getSelectedSensorPosition()));
    System.out.println(armIn());
    System.out.println(handFx.getClosedLoopTarget());
    System.out.println(handOut());
    
/*

    // Gyro balance code
    if (xBox1.getStartButtonPressed())
    {
      if (isBalanced())
      {
        diffDrive.arcadeDrive(0,0);
      }
      else
      {
        balance();
      }
    }*/
  }  
  /** This function is called once when the robotfrc.robot is disabled. */
  @Override
  public void disabledInit() {}
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  /** This function is called once when the robotfrc.robot is first started up. */
  @Override
  public void simulationInit() {}
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}