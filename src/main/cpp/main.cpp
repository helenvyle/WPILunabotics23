#define _USE_MATH_DEFINES

#include <frc/Joystick.h>

#include <frc/TimedRobot.h>
#include <frc/Encoder.h>
#include <frc/Servo.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>

#include <frc/Timer.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <iostream>
#include <frc/AnalogInput.h>

#include "ctre/Phoenix.h"

#include <cmath>
#include <string>
#include <cstdlib>
#include <bitset>
#include <vector>

#define RAND_MAX = 100


using namespace std;

class Robot : public frc::TimedRobot {

    //NT_Subscriber ySub;

    double bucket_encoder_readings  = 0.0;
    // double excavator_enocoder_readings = 0.0; //-AB

    //original
    static const int leftLeadDeviceID = 4, leftFollowDeviceID = 3, rightLeadDeviceID = 2, rightFollowDeviceID = 1;

    // static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;

    frc::Encoder bucket_encoder{0,1};

    frc::AnalogInput hall_effect{0};

    frc::AnalogInput pressure_sensor{1};

    rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_belt{5, rev::CANSparkMax::MotorType::kBrushless};

    //positive percentage is outward 
    TalonSRX linear_slider = {5};
    TalonSRX excavator = {1}; // double check
    TalonSRX bucket = {3};
    
    frc::Servo cameraServo{0};
    

  

    // bucket variables
    
    float initial_bucket_encoder_reading = 0.0;
    bool bucket_done = 0;
    bool bucket_dumping_done = 0;

    bool done = 0;
  
// variables for Autonomous Operations
// switch case with all the cases and variables as well

    enum states 
    {
    IDLE,
    DRIVE,
    DIG,
    DUMP,
    STOP
    };

    states state = IDLE;
    int received_data = -1;
    int driving_done = -1;
    double angle =0;
    
    rev::SparkMaxRelativeEncoder leftLead_encoder = m_leftLeadMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder rightLead_encoder = m_rightLeadMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder leftFollower_encoder = m_leftFollowMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder rightFollower_encoder = m_rightFollowMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder excavatorBelt_encoder = m_belt.GetEncoder();         //-AB
    // rev::SparkMaxAbsoluteEncoder excavatorBelt_encoder_absolute = m_belt.GetAbsoluteEncoder(); //-AB (test each 42 positions)

    float rightSpeed = 0.6;

    float leftSpeed = 0.6;

    double x_curr = 0;
    double y_curr = 0;

    bool hardStop = false;

    vector<int> speeds;
    
    double default_array[3];
    std::vector<double> heading = {-1,-1, -1};
    std::vector<double> current_coord = {-1,-1, -1};
    std::vector<double> target_coord = {-1, -1, -1};
    int counter = 0;

    // directly horizontal /start position : 1079590912 

    // frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
    // frc::DifferentialDrive::TankDrive m_robotDrive;
    private:

        int enc_res = 42;
        int wheelbase = 20; // in
        int wheel_diam = 10; // in
        float right_speed = 0.0;
        float left_speed = 0.0;
        bool flag = false;

    void RobotInit() override{

        //  cameraServo.SetBounds(2.5,1.520,0.759,0.002,0.5);

        bucket_encoder.SetDistancePerPulse(0.05); // encoder resolution 360/(188*4.65)
       
        m_rightFollowMotor.Follow(m_rightLeadMotor, false);
        m_leftFollowMotor.Follow(m_leftLeadMotor, false);
        m_rightLeadMotor.SetInverted(true);
        // m_leftLeadMotor.SetInverted(false);
        // m_robotDrive.SetSafetyEnabled(false);
        linear_slider.SetInverted(true);
        counter = 0;

        m_belt.SetOpenLoopRampRate(1); //-AB
        // frc::SmartDashboard::PutString("Motor_StickyFault", std::to_string(m_belt.GetStickyFaults())); //-AB
        // frc::SmartDashboard::PutString("Motor_Fault", std::to_string(m_belt.GetFaults())); //-AB
        // frc::SmartDashboard::PutNumber("Output Current", m_belt.GetOutputCurrent());
        // frc::SmartDashboard::PutNumber("Applied Output", m_belt.GetAppliedOutput());
        

    }
//     //pi per second at top speed
//     //pi/2 per second at half speed
//     //linear

//     //encoder resolution: 42
//     //wheelbase: 20inches 
//     //diameter: 10inches 


   
    void AutonomousPeriodic() override {

        // reconfigure_up(0.3);
        if(counter == 0) { 
            counter = 1;
            drive_to(-20, 30);

        }
    // AB   
        // for (int cycle = 1; cycle <= 42; cycle++)       //-AB
        // {
        //     std::cout << "Cycle " << cycle + 1 << std::endl;
        //     frc::SmartDashboard::PutNumber("Digger Test Cycle", cycle);
            
        //     try {
        //         // if (m_belt.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kQuadrature, 4096) == 0) {
                    
        //         // }
        //         m_belt.Set(-0.1);
        //         sleep(2);
        //     }
        //     catch (const std::exception &exc) {
        //         cout << "Error.\n";
        //         std::cerr << exc.what();
        //         // cout << "Age is: " << myNum;
        //         frc::SmartDashboard::PutString("Excavator Error", exc.what());
                
        //         m_belt.Set(0.1);
        //         sleep(2);
        //     }

        //     if (m_belt.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kQuadrature, 4096).GetPosition() == cycle) {
        //         m_belt.Set(0);
        //         std::cout << "Pos reached.\n";

        //     }
        // }


    // AB
        // rev::CANSparkMax::FaultID::kHasReset
        // int r = rand() / 1000000000;
       

        // print_m_belt();
        // m_belt.ClearFaults();


        // update_safety();
        // if(m_belt.GetOutputCurrent() > 35){
        //     m_belt.Set(0.0);
        // }
        // else{
        //     m_belt.Set(-0.3);
        //     if(10 > excavatorBelt_encoder.GetVelocity() && excavatorBelt_encoder.GetVelocity() > -10){
        //         m_belt.Set(0.1);
        //         sleep(0.5);
        //     }
        // }
        
        // print_m_belt();
        // sleep(1.5);
        // std::cout << m_belt.GetStickyFaults() << std::endl;
        // m_belt.Set(0);
        // print_m_belt();
        // sleep(1.5);
        
        // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kBrownout) || 
        // m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kDRVFault) ||
        // m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset) ||
        // m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kStall) || 
        // m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kSensorFault) ||
        // m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kMotorFault)) 
        // {
        //     checkStickyFaults(m_belt);
                
        
        //     // frc::SmartDashboard::PutString("Motor_StickyFault", "Brownout");

        //     // Clear the brownout fault
        //     // m_belt.ClearFaults();
        //     sleep(0.5);
        //     // Set the motor output to 0.1 (10% power)
        //     m_belt.Set(0.08);
        //     print_m_belt();
        //     sleep(2);
        //     m_belt.Set(0.1);
        //     print_m_belt();
        //     sleep(1);
        //     m_belt.Set(0.15);
        //     print_m_belt();
        //     sleep(1);
        //     m_belt.Set(0.2);
        //     print_m_belt();
        //     sleep(1);
        //     m_belt.Set(0.15);
        //     print_m_belt();
        //     sleep(1);
        //     m_belt.Set(0.1);
        //     print_m_belt();
        //     sleep(1);
        //     m_belt.Set(0.08);
        //     print_m_belt();
        //     sleep(1);
        //     m_belt.Set(0);
        //     print_m_belt();
        //     sleep(1);
        // } else {
        //     m_belt.Set(-0.1);
        //     print_m_belt();
        //     frc::SmartDashboard::PutString("Motor_StickyFault", "None");
        //     sleep(3 + r);

        // }
        // m_belt.Set(0);
        // print_m_belt();
        // sleep(3);

        

        
        // drive_to(0,50);
        // sleep(1);
            // drive_to(0,-45);
        // drive_to(0, -45);

        // // std::cout << pressure_sensor.GetValue() << std::endl;
        // update_safety();
        // if(counter == 0) {
        //     counter = 1;
        //  // m_belt.Set(-0.5);
        //     reconfigure_up(0.4);

        //     drive(50);

        //     reconfigure_down(0.4);
        //     sleep(1);
            // // bucket.Set(ControlMode::PercentOutput, -0.4);
            // linear_slider_control(2, -0.5);
            // linsear_slider.Set(0);
            // sleep(1);


            // linear_slider_control(2, -0.5);
            // excavator.Set(ControlMode::PercentOutput, 0.4);

            // start_belt(-0.5);
            // while(pressure_sensor.GetValue() > 2200){
            //     frc::SmartDashboard::PutNumber("Pressure Sensor", pressure_sensor.GetValue());
            // }
            // // m_belt.Set(0);
            // // linear_slider.Set(ControlMode::PercentOutput, 0.0);
            // bucket.Set(ControlMode::PercentOutput, 0.3);


        //     // linear_slider_control(2, 0.5);
        //     // reconfigure_down(0.4);
        //     reconfigure_up(0.4);
        //     // // // // turn(180);
        //     drive(-50);
        //     reconfigure_down(0.4);
        //     sleep(1);
        //     bucket_function(100);
        //     // reconfigure_up(0.4);
        //     // bucket.Set(ControlMode::PercentOutput,-0.1);
        // }
        

        // linear_slider.Set(ControlMode::PercentOutput, 0.3);
        // std::cout << pressure_sensor.GetValue() << std::endl;







//cameraServo.SetAngle(0);
   
    // //   cameraServo.Set(-1);
    //   if( angle <= 180)
    //   {
    //  // cameraServo.SetSpeed(-0.5);
    //   cameraServo.SetAngle(angle);
    // //  cameraServo.SetAngle(0);
    //   frc::SmartDashboard::PutNumber("Servo REadings",cameraServo.GetAngle());
    //   angle++;
    //   }
      
    //   cameraServo.
      // sleep(700);
//cameraServo.SetAngle(90);
        
       // cameraServo.SetAngle(90);
       // cameraServo.GetAngle();

        // switch(state) {
        //     case IDLE:
        //         while(received_data == -1) {
        //            // current_coord = frc::SmartDashboard::GetNumberArray("Coordinate Location m", default_array);
        //             if(current_coord[0] != -1) {
        //                 received_data = 1;
        //                 state = DRIVE;
        //                 break;
        //             }

        //         }
        //     case DRIVE: 
        //         while(driving_done == -1) {
        //             driving_done = frc::SmartDashboard::GetNumber("Driving Done", -1);
        //             target_coord = frc::SmartDashboard::GetNumberArray("Target Location m", default_array);
        //             drive_to(target_coord[0], target_coord[1]);
        //             if(driving_done == 1) {
        //                 state = DIG;
        //                 break;
        //             }
        //         }

        //     case DIG:
        //         // TODO: Add code here for digging. Check if motor does turn when requested. Detect if enough weight in bucket.
        //         break;
                
        //     case DUMP:
        //         break;

        //     case STOP:
        //         m_robotDrive.TankDrive(0,0);
        //         excavator.Set(ControlMode::PercentOutput, 0.0);
        //         linear_slider.Set(ControlMode::PercentOutput, 0.0);
        //         // belt.Set(ControlMode::PercentOutput, 0.0);

        
            

        // }

     // cameraServo.SetSpeed(0.5);
  
    // use SmartDashboard to send numbers signifying the state and use the switch cas accordingly.


        // drive(-20);
        // m_robotDrive.TankDrive(0.5, 0.0);
        // m_leftLeadMotor.Set(0.5);
        // turn(90);
        // drive(50);
        // turn(45);
        //negative - up
       //   bucket.SetInverted(true);  
        //      bucket.Set(ControlMode::PercentOutput,0.5);
 
 // bucket_function(115);
       
    //    drive(235);
        // drive(-75);
    //    drive_to(25,20);
     //   turn(45);

    //    excavator.Set(ControlMode::PercentOutput,-20);
    //    linear_slider.Set(ControlMode::PercentOutput, -20);
        // printf("Linear Slider\n");
        // printf( "%d\n",linear_slider.GetSelectedSensorPosition(0));
        // if(!done) {
        //     // drive(235);

        // }
        
        // if(counter <= 150){
        //     linear_slider.Set(ControlMode::PercentOutput, 0.3);
        //    // printf( "%d\n",excavator.GetSelectedSensorPosition(0));
        //     counter ++ ;
        // }
        // else {
        //     linear_slider.Set(ControlMode::PercentOutput, 0);
        //     counter = 1;
        // }

        // if(counter <= 150){
        // linear_slider.Set(ControlMode::PercentOutput, -0.2);
        //    // printf( "%d\n",excavator.GetSelectedSensorPosition(0));
        //     counter ++ ;
        // }
        //  encoder_values();
        // localization();
       // arc_turn_left(50, 10, 90);
        // m_robotDrive.TankDrive(0.4, 0.0);
        // m_belt.Set(0.4);
    //    std::cout << hall_effect.GetValue() << std::endl;

        // if(counter == 0){
        //     counter = 2;
        // //     reconfigure_up();
        //     // reconfigure_down(0.3);
        //     m_belt.Set(0.3);
        // }
        // if(counter == 1){
        //     counter = 2;
        // }
        
        // sleep(0.4);
        // linear_slider.Set(ControlMode::PercentOutput, -0.4);
        // reconfigure_up(0.3);
        // reconfigure_up();
        // m_belt.Set(-0.3);
       //bucket_function();

    //    std::cout << pressure_sensor.GetValue() << std::endl;

    //    if (counter == 0){
    //     counter++;
        // std::cout << hall_effect.GetValue() << std::endl;


    //    }

    } // End of AutonomousPeriodic

    void start_belt(float speed){
        bool flag = true;
        while(flag) {
            if(m_belt.GetOutputCurrent() > 35){
                m_belt.Set(0.0);
                flag = false;
            }
            else{
                m_belt.Set(speed);
                if(10 > excavatorBelt_encoder.GetVelocity() && excavatorBelt_encoder.GetVelocity() > -10){
                    m_belt.Set(0.2);
                    sleep(0.5);
                }
                else{
                    flag = false;
                }
            }
        }
    }

    void update_safety(){
        frc::SmartDashboard::PutNumber("Encoder", excavatorBelt_encoder.GetPosition());
        frc::SmartDashboard::PutString("Motor_StickyFault",std::to_string(m_belt.GetStickyFaults()));
        frc::SmartDashboard::PutNumber("Motor_Velocity", excavatorBelt_encoder.GetVelocity());
        frc::SmartDashboard::PutNumber("Belt Voltage", m_belt.GetBusVoltage());
        frc::SmartDashboard::PutNumber("Belt Motor Temp. (C)", m_belt.GetMotorTemperature());
        frc::SmartDashboard::PutNumber("FL Drive Voltage", m_leftLeadMotor.GetBusVoltage());
        frc::SmartDashboard::PutNumber("FL Drive Motor Temp. (C)", m_leftLeadMotor.GetMotorTemperature());
        frc::SmartDashboard::PutNumber("BL Drive Voltage", m_leftFollowMotor.GetBusVoltage());
        frc::SmartDashboard::PutNumber("BL Drive Motor Temp. (C)", m_leftFollowMotor.GetMotorTemperature());
        frc::SmartDashboard::PutNumber("FR Drive Voltage", m_rightLeadMotor.GetBusVoltage());
        frc::SmartDashboard::PutNumber("FR Drive Motor Temp. (C)", m_rightLeadMotor.GetMotorTemperature());
        frc::SmartDashboard::PutNumber("BR Drive Voltage", m_rightFollowMotor.GetBusVoltage());
        frc::SmartDashboard::PutNumber("BR Drive Motor Temp. (C)", m_rightFollowMotor.GetMotorTemperature());
        frc::SmartDashboard::PutNumber("Reconfiguration Encoder", excavator.GetSelectedSensorPosition(0));
        frc::SmartDashboard::PutNumber("Reconfiguration Motor Temp. (C)", excavator.GetTemperature());
        frc::SmartDashboard::PutNumber("Bucket Motor Temp. (C)", bucket.GetTemperature());
        frc::SmartDashboard::PutNumber("Bucket Position (Deg)", bucket_encoder.GetDistance());
    }

    // signature
    void linear_slider_control(int numPositions, float speed) 
    {
        bool flag1 = true;
        bool flag2 = false;
        int counter = 1;
        while(flag1){

            if (hall_effect.GetValue() < 8)
            {
                // std::cout << hall_effect.GetValue() << std::endl;
                if(flag2){
                    flag1 = false;
                    break;
                } else {
                    linear_slider.Set(ControlMode::PercentOutput, speed);
                }
            }
            if(hall_effect.GetValue() > 8){
                // std::cout << hall_effect.GetValue() << std::endl;
                linear_slider.Set(ControlMode::PercentOutput, speed);
                if(counter < numPositions){
                    flag2 = false;
                    counter++;
                }
                else{
                    flag2 = true;
                }
            }
        }
        linear_slider.Set(ControlMode::PercentOutput, 0.0);
        return;
    }

    void reconfigure_up(float speed){
        int current_encoder = excavator.GetSelectedSensorPosition(0);
        while (excavator.GetSelectedSensorPosition(0) < current_encoder + 500){
            excavator.Set(ControlMode::PercentOutput, -speed);
        }
        excavator.Set(ControlMode::PercentOutput, 0.0);
    }

    void reconfigure_down(float speed){
        int current_encoder = excavator.GetSelectedSensorPosition(0);
        while (excavator.GetSelectedSensorPosition(0) > current_encoder - 500){
            excavator.Set(ControlMode::PercentOutput, speed);
        }
        excavator.Set(ControlMode::PercentOutput, 0.0);
    }
    

    void bucket_encoder_values()
    {
        bucket_encoder_readings = bucket_encoder.GetDistance();
        bucket_encoder.GetRate();
    }

    // void excavator_encoder_relative_values()
    // {
    //     excavatorBelt_encoder_relative = excavator_en
    // }

    void bucket_function(float targetDegrees)
    {
        // get encoder values from the bucket
        bucket_encoder_values();
         while(abs(bucket_encoder_readings - targetDegrees) > 0 && !bucket_done)
         {
            bucket.Set(ControlMode::PercentOutput,1);
            bucket_encoder_values();
            print_encoders();
         };
     
           bucket.Set(ControlMode::PercentOutput,0);
           bucket_dumping_done = 1;
           sleep(1);
    
        if(bucket_dumping_done)
             {
            while (abs(bucket_encoder_readings - 5) > 0 )
            {
             bucket.SetInverted(true);  
             bucket.Set(ControlMode::PercentOutput,1);
             bucket_encoder_values();
             print_encoders();
            }
            bucket.Set(ControlMode::PercentOutput,0);
            bucket_dumping_done=0;
            bucket_done=1;
          }

    }

    //figure out how to get robot current heading and how to extract that value
    //positive/negative linear and angular diff -- check to make sure they behave correctly logically
    //fix turn and drive functions so they can actually go sequentially without getting stuck somewhere

    void drive_to(float x_goal, float y_goal) {
        float tolerance = 0.04;
        float linear_diff = abs(sqrt(pow((x_goal - x_curr), 2) + pow((y_goal - y_curr), 2)));
        while (linear_diff > tolerance) {
                
            linear_diff = abs(sqrt(pow((x_goal - x_curr), 2) + pow((y_goal - y_curr), 2)));
            //this is to test to see if it'd be a bit more accurate? 
            float target_angle =  atan2((x_goal - x_curr), (y_goal - y_curr));
            float angle_in_deg = target_angle*(180/(M_PI));
            //original working:
            // float target_angle =  atan2((y_goal - y_curr), (x_goal - x_curr)) + 4*M_PI;
            //this can be implemented or pulled somehow from localization once I figure out how it works
            // heading = frc::SmartDashboard::GetNumberArray("Heading Deg (Z,Y,X)", default_array);

            // float current_angle = heading[0];

            float current_angle = 0;
            float angular_diff = angle_in_deg - current_angle;
            send_speed(linear_diff, angular_diff);
        }
        // } else if(x_goal == 0) {
        //     send_speed(-y_goal, 0);
        // }


    }

    void send_speed(float linear_diff, float angular_diff) {        
        turn(angular_diff);
        sleep(2);
        m_leftLeadMotor.Set(0);
        m_rightLeadMotor.Set(0);
        std::cout << "enc" << leftLead_encoder.GetPosition() << std::endl;
        std::cout << "lin diff" << linear_diff << std::endl;
        drive(linear_diff);
    }
    
    void turn(float angle) {
        float speed = 0.4;
        if(angle > 0) {
            float encoder_goal = (((angle/360)*(3.14*wheel_diam) * enc_res) + leftLead_encoder.GetPosition())/2;

            while((leftLead_encoder.GetPosition()<= encoder_goal + 5|| leftLead_encoder.GetPosition() <= encoder_goal - 5)  && (abs(rightLead_encoder.GetPosition())<= encoder_goal -5 || abs(rightLead_encoder.GetPosition())<= encoder_goal + 5)) {
                int errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                int errorRight = encoder_goal - abs(rightLead_encoder.GetPosition());
                int k = 0.5;

                // m_leftLeadMotor.Set(speed - (errorLeft * k));
                float speedL = speed - (errorLeft * k);
                // m_rightLeadMotor.Set(-speed - (errorRight * k));
                float speedR = -speed - (errorRight * k);
                m_leftLeadMotor.Set(speedL);
                m_rightLeadMotor.Set(speedR);
                // m_robotDrive.TankDrive(speedL, speedR);
            }
        } else if (angle <= 0) {
            float encoder_goal = (leftLead_encoder.GetPosition() + ((angle/360)*(3.14*wheel_diam) * enc_res))/2;

            while(leftLead_encoder.GetPosition() >= encoder_goal) {    
                int errorLeft = abs(encoder_goal) - abs(leftLead_encoder.GetPosition());
                int errorRight = abs(encoder_goal) - abs(rightLead_encoder.GetPosition());
                int k = 0.2;

                // m_leftLeadMotor.Set(-speed - (errorLeft * k));
                float speedL = -speed - (errorLeft * k);
                // m_rightLeadMotor.Set(speed - (errorRight * k));
                float speedR = speed - (errorRight * k);
                // m_robotDrive.TankDrive(speedL,speedR);
                m_leftLeadMotor.Set(speedL);
                m_rightLeadMotor.Set(speedR);
            }
        } else {
            m_leftLeadMotor.Set(0);
            m_rightLeadMotor.Set(0);
            // m_robotDrive.TankDrive(0.0,0.0);
        }
    } 


    bool drive (float distance) {
        // sleep(3);

        if(distance > 0) {
            double encoder_goal = (distance)/(3.14*wheel_diam) * enc_res * 3.95 + leftLead_encoder.GetPosition();
            float start = leftLead_encoder.GetPosition();
            std::cout << "enc goal: " << encoder_goal << std::endl;
            float speed = 0.5;
            float dist_trav = leftLead_encoder.GetPosition() - start;
            while((dist_trav <= encoder_goal + 5 || dist_trav <=encoder_goal - 5)) {
                double errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                double errorRight = encoder_goal - rightLead_encoder.GetPosition();
                int k = 0.2;
                // m_leftLeadMotor.Set(speed - (errorLeft * k));
                float speedL = speed - (errorLeft * k);
                // m_rightLeadMotor.Set(speed - (errorRight * k));
                float speedR = speed - (errorRight * k);
                m_leftLeadMotor.Set(speedL);
                m_rightLeadMotor.Set(speedR);
                dist_trav = leftLead_encoder.GetPosition() - start;
            }
            m_leftLeadMotor.Set(0);
            m_rightLeadMotor.Set(0);
            // m_robotDrive.TankDrive(0.0,0.0);
            sleep(3);
            done = 1;
            return true;
        }
        else {
            double encoder_goal = leftLead_encoder.GetPosition() + (distance)/(3.14*wheel_diam) * enc_res * 3.95;

            float speed = -0.5;
            // m_robotDrive.TankDrive(speed, speed);
            m_leftLeadMotor.Set(speed);
            m_rightLeadMotor.Set(speed);
            while(leftLead_encoder.GetPosition() >= encoder_goal) {    
                double errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                double errorRight = encoder_goal - rightLead_encoder.GetPosition();
                // double errorLeft = abs(encoder_goal) - abs(leftLead_encoder.GetPosition());
                // int errorRight = abs(encoder_goal) - abs(rightLead_encoder.GetPosition());
                int k = 0.2;

                // m_leftLeadMotor.Set(-speed - (errorLeft * k));
                float speedL = speed + (errorLeft * k);
                // m_rightLeadMotor.Set(-speed - (errorRight * k));
                float speedR = speed + (errorLeft * k);
                // m_robotDrive.TankDrive(speedL, speedR);
                m_leftLeadMotor.Set(speedL);
                m_rightLeadMotor.Set(speedR);

            }
            m_leftLeadMotor.Set(0);
            m_rightLeadMotor.Set(0);
            // m_robotDrive.TankDrive(0.0,0.0);
            done = 1;
            return true;

        }
    }

    void print_encoders() {
        // frc::SmartDashboard::PutNumber("Left Lead Encoder Position", leftLead_encoder.GetPosition());
        // frc::SmartDashboard::PutNumber("Right Lead Position", rightLead_encoder.GetPosition());
        // frc::SmartDashboard::PutNumber("Left Follower Position", leftFollower_encoder.GetPosition());
        // frc::SmartDashboard::PutNumber("Right Follower Position", rightLead_encoder.GetPosition());

       // double default_array [3];
      //   std::vector<int> v = {0, 0, 0};
       //  default_array(v);          // OK
       frc::SmartDashboard::PutNumber("Encoder",bucket_encoder_readings);
    //    frc::SmartDashboard::PutNumber("Excavator Encoder",excavatorBelt_encoder_readings); //-AB
    }

    
    // Send values to SmartDashboard for excavator belt motor
    //   Temperature (Celsius)
    //   StickyFaults (bits)
    //   Faults (bits)
    //   Motor Output Current [double] (sum of Amps across three phases)
    //   Motor Output [double] (-1 to 1)
    // More info: 
    void print_m_belt() {
        frc::SmartDashboard::PutNumber("Excavator MotorTemp", m_belt.GetMotorTemperature());
        frc::SmartDashboard::PutString("Excavator Motor StickyFault", std::to_string(m_belt.GetStickyFaults()));
        frc::SmartDashboard::PutString("Excavator Motor Fault", std::to_string(m_belt.GetFaults()));
        frc::SmartDashboard::PutNumber("Excavator Output Current", m_belt.GetOutputCurrent());
        frc::SmartDashboard::PutNumber("Excavator Applied Output", m_belt.GetAppliedOutput());
    }



    // Function to check for sticky faults in m_belt and update SmartDashboard
    // void checkStickyFaults(rev::CANSparkMax& motor) {
    //     // Get the sticky faults
    //     auto faults = motor.GetStickyFaults();

    //     // Check if there are any sticky faults
    //     bool hasFault = false;
    //     if (faults & rev::CANSparkMax::FaultID::kMotorFault) {
    //         hasFault = true;
    //         // Print the fault to the console
    //         std::cout << "Sticky Fault: " << faults.sticky_fault << std::endl;

    //         // Add the fault to SmartDashboard
    //         frc::SmartDashboard::PutBoolean("Belt Sticky Fault", true);
    //     } else {
    //         // No sticky faults
    //         frc::SmartDashboard::PutBoolean("Belt Sticky Fault", false);
    //     }

    //     // Update SmartDashboard
    //     frc::SmartDashboard::UpdateValues();
    // }


    // Check for sticky faults with the excavator motor and update values to the SmartDashboard
    //   More info: https://codedocs.revrobotics.com/cpp/classrev_1_1_c_a_n_spark_max.html
    
// kMotorFault 	
// kSensorFault 	
// kStall 	
// kEEPROMCRC 	
// kCANTX 	
// kCANRX 	
// kHasReset 	
// kDRVFault 	
// kOtherFault 	
// kSoftLimitFwd 	
// kSoftLimitRev 	
// kHardLimitFwd 	
// kHardLimitRev 
    // void checktickyfaultChecker() 
    // {
    //     const std::unordered_map<rev::CANSparkMax::FaultID, std::string> faultNames = {
    //         {rev::CANSparkMax::FaultID::kBrownout, "Brownout"},
    //         {rev::CANSparkMax::FaultID::kCANRX, "CANRX"},
    //         {rev::CANSparkMax::FaultID::kCANTX, "CANTX"},
    //         {rev::CANSparkMax::FaultID::kDRVFault, "DRVFault"},
    //         {rev::CANSparkMax::FaultID::kEEPROMCRC, "EEPROMCRC"},
    //         {rev::CANSparkMax::FaultID::kHardLimitFwd, "HardLimitFwd"},
    //         {rev::CANSparkMax::FaultID::kHardLimitRev, "HardLimitRev"},
    //         {rev::CANSparkMax::FaultID::kHasReset, "HasReset"},
    //         {rev::CANSparkMax::FaultID::kIWDTReset, "IWDTReset"},
    //         {rev::CANSparkMax::FaultID::kMotorFault, "MotorFault"},
    //         {rev::CANSparkMax::FaultID::kOtherFault, "OtherFault"},
    //         {rev::CANSparkMax::FaultID::kOvercurrent, "Overcurrent"},
    //         {rev::CANSparkMax::FaultID::kSensorFault, "SensorFault"},
    //         {rev::CANSparkMax::FaultID::kSoftLimitFwd, "SoftLimitFwd"},
    //         {rev::CANSparkMax::FaultID::kSoftLimitRev, "SoftLimitRev"},
    //         {rev::CANSparkMax::FaultID::kStall, "Stall"}
    //     };
        
    //     std::bitset<16> faultBits(m_belt.GetStickyFaults());

    //     std::unordered_map<std::string, bool> faultStatus;

    //     for (int i = 0; i < faultBits.size(); ++i) {
    //         if (faultBits[i]) {
    //             rev::CANSparkMax::FaultID faultID = static_cast<rev::CANSparkMax::FaultID>(i);
    //             std::string faultName = faultNames[faultID];
    //             frc::SmartDashboard::PutString(faultName, "Fault");
    //             faultStatus[faultName] = true;
    //         }
    //     }

    //     for (auto const& faultName : faultNames) {
    //         if (faultStatus.find(faultName.second) == faultStatus.end()) {
    //             frc::SmartDashboard::PutString(faultName.second, "Not Present");
    //         }
    //     }
    //     frc::SmartDashboard::UpdateValues();

    //     //************************************************

    //     // const std::unordered_map<rev::CANSparkMax::FaultID, std::string> kFaultNames = {
    //     //     {rev::CANSparkMax::FaultID::kBrownout, "Brownout"},
    //     //     {rev::CANSparkMax::FaultID::kCANRX, "CANRX"},
    //     //     {rev::CANSparkMax::FaultID::kCANTX, "CANTX"},
    //     //     {rev::CANSparkMax::FaultID::kDRVFault, "DRVFault"},
    //     //     {rev::CANSparkMax::FaultID::kEEPROMCRC, "EEPROMCRC"},
    //     //     {rev::CANSparkMax::FaultID::kHardLimitFwd, "HardLimitFwd"},
    //     //     {rev::CANSparkMax::FaultID::kHardLimitRev, "HardLimitRev"},
    //     //     {rev::CANSparkMax::FaultID::kHasReset, "HasReset"},
    //     //     {rev::CANSparkMax::FaultID::kIWDTReset, "IWDTReset"},
    //     //     {rev::CANSparkMax::FaultID::kMotorFault, "MotorFault"},
    //     //     {rev::CANSparkMax::FaultID::kOtherFault, "OtherFault"},
    //     //     {rev::CANSparkMax::FaultID::kOvercurrent, "Overcurrent"},
    //     //     {rev::CANSparkMax::FaultID::kSensorFault, "SensorFault"},
    //     //     {rev::CANSparkMax::FaultID::kSoftLimitFwd, "SoftLimitFwd"},
    //     //     {rev::CANSparkMax::FaultID::kSoftLimitRev, "SoftLimitRev"},
    //     //     {rev::CANSparkMax::FaultID::kStall, "Stall"}
    //     // };

    //     // std::bitset<16> faultBits(m_belt.GetStickyFaults());

    //     // std::unordered_map<std::string, bool> faultStatus;

    //     // for (const auto& fault : m_belt.GetStickyFaults()) {
    //     //     const auto& faultID = fault.first;
    //     //     const auto& faultName = kFaultNames.at(faultID);
    //     //     const auto& hasFault = fault.second;

    //     //     if (hasFault) {
    //     //         frc::SmartDashboard::PutString(faultName, "Fault");
    //     //     } else {
    //     //         frc::SmartDashboard::PutString(faultName, "No Fault");
    //     //     }
    //     //     faultStatus[faultName] = hasFault;
    //     // }

    //     // for (const auto& faultName : kFaultNames) {
    //     //     if (faultStatus.find(faultName.second) == faultStatus.end()) {
    //     //         frc::SmartDashboard::PutString(faultName.second, "Not Present");
    //     //     }
    //     // }

    //     // frc::SmartDashboard::UpdateValues();



    //     // ************************

    //     // // Convert FaultID enum values to their corresponding string names
    //     // std::unordered_map<rev::CANSparkMax::FaultID, std::string> faultNames = {
    //     //     {rev::CANSparkMax::FaultID::kBrownout, "Brownout"},
    //     //     {rev::CANSparkMax::FaultID::kCANRX, "CANRX"},
    //     //     {rev::CANSparkMax::FaultID::kCANTX, "CANTX"},
    //     //     {rev::CANSparkMax::FaultID::kDRVFault, "DRVFault"},
    //     //     {rev::CANSparkMax::FaultID::kEEPROMCRC, "EEPROMCRC"},
    //     //     {rev::CANSparkMax::FaultID::kHardLimitFwd, "HardLimitFwd"},
    //     //     {rev::CANSparkMax::FaultID::kHardLimitRev, "HardLimitRev"},
    //     //     {rev::CANSparkMax::FaultID::kHasReset, "HasReset"},
    //     //     {rev::CANSparkMax::FaultID::kIWDTReset, "IWDTReset"},
    //     //     {rev::CANSparkMax::FaultID::kMotorFault, "MotorFault"},
    //     //     {rev::CANSparkMax::FaultID::kOtherFault, "OtherFault"},
    //     //     {rev::CANSparkMax::FaultID::kOvercurrent, "Overcurrent"},
    //     //     {rev::CANSparkMax::FaultID::kSensorFault, "SensorFault"},
    //     //     {rev::CANSparkMax::FaultID::kSoftLimitFwd, "SoftLimitFwd"},
    //     //     {rev::CANSparkMax::FaultID::kSoftLimitRev, "SoftLimitRev"},
    //     //     {rev::CANSparkMax::FaultID::kStall, "Stall"},
    //     // };

    //     // std::bitset<16> faultBits(m_belt.GetStickyFaults());


    //     // std::unordered_map<std::string, bool> faultStatus;

    //     // for (auto &const fault : m_belt.GetStickyFaults()) {
    //     //     rev::CANSparkMax::FaultID faultID = fault.first;
    //     //     // std::vector<std::string> faultNames{};
    //     //     std::string faultName = faultNames[faultID];
    //     //     bool hasFault = fault.second;

    //     //     if (hasFault) {
    //     //         frc::SmartDashboard::PutString(faultName, "Fault");
    //     //         faultStatus[faultName] = true;
    //     //     } else {
    //     //         frc::SmartDashboard::PutString(faultName, "No Fault");
    //     //         faultStatus[faultName] = false;
    //     //     }
    //     // }

    //     // for (auto const& faultName : faultNames) {
    //     //     if (faultStatus.find(faultName.second) == faultStatus.end()) {
    //     //         frc::SmartDashboard::PutString(faultName.second, "Not Present");
    //     //     }
    //     // }
    //     // frc::SmartDashboard::UpdateValues();

    //     // ********************************************


    //     // Map the uint16_t bits from GetStickyFaults to the enum map of faults
    //     // std::unordered_map<rev::CANSparkMax::FaultID, bool> faults;
        
    //     // uint16_t faultBits = m_belt.GetStickyFaults();
    //     // for (int i = 0; i < 16; i++) {
    //     //     faults[static_cast<rev::CANSparkMax::FaultID>(i)] = (faultBits & (1 << i)) != 0;
    //     // }
        
    //     // for (const auto& fault : faults) {
    //     //     std::string faultName = fault.first;
    //     //     std::string faultMessage;
    //     //     if (fault.second) {
    //     //         faultMessage = faultName + " fault has occured"; // TODO: add timestamp or to a log
    //     //     } else {
    //     //         faultMessage = faultName + " not present";
    //     //         // frc::SmartDashboard::Delete(faultName); //not valid
    //     //     }
    //     //     frc::SmartDashboard::PutString(faultName, faultMessage);
    //     // }
    //     // frc::SmartDashboard::UpdateValues();


    //     // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kBrownout))
    //     //     {
    //     //         frc::SmartDashboard::PutNumber("Excavator StickyFault kBrownout", 1);
    //     //     } else {
    //     //         frc::SmartDashboard::PutNumber("Excavator StickyFault kBrownout", 0);
    //     //     }
    //     // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kOvercurrent)) 
    //     //     {
    //     //         frc::SmartDashboard::PutNumber("Excavator StickyFault kOvercurrent", 1);
    //     //     } else {
    //     //         frc::SmartDashboard::PutNumber("Excavator StickyFault kOvercurrent", 0);
    //     //     }
    //     // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kIWDTReset)) 
    //     //     {
    //     //         frc::SmartDashboard::PutNumber("Excavator StickyFault kIWDTReset", 1);
    //     //     } else {
    //     //         frc::SmartDashboard::PutNumber("Excavator StickyFault kIWDTReset", 0);
    //     //     }
    //     // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kMotorFault))
    //     //     {
                
    //     //     }
    //     // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kSensorFault)) 
    //     //     {
                
    //     //     }
    //     // if (m_belt.GetStickyFault(rev::CANSparkMax::FaultID::kMotorFault))
    //     //     {
                
    //     //     }
    // }
};

#ifndef RUNNING_FRC_TESTS   
int main() { return frc::StartRobot<Robot>(); }
#endif
