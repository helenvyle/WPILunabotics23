#define _USE_MATH_DEFINES

#include <frc/Joystick.h>

#include <frc/TimedRobot.h>
#include <frc/Encoder.h>
#include<frc/Servo.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include <frc/Timer.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <iostream>
#include <frc/AnalogInput.h>

#include "ctre/Phoenix.h"

#include <cmath>



using namespace std;

class Robot : public frc::TimedRobot {

    //NT_Subscriber ySub;

    double bucket_encoder_readings  = 0.0;

    static const int leftLeadDeviceID = 4, leftFollowDeviceID = 3, rightLeadDeviceID = 2, rightFollowDeviceID = 1;

    frc::Encoder bucket_encoder{0,1};

    frc::AnalogInput hall_effect{0};

    frc::AnalogInput pressure_sensor{1};

    rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_belt{5, rev::CANSparkMax::MotorType::kBrushless};

    TalonSRX linear_slider = {5};
    TalonSRX excavator = {1}; // double check
    TalonSRX bucket = {3};
    
    frc::Servo exampleServo {0};

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
    
    rev::SparkMaxRelativeEncoder leftLead_encoder = m_leftLeadMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder rightLead_encoder = m_rightLeadMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder leftFollower_encoder = m_leftFollowMotor.GetEncoder();

    rev::SparkMaxRelativeEncoder rightFollower_encoder = m_rightFollowMotor.GetEncoder();

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
    // 


    frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
    // frc::DifferentialDrive::TankDrive m_robotDrive;
    private:

        int enc_res = 42;
        int wheelbase = 20; // in
        int wheel_diam = 10; // in
        float right_speed = 0.0;
        float left_speed = 0.0;
        bool flag = false;

    void RobotInit() override{

        bucket_encoder.SetDistancePerPulse(0.05); // encoder resolution 360/(188*4.65)
       
        m_rightFollowMotor.Follow(m_rightLeadMotor, false);
        m_leftFollowMotor.Follow(m_leftLeadMotor, false);
        m_rightLeadMotor.SetInverted(true);
        // m_robotDrive.SetSafetyEnabled(false);
        // units::unit_t<units::time::second, double, units::linear_scale> experation;
        units::second_t experation(1.5);
        m_robotDrive.SetExpiration(experation);
        linear_slider.SetInverted(true);
        counter = 0;

    }
//     //pi per second at top speed
//     //pi/2 per second at half speed
//     //linear

//     //encoder resolution: 42
//     //wheelbase: 20inches 
//     //diameter: 10inches 

   
    void AutonomousPeriodic() override {

        switch(state) {
            case IDLE:
                while(received_data == -1) {
                    current_coord = frc::SmartDashboard::GetNumberArray("Coordinate Location m", default_array);
                    if(current_coord[0] != -1) {
                        received_data = 1;
                        state = DRIVE;
                        break;
                    }

                }
            case DRIVE: 
                while(driving_done == -1) {
                    driving_done = frc::SmartDashboard::GetNumber("Driving Done", -1);
                    target_coord = frc::SmartDashboard::GetNumberArray("Target Location m", default_array);
                    drive_to(target_coord[0], target_coord[1]);
                    if(driving_done == 1) {
                        state = DIG;
                        break;
                    }
                }

            case DIG:
                
            case DUMP:

        }

    //  exampleServo.Set(75);
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
        turn(45);

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
        //     linear_slider.Set(ControlMode::PercentOutput, 0.3);
        //    // printf( "%d\n",excavator.GetSelectedSensorPosition(0));
        //     counter ++ ;
        // }
        //  encoder_values();
        // localization();
       // arc_turn_left(50, 10, 90);
        // m_robotDrive.TankDrive(0.4, 0.0);
        // m_belt.Set(0.4);
       // std::cout << hall_effect.GetValue() << std::endl;

        // if(counter == 0){
        //     counter = 2;
        //     reconfigure_up();
        //     reconfigure_down();
        // }
        // if(counter == 1){
        //     counter = 2;
        // }
        
        // sleep(0.4);
        // linear_slider.Set(ControlMode::PercentOutput, 0.0);
        // m_belt.Set(-0.3);
       //bucket_function();

    //    std::cout << pressure_sensor.GetValue() << std::endl;

    //    if (counter == 0){
    //     counter++;
    //     linear_slider_down(1);
    //    }
    }

    void linear_slider_down(int numPositions){
        bool flag1 = true;
        bool flag2 = false;
        int counter = 1;
        while(flag1){
            if (hall_effect.GetValue() < 8)
            {
                // std::cout << hall_effect.GetValue() << std::endl;
                linear_slider.Set(ControlMode::PercentOutput, 0.2);
                if(flag2){
                    flag1 = false;
                }
            }
            if(hall_effect.GetValue() > 8){
                // std::cout << hall_effect.GetValue() << std::endl;
                linear_slider.Set(ControlMode::PercentOutput, 0.2);
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
    }

    void reconfigure_up(){
        int current_encoder = linear_slider.GetSelectedSensorPosition(0);
        while (linear_slider.GetSelectedSensorPosition(0) < current_encoder + 600){
            linear_slider.Set(ControlMode::PercentOutput, -0.4);
        }
        linear_slider.Set(ControlMode::PercentOutput, 0.0);
        counter = 1;
    }

    void reconfigure_down(){
        int current_encoder = linear_slider.GetSelectedSensorPosition(0);
        while (linear_slider.GetSelectedSensorPosition(0) > current_encoder - 600){
            linear_slider.Set(ControlMode::PercentOutput, 0.4);
        }
        linear_slider.Set(ControlMode::PercentOutput, 0.0);
    }
    

       void bucket_encoder_values()
   {
        bucket_encoder_readings = bucket_encoder.GetDistance();
        bucket_encoder.GetRate();
   }

    void bucket_function(float targetDegrees)
    {
        // get encoder values from the bucket
        
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
    //turns until 45 degrees, if cant find april tag then drive forward and repeat
    //once finds april tag, it stops (unless we want it to drive forward a bit/be perpendicular?)
   void servoTest(){

   }

    void localization() {
        double dist_goal = 0.5;

        coord = frc::SmartDashboard::GetNumberArray("Coordinate Location m", default_array);
        x_curr = coord[0];
        y_curr = coord[1];

        float left_encoder_start = leftLead_encoder.GetPosition();
        // //below is the number of encoder counts for a 45 degree turn
        float encoder_cycle = 165;
        while(x_curr == -1 && y_curr == -1) {
            if(leftLead_encoder.GetPosition() - left_encoder_start <= encoder_cycle) {
                // m_robotDrive.TankDrive(0.4, -0.4);
                coord = frc::SmartDashboard::GetNumberArray("Coordinate Location m", default_array);
                x_curr = coord[0];
                y_curr = coord[1];

            } else {
                m_robotDrive.TankDrive(0,0);
                drive(10);
                left_encoder_start = leftLead_encoder.GetPosition();
            }

        }

        m_robotDrive.TankDrive(0,0);

    }

    void clearEncoders(){
        leftLead_encoder.SetPosition(0);
        leftFollower_encoder.SetPosition(0);
        rightLead_encoder.SetPosition(0);
        rightFollower_encoder.SetPosition(0);
    }

    //things to work on/fix:
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

            std::cout << "Target angle: " << angle_in_deg << std::endl;


            float current_angle = 0;
            float angular_diff = angle_in_deg - current_angle;
            // send_speed(linear_diff, angular_diff);
        }

    }

    void send_speed(float linear_diff, float angular_diff) {

        turn(angular_diff);
        sleep(1);
        drive(linear_diff);

    }
    
    void turn(float angle) {
        float encoder_goal = abs(((angle/360)*(3.14*wheel_diam) * enc_res))/2;
        float speed = 0.4;
        if(angle > 0) {
            while((leftLead_encoder.GetPosition()<= encoder_goal + 5|| leftLead_encoder.GetPosition() <= encoder_goal - 5)  && (abs(rightLead_encoder.GetPosition())<= encoder_goal -5 || abs(rightLead_encoder.GetPosition())<= encoder_goal + 5)) {
                int errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                int errorRight = encoder_goal - abs(rightLead_encoder.GetPosition());
                int k = 0.5;

                // m_leftLeadMotor.Set(speed - (errorLeft * k));
                float speedL = speed - (errorLeft * k);
                // m_rightLeadMotor.Set(-speed - (errorRight * k));
                float speedR = -speed - (errorRight * k);
                m_robotDrive.TankDrive(speedL, speedR);
            }
        } else if (angle <= 0) {
            while((leftLead_encoder.GetPosition()>= encoder_goal - 5 && leftLead_encoder.GetPosition() >= encoder_goal + 5) || (rightLead_encoder.GetPosition()<= abs(encoder_goal-5)  && rightLead_encoder.GetPosition() <= abs(encoder_goal + 5))) {
                int errorLeft = abs(encoder_goal) - abs(leftLead_encoder.GetPosition());
                int errorRight = abs(encoder_goal) - abs(rightLead_encoder.GetPosition());
                int k = 0.2;

                // m_leftLeadMotor.Set(-speed - (errorLeft * k));
                float speedL = -speed - (errorLeft * k);
                // m_rightLeadMotor.Set(speed - (errorRight * k));
                float speedR = speed - (errorRight * k);
                m_robotDrive.TankDrive(speedL,speedR);
            }
        } else {
            // m_leftLeadMotor.Set(0);
            // m_rightLeadMotor.Set(0);
            m_robotDrive.TankDrive(0.0,0.0);
        }
    } 

    void arc_turn_left(float radius, float time, int theta) {

        float radius_left = radius - (wheelbase/2);
        float radius_right = radius + (wheelbase/2);

        float dist_left = theta * radius_left;
        float dist_right = theta * radius_right;

        float speed_left = dist_left/(2*3.14*(wheel_diam/2)*time); //rad/sec
        float speed_right = dist_right/(2*3.14*(wheel_diam/2)*time); //rad/sec

        auto prev_time = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::now();
        auto elapsed_seconds = now_time - prev_time;
        while(elapsed_seconds.count() < time){
            float speedL = speed_left/3.14;
            float speedR = speed_right/3.14;
            std::cout << "SpeedL: " << speedL << std::endl;
            std::cout << "SpeedR: " << speedR << std::endl;
            // m_leftLeadMotor.Set(speedL);
            // m_rightLeadMotor.Set(speedR);
            m_robotDrive.TankDrive(speedL,speedR);
            auto now_time = std::chrono::system_clock::now();
            auto elapsed_seconds = now_time - prev_time;
        }
        // m_leftLeadMotor.Set(0);
        // m_rightLeadMotor.Set(0);
        m_robotDrive.TankDrive(0.0,0.0);
    }

    void arc_turn_right(float radius, float time, chrono::milliseconds time_ms, float theta) {
        float radius_left = radius + (wheelbase/2);
        float radius_right = radius - (wheelbase/2);

        float dist_left = theta * radius_left;
        float dist_right = theta * radius_right;

        float speed_left = dist_left/(2*3.14*(wheel_diam/2)*time); //rad/sec
        float speed_right = dist_right/(2*3.14*(wheel_diam/2)*time); //rad/sec

        auto now_time = std::chrono::high_resolution_clock::now();
        auto set_time = now_time + time_ms;
        bool should_turn = now_time < set_time;
        std::cout << "duration: " << std::chrono::duration_cast<chrono::milliseconds>(set_time - now_time).count() << std::endl;
        std::cout << "should turn: " << should_turn << std::endl;
        while(should_turn){
            float speedL = speed_left/3.14;
            float speedR = speed_right/3.14;
            // std::cout << "time: " << now_time << std::endl;
            // std::cout << "duration: " << duration << std::endl;
            // m_leftLeadMotor.Set(speedL);
            // m_rightLeadMotor.Set(speedR);
            m_robotDrive.TankDrive(speedL,speedR);
            now_time = std::chrono::high_resolution_clock::now();
            // double duration = (double)(set_time - now_time) / (double) CLOCKS_PER_SEC;
            should_turn = now_time < set_time;
        }
        // m_leftLeadMotor.Set(0);
        // m_rightLeadMotor.Set(0);

        m_robotDrive.TankDrive(0.0,0.0);
        sleep(1);
    }

    bool drive (float distance) {
        sleep(3);
        double encoder_goal = (distance)/(3.14*wheel_diam) * enc_res * 3.95 + leftLead_encoder.GetPosition();

        if(distance > 0) {

            float speed = 0.5;
            while((leftLead_encoder.GetPosition() <= encoder_goal + 5 || leftLead_encoder.GetPosition() <=encoder_goal - 5) && (rightLead_encoder.GetPosition() <= encoder_goal + 5 || rightLead_encoder.GetPosition() <= encoder_goal- 5)) {
                double errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                double errorRight = encoder_goal - rightLead_encoder.GetPosition();
                int k = 0.2;
                // m_leftLeadMotor.Set(speed - (errorLeft * k));
                float speedL = speed - (errorLeft * k);
                // m_rightLeadMotor.Set(speed - (errorRight * k));
                float speedR = speed - (errorRight * k);
                m_robotDrive.TankDrive(speedL,speedR);

            }
            // m_leftLeadMotor.Set(0);
            // m_rightLeadMotor.Set(0);
            m_robotDrive.TankDrive(0.0,0.0);
            sleep(3);
            done = 1;
            return true;
        }
        else {
            
            float speed = -0.5;
            m_robotDrive.TankDrive(speed, speed);

            while(encoder_goal - leftLead_encoder.GetPosition() <=0) {    
                double errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                double errorRight = encoder_goal - rightLead_encoder.GetPosition();
                // double errorLeft = abs(encoder_goal) - abs(leftLead_encoder.GetPosition());
                // int errorRight = abs(encoder_goal) - abs(rightLead_encoder.GetPosition());
                int k = 0.2;

                // m_leftLeadMotor.Set(-speed - (errorLeft * k));
                float speedL = speed + (errorLeft * k);
                // m_rightLeadMotor.Set(-speed - (errorRight * k));
                float speedR = speed + (errorLeft * k);
                m_robotDrive.TankDrive(speedL, speedR);

            }
            // m_leftLeadMotor.Set(0);
            // m_rightLeadMotor.Set(0);
            m_robotDrive.TankDrive(0.0,0.0);
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

    }
};

#ifndef RUNNING_FRC_TESTS   
int main() { return frc::StartRobot<Robot>(); }
#endif
