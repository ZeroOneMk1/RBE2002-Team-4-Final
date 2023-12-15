#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){

        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        float dleft = e_left - prev_e_left;
        float dright = e_right - prev_e_right;

        E_left += e_left;
        E_right += e_right;

        

        float u_left = Kp*e_left + Ki*E_left + Kd * dleft;
        float u_right = Kp*e_right + Ki*E_right + Kd * dright;

        prev_e_left = e_left;
        prev_e_right = e_right;

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right); //this is where your newly programmed function is/will be called
    }
}

boolean SpeedController::Turn(int degree, int direction)
{
    E_left = 0;
    E_right = 0;
    motors.setEfforts(0, 0);
    long int turns = counts*(degree/180.0); // ! CALIBRATE COUNTS ON SITE
    MagneticEncoder.UpdateEncoderCounts();
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while((abs((count_turn) - (MagneticEncoder.ReadEncoderCountLeft()))) <= turns)
    {
        if(!direction) Run(50,-50);
        else Run(-50,50);
    }

    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time) //in mm/s and s
{
    E_left = 0;
    E_right = 0;
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    E_left = 0;
    E_right = 0;
    motors.setEfforts(0, 0);
    
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    E_left = 0;
    E_right = 0;
    prev_e_left = 0;
    prev_e_right = 0;
    motors.setEfforts(0,0);
    // odometry.Stop();
}


// put function definitions here;
void SpeedController::LineFollow(int speed) {
    if(MagneticEncoder.UpdateEncoderCounts()){
        float error = 0;

        if (analogRead(leftReflectance) >= 300 || analogRead(rightReflectance) >= 300)
        {
        
            error = (analogRead(leftReflectance) - analogRead(rightReflectance)) * linefollowkp;
        }

        float e_left = speed - MagneticEncoder.ReadVelocityLeft() + error;
        float e_right = speed - MagneticEncoder.ReadVelocityRight() - error;
        
        float dleft = e_left - prev_e_left;
        float dright = e_right - prev_e_right;

        E_left += e_left;
        E_right += e_right;

        

        float u_left = Kp*e_left + Ki*E_left + Kd * dleft;
        float u_right = Kp*e_right + Ki*E_right + Kd * dright;

        prev_e_left = e_left;
        prev_e_right = e_right;

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(speed,speed); //this is where your newly programmed function is/will be called
    }
}


  
  
