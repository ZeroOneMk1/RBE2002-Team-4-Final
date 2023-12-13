#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Position_estimation.h"
#include "apriltagdatum.h"
#include "openmv.h"
#include "Wire.h"
#include "IR_sensor.h"

#define THRESHOLD_HIGH 60
#define THRESHOLD_LOW 50
#define WALL_THRESHOLD 15
//sensors
Romi32U4ButtonA buttonA;

//camera 
OpenMV camera;

//IR sensor
IRsensor ir_sensor;

//motor-speed controller
SpeedController robot;

// Serial String

bool Behaviors::checkSerial1(void)
{
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n')
        {
            return true;
        }
    }

    return false;
}

void Behaviors::setupESP() 
{
    Serial.begin(115200);
    delay(100);  //give it a moment to bring up the Serial

    Serial.println("setup()");

    Serial1.begin(115200);
    digitalWrite(0, HIGH); // Set internal pullup on RX1 to avoid spurious signals

    Serial.println("/setup()");
}

void Behaviors::Init(void)
{
    robot.Init();
    ir_sensor.Init();
    setupESP();
    delay(1000);
    Wire.begin();
    Wire.setClock(100000ul);
    // robot.Init();
}

void Behaviors::Stop(void)
{
    // robot.Stop();
}

void Behaviors::setWallDistance(enum DIRECTION dir)
{
    // Sets appropriate wall distance global to the current distance sensor value.
    delay(100);
    if (dir == LEFT){
        distL = ir_sensor.ReadData();
        Serial.println("LEFTDIST");
        Serial.println(distL);
    }else if(dir == RIGHT){
        distR = ir_sensor.ReadData();
        Serial.println("RIGHTDIST");
        Serial.println(distR);
    }else if(dir == FRONT){
        distF = ir_sensor.ReadData();
    }
}

int Behaviors::collisionDetected(void)
{
    // Return true if the IMU records a high spike in acceleration, else returns false.
    return 1; // TODO Whoever's first do this
}

// Returns the april tag the Camera is seeing, returning NOTAG if none
Behaviors::APRILTAG Behaviors::getAprilTag(void)
{
    uint8_t numTags = camera.getTagCount();
    //    Serial.println(numTags);
    AprilTagDatum aprltag;
    camera.readTag(aprltag);
    uint16_t tagID = aprltag.id;
    if(tagID <=4 && numTags > 0)
    {
        return (APRILTAG)tagID;
    }
    return NOTAG; 
}

void Behaviors::setFlags(void)
{
    if(checkSerial1()) {
        if (serString1.startsWith("Room:")) {
            Serial1.print("EDITING ROOM");
            // Parse the message and extract the room information
            int roomValue = serString1.substring(5).toInt();
            // Store the result in target_room
            switch(roomValue){
                case 1:
                    target_room = ROOM1;
                    break;
                case 2:
                    target_room = ROOM2;
                    break;
                case 3:
                    target_room = ROOM3;
                    break;
                default:
                    target_room = -1;
            }
        } else if(serString1.startsWith("DeliveryCofirm:")){
            Serial1.print("EDITING CONFIRM");
            bool conf = (bool) (serString1.substring(16).toInt());
            if(conf){
                confirm_delivery = true;
            }else{
                confirm_delivery = false;
            }
        }
        serString1 = "";
    }
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        // Serial.print("idle");
        // The idle state. Robot starts on A button press.
        // setFlags() reads MQTT and sets the target room.
        if(buttonA.getSingleDebouncedRelease()){ 
            Serial.println("button!");
            //delay(1000);
            robot_state = FORWARD;
            // setFlags();
            // if(target_room == -1){
            //     // IF FAILS TO SET ROOM, DONT START
            //     robot_state = IDLE;
            // }
            // robot.Stop();             
        } 
        else { 
        //    Serial.println("nobutton");
            //setFlags();
            // delay(500);
            // Serial.print("Target Room: ");
            // Serial.print(target_room);
            // Serial.print("\n");
            // Serial.print("Confirm Delivery: ");
            // Serial.print(confirm_delivery);
            // Serial.print("\n");
            // robot_state = IDLE;
            // robot.Stop(); 
        }   
        break;
    
    case FORWARD:
        // Robot goes forward until: It's too close to the front wall -> Turn 90 degrees, CHECKD_1
        //                           It is at the HOME location -> IDLE
        //                           It's in the hallway and in front of the outliar room -> Turn 90 degrees, CHECKD_1
        /**
        * @param april stores the current april tag that's in front of it
        *        HALLWAY = hallway tag
        *        HOME    = home waypoint tag
        *        NOTAG   = no tag seen
        * @param in_hallway the flag that tells it to check for the outliar room
        **/
        setWallDistance(FRONT);
        april = getAprilTag();
        //robot.LineFollow(100);

        if(april == HALLWAY){
            in_hallway = 1;
        }

        if(buttonA.getSingleDebouncedRelease()) {
            Serial.println("double pressed");
            robot_state = IDLE;
            robot.Stop();
        }else if(WALL_THRESHOLD > distF){
            Serial.print("Something");
            if(april == HOME){
                robot_state = IDLE;
                robot.Stop();
            }else{
                robot_state = CHECKD_1;
                Serial.println("turning 90 Right");
                robot.Turn(90, 0);
                robot.Stop();
            }
        }else if(distF < THRESHOLD_HIGH && distF > THRESHOLD_LOW && in_hallway){
            // Serial.print("Hallway");
            robot_state = CHECKD_1;
            Serial.println("turning 90 Right HALLWAY");
            robot.Turn(90, 0);
            Serial.println("DONE");
            delay(10000);
            robot.Stop();
        }else{
            robot.LineFollow(100);
            // Serial.print("line");
        }
        break;

    case CHECKD_1:
        // Robot Checks distance ahead of it and Tag: Tag = target room -> KNOCK
        //                                            Tag = Hallway -> set hallway to true
        //                                            Tag = Home -> IDLE
        //                                            Tag = NOTAG -> Turn 180 degrees, CHECKD_2
        /**
        * @param april stores the current april tag that's in front of it
        * @param in_hallway true/false flag if in hallway.
        **/
        setWallDistance(RIGHT);
        april = getAprilTag();

        if(april == target_room){
            robot_state = KNOCK;
            robot.Stop();
        }else if(april == HALLWAY){
            in_hallway = 1;
        }else if(april == HOME){
            robot.Stop();
            robot_state = IDLE;
        }else{
            Serial.println("turning 180 Left");
            robot.Turn(90, 1); // CHANGE 180
            robot.Turn(90, 1); // CHANGE 180
            robot.Stop();
            
            robot_state = CHECKD_2;
        }
        break;

    case CHECKD_2:
        // Robot Checks distance ahead of it and Tag: Tag = target room -> KNOCK
        //                                            Tag = Hallway -> set hallway to true
        //                                            Tag = Home -> IDLE
        //                                            Tag = NOTAG -> Turn to larger distance value, FORWARD
        /**
        * @param april stores the current april tag that's in front of it
        * @param in_hallway true/false flag if in hallway.
        * @param distR one of two distances. Stores distance from robot to closest wall to its right and vice versa. Used to turn to the wall that's further away.
        **/
        setWallDistance(LEFT);
        april = getAprilTag();
        Serial.println(april);

        if(april == target_room){
            robot_state = KNOCK;
            robot.Stop();
        }else if(april == HALLWAY){
            in_hallway = 1;
        }else if(april == HOME){
            robot.Stop();
            robot_state = IDLE;
        }else{
            if(distR > distL){
                Serial.println("R > L -> Turning 180 RIGHT");
                robot.Turn(90, 0); // TODO get direction
                robot.Turn(90, 0); // TODO get direction
                robot.Stop();
                robot_state = FORWARD;
            }else if(distL > distR){
                Serial.println("L > R -> Going forward");
                robot.Stop();
                robot_state = FORWARD;
            }else{
                Serial.println("L == R -> Stopping");
                robot_state = IDLE;
                robot.Stop();
            }
            robot.Stop();
        }
        break;

    // case KNOCK:
    //     // Robot runs forward until it collides with a wall
    //     /**
    //     * @param collisions stores the number of collisions that have happened. Used to knock 3 times.
    //     **/
    //     robot.Run(200, 200);
    //     if(collisionDetected()){
    //         robot_state = REVERSE;
    //         robot.Stop();
    //         collisions += 1;
    //     }
    //     break;
    // case REVERSE:
    //     // Robot reverses until WALLDIST away. IF collisions >= 3 -> reset collisions, WAIT
    //     //                                     ELSE -> KNOCK
    //     /**
    //     * @param collisions stores the number of collisions that have happened. Used to knock 3 times.
    //     * @param WALLDIST 
    //     **/
    //     setWallDistance(FRONT);
    //     if(distF < ir_sensor.ReadData()){
    //         robot.LineFollow(-100);
    //     }else{
    //         if(collisions >= 3){
    //             collisions = 0;
    //             robot_state = WAIT;
    //             robot.Stop();
    //         }else{
    //             robot_state = KNOCK;
    //             robot.Stop();
    //         }
    //     }
    //     break;
    // case WAIT:
    //     // Robot waits until it receives an OK from the ESP32 to return.
    //     /**
    //     * @param target_room the april tag enum of the room we're trying to find.
    //     * @param confirm_delivery the flag that stores if delivery is confirmed via MQTT 
    //     * @param setFlags() sets if the delivery was confirmed.
    //     **/
    //     setFlags();
    //     if(confirm_delivery){
    //         target_room = -1;
    //         robot.Stop();
    //         robot_state = FORWARD;
    //     }
    //     break;
    }
}