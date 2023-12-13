#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        enum ROBOT_STATE {IDLE, FORWARD, CHECKD_1, CHECKD_2, KNOCK, REVERSE, WAIT};
        enum APRILTAG {HOME = 0, ROOM1 = 1, ROOM2 = 2, ROOM3 = 3, HALLWAY = 4, NOTAG};
        enum DIRECTION {RIGHT, LEFT, FRONT};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE

        //FLAGS
//FLAGS
        int target_room = -1;
        int collisions = 0;
        bool in_hallway = false;
        bool confirm_delivery = false;
        float distR = 0;
        float distL = 0;
        float distF = 0;
        int april = NOTAG;
        String serString1;
        
    public:
        void Init(void); /// a and b
        void Stop(void);
        void Run(void);
        void setFlags(void);
        void setWallDistance(enum DIRECTION);
        int collisionDetected(void);
        int getAprilTag(void);
        bool checkSerial1(void);
        void setup();
};

#endif