#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        enum ROBOT_STATE {IDLE, FORWARD, CHECKD_1, CHECKD_2, KNOCK, REVERSE, WAIT};
        enum APRILTAG {ROOM1, ROOM2, ROOM3, HOME, HALLWAY, NOTAG};
        enum DIRECTION {RIGHT, LEFT, FRONT};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE

        //FLAGS
        int target_room = -1;
        int collisions = 0;
        int in_hallway = 0;
        int confirm_delivery = 0;
        float distR = 0;
        float distL = 0;
        float distF = 0;
        int april = NOTAG;
         
    public:
        void Init(void); /// a and b
        void Stop(void);
        void Run(void);
        void setTargetRoom(void);
        void setConfirmDelivery(void);
        void setWallDistance(enum DIRECTION);
        int collisionDetected(void);
        int getAprilTag(void);
};

#endif