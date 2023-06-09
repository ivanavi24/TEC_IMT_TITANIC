


#define Z_SAFE_POS_UP      0.15 /*Z upper pose for sequence*/
#define Z_SAFE_POS_DOWN      0.15 /*Z upper pose for sequence*/

/*Deposit coordinates*/
#define X_POS_DEPOSIT      0.15 /*Z upper pose for sequence*/
#define Y_POS_DEPOSIT      0.15 /*Z upper pose for sequence*/


/*Movement safe position*/
#define X_SAFE_POSITION_MOVEMENT        0.2
#define Y_SAFE_POSITION_MOVEMENT        0.55
#define Z_SAFE_POSITION_MOVEMENT        0.05

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H


enum craneState
{
    sailing,
    exploring,
    recolection
};


class stateMachine
{


    private:
    craneState currentState;

    public:
    
    stateMachine();
    craneState getCurrentState();
    craneState determineNextState();
    void changeState(craneState desired_state);
    

};

#endif