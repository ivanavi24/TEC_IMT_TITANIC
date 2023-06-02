


#define safe_pos_upper      0.15 /*Z upper pose for sequence*/
#define los_pos_hold      0.15 /*Z upper pose for sequence*/
#define X_POS_DEPOSIT      0.15 /*Z upper pose for sequence*/
#define Y_POS_DEPOSIT      0.15 /*Z upper pose for sequence*/
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H


enum craneState
{
    sailing,
    scaning,
    arm2target,
    descending,
    holding,
    rising,
    arm2deposit,
    holdingoff,
};


class stateMachine
{


    private:
    craneState currentState;

    public:
    
    stateMachine();
    void changeState();

};

#endif