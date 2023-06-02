
#include "stateMachine.h"

#include "Crane3dof.h"
extern Crane3dof titanicCrane;
stateMachine::stateMachine()
{
    currentState = sailing;
}



void stateMachine::changeState()
{
    switch (currentState)
    {
    case sailing:
        /* code */
        break;
    case scaning:/*Scaning find something or time out, again to sailing*/
        /* code */

        titanicCrane.set_target_position(
            titanicCrane.getXdesired(),
            titanicCrane.getYdesired(),
            los_pos_hold
        );
        break;    
    case arm2target: /*Close gripper action here*/
        currentState = descending;
        /*Action of the next state*/
        titanicCrane.set_target_position(
            titanicCrane.getXdesired(),
            titanicCrane.getYdesired(),
            los_pos_hold
        );
        break;
    case descending:
        currentState = holding;
        /*Close gripper action here*/
        break;

    case holding:
        currentState = rising;
        /*Action of the next state*/
        titanicCrane.set_target_position(
            titanicCrane.getXdesired(),
            titanicCrane.getYdesired(),
            safe_pos_upper
        );
        break;
    case rising: /*Rising finished, now move to deposit*/
        currentState = arm2deposit;
        /*Action of the next state*/
        titanicCrane.set_target_position(
            X_POS_DEPOSIT,
            X_POS_DEPOSIT,
            safe_pos_upper
        );
        break;
    case arm2deposit:
        currentState = holdingoff;
        break;
    case holdingoff:
        /* code */
        break;
    default:
        break;
    }
}