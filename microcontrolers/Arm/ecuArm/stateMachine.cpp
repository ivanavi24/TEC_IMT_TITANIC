
#include "stateMachine.h"

#include "Crane3dof.h"
extern Crane3dof titanicCrane;
stateMachine::stateMachine()
{
    currentState = sailing;
}


craneState stateMachine::determineNextState(){
    switch (currentState)
    {
    case sailing:
        /* Return same state since control aims to remain in same pos  */
        return sailing;
        break;
    case scaning:
        /* Return same state since control aims to remain in same pos  */
        return scaning;
        break;    
    case arm2target: 
        return descend;
        break;
    case descend:
        /*Descend arm to pos to hold*/
        return holding;
        break;

    case holding:
        return rising;
        break;
    case rising: /*Rising finished, now move to deposit*/
        return arm2deposit;
        break;
    case arm2deposit:
        return holdingoff;
        break;
    case holdingoff:
        return scaning;
        break;
    default:
        break;
    }
}
void stateMachine::changeState(craneState desired_state)
{
    switch (desired_state)
    {
    case sailing:
        /* Move arm to safe sailing position */
        titanicCrane.stopAllMotors();
        break;
    case scaning:
        /* Move arm to safe sailing position */
        /* Move camera pan servo to scan */
        titanicCrane.inverse_kinematics(
            X_SAFE_POSITION_MOVEMENT,
            Y_SAFE_POSITION_MOVEMENT,
            Z_SAFE_POSITION_MOVEMENT
        );
        break;    
    case arm2target: 
        /*Move arm to target pos set by I2C Interruption*/
        titanicCrane.inverse_kinematics(
            titanicCrane.getXdesired(),
            titanicCrane.getYdesired(),
            Z_SAFE_POS_UP
        );
        break;
    case descend:
        /*Descend arm to pos to hold*/
        titanicCrane.inverse_kinematics(
            titanicCrane.getXdesired(),
            titanicCrane.getYdesired(),
            Z_SAFE_POS_DOWN
        );
        break;
        currentState = holding;
        /*Close gripper action here*/
        break;

    case holding:
        currentState = rising;
        /*Close gripper with timer here*/
        break;
    case rising: /*Rising finished, now move to deposit*/
        currentState = arm2deposit;
        /*Action of the next state*/
        /*Descend arm to pos to hold*/
        titanicCrane.inverse_kinematics(
            titanicCrane.getXdesired(),
            titanicCrane.getYdesired(),
            Z_SAFE_POS_UP
        );
        break;
    case arm2deposit:
        titanicCrane.inverse_kinematics(
            X_POS_DEPOSIT,
            Y_POS_DEPOSIT,
            Z_SAFE_POS_UP
        );
        currentState = holdingoff;
        
        break;
    case holdingoff:
        /*Close gripper here*/
        break;
    default:
        break;
    }
    currentState = desired_state;
}


craneState stateMachine::getCurrentState()
{
    return currentState;
};