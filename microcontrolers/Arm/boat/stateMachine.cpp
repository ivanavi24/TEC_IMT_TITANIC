
#include "stateMachine.h"


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
    case recolection:
        /* Return same state since control aims to remain in same pos  */
        return recolection;
        break;    
    default:
        break;
    }
}
void stateMachine::changeState(craneState desired_state)
{
    switch (currentState)
    {
    case sailing:
        /* Return same state since control aims to remain in same pos  */
        break;
    case recolection:
        /* Return same state since control aims to remain in same pos  */
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