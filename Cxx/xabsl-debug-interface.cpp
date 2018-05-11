#include "xabsl-debug-interface.h"

stringstream ss;

void debug_interface::getDebugInfo()
{
	action = pEngine->getRootAction(0);
	option = action->getOption();
	activeState = option->activeState;
	activeStateIndex = activeState->index;

	while (true)
	{
		activeStateIndexQ.push(activeStateIndex);

		option = option->states[activeStateIndex]->subsequentOption;
		if (option == NULL)
		{
			break;
		}
		else
		{
			activeState = option->activeState;
			activeStateIndex = activeState->index;
		}
	}

	depth = activeStateIndexQ.size();
}

stringstream &debug_interface::showDebugInfo()
{
	getDebugInfo();
	int i;

	ss.str("");

	ss << "*************Debug Information*************" << endl;
    for (i = 0; i < 6; i++)
	{
		ss << "*	Active state[layer ";
		ss << i;
		ss << "]: ";
		if (i < depth)
		{
			ss << activeStateIndexQ.front();
			ss << "\n";
			activeStateIndexQ.pop();
		}
		else
		{
			ss << "\n";
		}
	}

	ss << "*******************************************" << endl;
	return ss;
}
