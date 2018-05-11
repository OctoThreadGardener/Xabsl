#ifndef XABSL_DEBUG_INTERFACE
#define XABSL_DEBUG_INTERFACE

#include "../Xabsl/XabslEngine/XabslEngine.h"
#include "../Xabsl/XabslEngine/XabslArray.h"
#include "../Xabsl/XabslEngine/XabslAgent.h"
#include "../Xabsl/XabslEngine/XabslAction.h"
#include "../Xabsl/XabslEngine/XabslOption.h"

#include <iostream>
#include <string>
#include <sstream>
#include <queue>

using namespace std;
using namespace xabsl;

class debug_interface
{
public:
	debug_interface(Engine *_pEngine)
		: pEngine(_pEngine)
	{	
		cout << "****************************************" << endl;
		cout << "* Running with debug_interface enabled *" << endl;
		cout << "****************************************" << endl;
	}

	~debug_interface()
	{	}

	stringstream &showDebugInfo();

private:
	void getDebugInfo();

	Engine *pEngine;

	Action *action;
	Option *option;
	State *activeState;
	int activeStateIndex;
	int depth;

	queue<int> activeStateIndexQ;
};

extern stringstream ss;

#endif //XABSL_DEBUG_INTERFACE