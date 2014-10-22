#include "AppSim.h"
#include <iostream>
#include <cstdlib>
#include <cassert>

using namespace std;

//======================================================================
void AppSim::init(int argc, char** argv, bool output)
{
    AppMates::init(argc, argv, false);
    assert(_simulator);
    _vis.reset(new Visualizer());
}

//======================================================================
void AppSim::parseArgument(int argc, char** argv)
{
    /*
     * AppSim独自のオプションを使用する場合はここに記述する．
     * これはApp::initから呼ばれる．
     */
}

//======================================================================
void AppSim::printUsage()
{
    cout <<
        "Usage  : ./advmates-sim [Option list] \n"
         << endl;
    AppMates::printUsage();
    exit(EXIT_SUCCESS);
}

//======================================================================
int AppSim::run()
{
    if (_vis.get())
    {
        _vis->setSimulator(_simulator);
        _vis->visualize();
        return EXIT_SUCCESS;
    }
    else
    {
        cerr << "Visualizer not found." << endl;
        return EXIT_FAILURE;
    }
}
