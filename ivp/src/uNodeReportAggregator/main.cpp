/************************************************************/
/* NAME: Maximilian von Neukirch                            */
/* ORGN: Cyber-Physical Systems Group Universiät Konstanz   */
/* FILE: main.cpp                                           */
/* DATE: 05.07.2026                                         */
/************************************************************/

#include <string>
#include <iostream>
#include "MBUtils.h"
#include "ColorParse.h"
#include "NodeReportAggregator.h"

using namespace std;

int main(int argc, char *argv[])
{
    string mission_file;
    string run_command = argv[0];

    for(int i=1; i<argc; i++) {
        string argi = argv[i];
        
        if((argi=="-v") || (argi=="--version") || (argi=="-version")) {
            cout << "uNodeReportAggregator Version 1.0" << endl;
            return(0);
        }
        else if((argi == "-h") || (argi == "--help") || (argi=="-help")) {
            cout << "Usage: uNodeReportAggregator file.moos [options]" << endl;
            return(0);
        }
        else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++")) {
            mission_file = argv[i];
        }
        else if(strBegins(argi, "--alias=")) {
            run_command = argi.substr(8);
        }
        else if(i==2) {
            run_command = argi;
        }
    }

    if(mission_file == "") {
        cout << "Error: No .moos file provided!" << endl;
        cout << "Usage: ./uNodeReportAggregator file.moos" << endl;
        return(1);
    }

    cout << termColor("green");
    cout << "uNodeReportAggregator launching as " << run_command << endl;
    cout << termColor() << endl;

    NodeReportAggregator uNodeReportAggregatorApp;
    
    uNodeReportAggregatorApp.Run(run_command.c_str(), mission_file.c_str());

    return(0);
}