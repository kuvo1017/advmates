#include "GeneratingTable.h"
#include "Conf.h"
#include "Router.h"
#include "AmuStringOperator.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>

using namespace std;

//######################################################################
// GTCellクラス
//======================================================================
GTCell::GTCell():_begin(0),_end(0), _volume(0){}

//======================================================================
GTCell::~GTCell(){}

//======================================================================
bool GTCell::setValue(ulint begin, ulint end, int volume,
		      const string& start, const string& goal)
{
    if(begin <= end && volume >= 0)
    {
        _begin = begin;
        _end = end;
        _volume = volume;
    }
    else
    {
        _begin = 0;
        _end = 0;
        _od.clear();
        return false;
    }
  
    if(!_od.setValue(start, goal))
    {
        return false;
    }
  
    return true;
}

//======================================================================
bool GTCell::setValue(ulint begin, ulint end, int volume, int vehicleType,
		      const string& start, const string& goal,
		      const vector<string>& stopPoints)
{
    if(begin <= end && volume >= 0){
        _begin = begin;
        _end = end;
        _volume = volume;
        _vehicleType = vehicleType;
    }
    else
    {
        _begin = 0;
        _end = 0;
        _od.clear();
        return false;
    }
  
    if(!_od.setValue(start, goal, stopPoints))
    {
        return false;
    }

    return true;
}

//======================================================================
ulint GTCell::begin() const
{
    return _begin;
}
//======================================================================
ulint GTCell::end() const
{
    return _end;
}
//======================================================================
int GTCell::volume() const
{
    return _volume;
}
//======================================================================
int GTCell::vehicleType() const
{
    return _vehicleType;
}
//======================================================================
const string& GTCell::start() const
{
    return _od.start();
}
//======================================================================
const string& GTCell::goal() const
{
    return _od.goal();
}
//======================================================================
const vector<string>* GTCell::stopPoints() const
{
    return _od.stopPoints();
}
//======================================================================
const OD* GTCell::od() const
{
    return &_od;
}

//######################################################################
// GeneratingTableクラス
//======================================================================
GeneratingTable::GeneratingTable() :_presentTime(0){}

//======================================================================
GeneratingTable::~GeneratingTable()
{
    _table.clear();
    _presentCells.clear();
}

//======================================================================
void GeneratingTable::setTime(ulint time)
{
    _presentTime = time;
    _presentCells.clear();
    // tableから現在のtimeに関係する要素のみを抜きだして、_presentCellsにコピー
    vector<GTCell>::const_iterator where;
    for(where=_table.begin(); where!=_table.end(); where++)
    {
        if((*where).begin() <= _presentTime
           && (*where).end() > _presentTime)
        {
            _presentCells.push_back(*where);
        }
    }
}

//======================================================================
ulint GeneratingTable::time() const
{
    return _presentTime;
}

//======================================================================
int GeneratingTable::getTiming(const string& intersectionId,
			       vector<string>* result_goals,
			       vector<int>* result_volumes) const
{
    int result=0;
    //引数の初期化  
    (*result_goals).clear();
    (*result_volumes).clear();

    vector<GTCell>::const_iterator where;
    for(where=_presentCells.begin(); where!=_presentCells.end(); where++)
    {
        if((*where).start().compare(intersectionId)==0)
        {
            (*result_goals).push_back((*where).goal());
            (*result_volumes).push_back((*where).volume());
            result++;
        }
    }
    return result;
}

//======================================================================
void GeneratingTable::getActiveGTCells
(const string& intersectionId,
 vector<const GTCell*>* result_GTCells) const
{
    //引数の初期化  
    (*result_GTCells).clear();

    vector<GTCell>::const_iterator where;
    for(where=_presentCells.begin(); where!=_presentCells.end(); where++)
    {
        if((*where).start().compare(intersectionId)==0)
        {
            (*result_GTCells).push_back(&(*where));
        }
    }
}

//======================================================================
bool GeneratingTable::init(const string& fileName)
{
    bool result = false;
    fstream fin;
    fin.open(fileName.c_str(), ios::in);
    if(!fin.good())
    {
        cout << "no vehicle generate table file: "
             << fileName << endl;
        return false;
    }
    else
    {
        result = true;
    }
    vector<string> tokens;
    string str;
    while(fin.good())
    {
        getline(fin, str);
        //文字列の整形
        AmuStringOperator::getAdjustString(&str);
        if(!str.empty())
        {
            AmuStringOperator::getTokens(&tokens, str, ',');
        }
        if(tokens.size() >= 6)
        {
            int i;
            int curIndex;
            GTCell cell;
            ulint begin;
            ulint end;
            int volume;
            int vehicleType;
            string start;
            string goal;
            int numStoppingInters;
            vector<string> stopPoints;
            vector<string>::iterator it;

            // 発生開始、終了時刻
            begin = atoi(tokens[0].c_str());
            end = atoi(tokens[1].c_str());

            // 出発地、目的地
            ostringstream ost0, ost1;
            ost0.width(NUM_FIGURE_FOR_INTERSECTION);ost0.fill('0');
            ost1.width(NUM_FIGURE_FOR_INTERSECTION);ost1.fill('0');
            ost0 << (tokens[2]);
            ost1 << (tokens[3]);
            start = ost0.str();
            goal = ost1.str();

            // 発生量
            volume = atoi(tokens[4].c_str());

            // 車種ID
            vehicleType = atoi(tokens[5].c_str());

            // 経由地
            curIndex = 6;
            numStoppingInters = atoi(tokens[curIndex].c_str());
            curIndex++;
            stopPoints.reserve(numStoppingInters);
            for(i = 0; i < numStoppingInters; i++)
            {
                ostringstream ost;
                string stopPoint;
                ost.width(NUM_FIGURE_FOR_INTERSECTION);ost.fill('0');
                ost << (tokens[curIndex]);
                curIndex++;
                stopPoint = ost.str();
                stopPoints.push_back(stopPoint);
            }

            if(cell.setValue(begin, end, volume, vehicleType, 
                             start, goal, stopPoints))
            {
                _table.push_back(cell);
            }
            else
            {
                cerr << "Unknown error occured in GeneratingTable.\n"
                     << "begin, end, traffic volume, start, goal\n"
                     << begin << "\t" << end << "\t" << volume
                     << "\t" << start << "\t" << goal << endl;
            }
            tokens.clear();
        }
    }
    fin.close();
    return result;
}
