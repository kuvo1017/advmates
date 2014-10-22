#include <string>
#include <cassert>
#include "Signal.h"

using namespace std;

//======================================================================
Signal::Signal()
{
    _isReady = 1;
    _stateSet.resize(NUM_MAX_SPLIT_INT);
}

//======================================================================
Signal::Signal(const string& id)
{
    _id = id;
    _isReady = 1;
    _stateSet.resize(NUM_MAX_SPLIT_INT);
}

//======================================================================
Signal::~Signal(){}

//======================================================================
const string& Signal::id() const
{
    return _id;
}

//======================================================================
void Signal::setId(const string& id)
{
    _id = id;
    _isReady = 1;
    _stateSet.resize(NUM_MAX_SPLIT_INT);
}

//======================================================================
int Signal::numDirections() const
{
    if (_stateSet.empty())
    {
        return 0;
    }
    else
    {
        return _stateSet.front().numDirections();
    }
}

//======================================================================
int Signal::cycle() const
{
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    return aControlData.cycle();
}

//======================================================================
void Signal::setStateSet(const vector<SignalAspect>& stateSet)
{
    assert(stateSet.size() == NUM_MAX_SPLIT_ULINT);
    copy(stateSet.begin(), stateSet.end(), _stateSet.begin());
    _isReady++;
}

//======================================================================
void Signal::setSignalControlDataSet(const SignalControlDataSet& controlData)
{
    _controlData = controlData;
    _isReady++;
}

//======================================================================
bool Signal::isReady()
{
    bool result = false;
    if(_isReady == 3)
    {
        result = true;
    }
    return result;
}

//======================================================================
SignalMainState Signal::mainColor(const int direction) const
{
    // 下のforループのif文に引っかからなかったらデフォルトとして、最後の現示が返される。
    SignalMainState result = _stateSet[NUM_MAX_SPLIT_INT-1].mainColor(direction);
  
    // 現在時刻の取得
    ulint time = TimeManager::time();
  
    // 信号制御データのコンテナを取得。
    // （signals/*.msfから取得された情報。）
    const SignalControlData& aControlData = _controlData.aspectData(time);
    if (aControlData.cycle() <= 0)
    {
        cerr << "ERROR: Signal::mainColor: bad cycle length = " 
             << aControlData.cycle() << endl;
        crt();
        assert(0);
    }
  
    // 現在時刻の、現サイクル開始時からの経過時間(offset)を得る。
    ulint offset = (time - aControlData.begin()) % aControlData.cycle();
  
    // 上のoffsetを使って現在時刻が何個目の現示パターンに相当するかを判定する。
    for (int i=0; i < NUM_MAX_SPLIT_INT; i++)
    {
        if (offset < aControlData.split(i))
        {
            result = _stateSet[i].mainColor(direction);
            // コンストラクタおよびSignal::setStateSetにおいて
            // _stateSetがNUM_MAX_SPLIT_INT個の要素を持つことは（ほぼ）保証されている。
            break;
        }
        else
        {
            offset -= aControlData.split(i);
        }
    }
    return result;
}

//======================================================================
SignalSubState Signal::subColor(const int direction) const
{
    // 内容は上のmainColorと同じ
    SignalSubState result = _stateSet[NUM_MAX_SPLIT_INT-1].subColor(direction);
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    if (aControlData.cycle() <= 0)
    {
        cerr << "ERROR: Signal::subColor: bad cycle length = " 
             << aControlData.cycle() << endl;
        crt();
        assert(0);
    }
    ulint offset = (time - aControlData.begin()) % aControlData.cycle();
  
    for (int i=0; i < NUM_MAX_SPLIT_INT; i++)
    {
        if(offset < aControlData.split(i))
        {
            result = _stateSet[i].subColor(direction);
            break;
        }
        else
        {
            offset -= aControlData.split(i);
        }
    }
    return result;
}

//======================================================================
SignalWalkerState Signal::walkerColor(const int direction) const
{
    // 内容は上のmainColorと同じ
    SignalWalkerState result = _stateSet[NUM_MAX_SPLIT_INT-1].walkerColor(direction);
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    if (aControlData.cycle() <= 0)
    {
        cerr << "ERROR: Signal::walkerColor: bad cycle length = " 
             << aControlData.cycle() << endl;
        crt();
        assert(0);
    }
    ulint offset = (time - aControlData.begin()) % aControlData.cycle();
  
    for (int i=0; i < NUM_MAX_SPLIT_INT; i++)
    {
        if(offset < aControlData.split(i)) {
            result = _stateSet[i].walkerColor(direction);
            break;
        }
        else
        {
            offset -= aControlData.split(i);
        }
    }
    return result;
}

//======================================================================
SignalMainState Signal::prevMainColor(const int direction) const
{
    SignalMainState result = _stateSet[NUM_MAX_SPLIT_INT-1].mainColor(direction);
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    if (aControlData.cycle() <= 0)
    {
        cerr << "ERROR: Signal::prevMainColor: bad cycle length = " 
             << aControlData.cycle() << endl;
        crt();
        assert(0);
    }
    ulint offset = (time - aControlData.begin()) % aControlData.cycle();
  
    for (int i=0; i < NUM_MAX_SPLIT_INT; i++)
    {
        if (offset < aControlData.split(i))
        {
            result
                = _stateSet[(i+NUM_MAX_SPLIT_INT-1)%NUM_MAX_SPLIT_INT]
                .mainColor(direction);
            break;
        }
        else
        {
            offset -= aControlData.split(i);
        }
    }
    return result;
}

//======================================================================
SignalSubState Signal::prevSubColor(const int direction) const
{
    SignalSubState result = _stateSet[NUM_MAX_SPLIT_INT-1].subColor(direction);
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    if (aControlData.cycle() <= 0)
    {
        cerr << "ERROR: Signal::prevSubColor: bad cycle length = " 
             << aControlData.cycle() << endl;
        crt();
        assert(0);
    }
    ulint offset = (time - aControlData.begin()) % aControlData.cycle();
  
    for (int i=0; i < NUM_MAX_SPLIT_INT; i++)
    {
        if (offset < aControlData.split(i))
        {
            result
                = _stateSet[(i+NUM_MAX_SPLIT_INT-1)%NUM_MAX_SPLIT_INT]
                .subColor(direction);
            break;
        }
        else
        {
            offset -= aControlData.split(i);
        }
    }
    return result;
}

//======================================================================
SignalWalkerState Signal::prevWalkerColor(const int direction) const
{
    SignalSubState result = _stateSet[NUM_MAX_SPLIT_INT-1].walkerColor(direction);
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    if (aControlData.cycle() <= 0)
    {
        cerr << "ERROR: Signal::prevWalkerColor: bad cycle length = " 
             << aControlData.cycle() << endl;
        crt();
        assert(0);
    }
    ulint offset = (time - aControlData.begin()) % aControlData.cycle();
  
    for(int i=0; i < NUM_MAX_SPLIT_INT; i++)
    {
        if(offset < aControlData.split(i))
        {
            result
                = _stateSet[(i+NUM_MAX_SPLIT_INT-1)%NUM_MAX_SPLIT_INT]
                .walkerColor(direction);
            break;
        }
        else
        {
            offset -= aControlData.split(i);
        }
    }
    return result;
}

//=====================================================================
int Signal::hasPermission(const int direction) const
{
    int result = 0;
    if (mainColor(direction) == SignalColor::blue() 
        || mainColor(direction) == SignalColor::yellow())
    {
        result = 1;
    }
    else if (mainColor(direction) == SignalColor::yellowblink())
    {
        result = 2;
    }
    else if (mainColor(direction) == SignalColor::redblink())
    {
        result = 3;
    }
    return result;
}

//======================================================================
void Signal::crt() const
{
    // デバッグ用出力
    ulint time = TimeManager::time();
    const SignalControlData& aControlData = _controlData.aspectData(time);
    int cycl = aControlData.cycle();
    cout << " control data : id = " << _id;
    if (cycl == 0)
    {
        cout << endl 
             << " : begin = " << aControlData.begin() << endl
             << " : end   = " << aControlData.end()   << endl
             << " : cycle = " << cycl
             << " (data is not setted yet?)\n";
    }
    else
    {
        ulint offset = (time - aControlData.begin()) % aControlData.cycle();
        cout << " : cycle = " << cycl
             << " : offset = " << offset << endl;
        for (int i = 0; i<(int)_stateSet.size(); i++)
        {
            cout << " aspect = " << i 
                 << " : split = " << aControlData.split(i) << endl;
            _stateSet[i].crt();
        }
    }
}
