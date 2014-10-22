#include "GVManager.h"
#include "AmuStringOperator.h"
#include "AmuConverter.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <cassert>

using namespace std;

//======================================================================
GVManager::GVManager()
{
    _maxTime = 0;
}

//======================================================================
GVManager::~GVManager(){}

//======================================================================
GVManager::GVManager(const GVManager& rhs)
    :_strings(rhs._strings), _numerics(rhs._numerics){}

//======================================================================
GVManager& GVManager::operator=(const GVManager& rhs)
{
    _strings = rhs._strings;
    _numerics = rhs._numerics;
    return *this;
}

//======================================================================
GVManager& GVManager::instance()
{
    static GVManager instance;
    return instance;
}

//======================================================================
bool GVManager::getVariable(const string& key, string* value)
{
    bool result = false;
    map<string, string>::iterator where;

    //キーを探す
    where = _strings.find(key);
    if(where != _strings.end())
    {
        *value = (*where).second;
        result = true;
    }
    return result;
}

//======================================================================
bool GVManager::getVariable(const string& key, double* value)
{
    bool result = false;
    map<string, double>::iterator where;

    //キーを探す
    where = _numerics.find(key);
    if(where != _numerics.end())
    {
        *value = (*where).second;
        result = true;
    }
    return result;
}

//======================================================================
double GVManager::getNumeric(const string& key)
{
    map<string, double>::iterator where;

    // キーを探す
    where = _numerics.find(key);
    if (where != _numerics.end())
    {
        return (*where).second;
    }
    else
    {
        cerr << "GVManager: " << key << " was not declared." << endl;
        assert(0);
    }
}

//======================================================================
unsigned long GVManager::getMaxTime() const
{
    return _maxTime;
}

//======================================================================
bool GVManager::setNewVariable(const string& key, 
			       const string& value)
{
    bool result = false;
    map<string, string>::iterator where;

    //キーを探して、重複がなければ設定
    where = _strings.find(key);
    if(where == _strings.end())
    {
        _strings.insert(make_pair(key, value));
        result = true;
    }
    return result;
}

//======================================================================
bool GVManager::setNewVariable(const string& key, 
			       const double value)
{
    bool result = false;
    map<string, double>::iterator where;

    //キーを探して、重複がなければ設定
    where = _numerics.find(key);
    if(where == _numerics.end())
    {
        _numerics.insert(make_pair(key, value));
        result = true;
    }
    return result;
}

//======================================================================
bool GVManager::resetVariable(const string& key, const string& value)
{
    bool result = false;
    map<string, string>::iterator where;

    //キーを探して上書き
    where = _strings.find(key);
    if(where != _strings.end())
    {
        (*where).second = value;
        result = true;
    }
    return result;
}

//======================================================================
bool GVManager::resetVariable(const string& key, const double value)
{
    bool result = false;
    map<string, double>::iterator where;

    //キーを探して上書き
    where = _numerics.find(key);
    if(where != _numerics.end())
    {
        (*where).second = value;
        result = true;
    }
    return result;
}

//======================================================================
bool GVManager::setVariablesFromFile(const string& fileName)
{
    // ファイルを読み込む
    ifstream ifs(fileName.c_str(), ios::in);
    if (!ifs)
    {
        cout << "no init file: " << fileName << endl;
        return false;
    }

    string str;
    while (ifs.good())
    {
        // 1行毎の処理
        getline(ifs, str);
        AmuStringOperator::getAdjustString(&str);
        if (!str.empty())
        {
            vector<string> tokens;
            AmuStringOperator::getTokens(&tokens, str, '=');
            if (tokens.size()!=2)
            {
                cerr << "invalid format of global variable: "
                     << str << endl;
                cerr << "format is <symbol = value>" << endl;
            }
            else
            {
                if (!tokens[0].empty() && !tokens[1].empty())
                {
                    if (tokens[0]=="MAX_TIME")
                    {
                        _maxTime = AmuConverter::strtoul(tokens[1]);
                        continue;
                    }
                    // 数値か文字列か判断し，
                    // 数値であれば_numericsに，
                    // 文字列であれば_stringsに登録する
                    bool isNumeric = false;
                    if (isdigit(tokens[1][0])
                        || ( tokens[1].size()>=2
                             && tokens[1][0]=='-'
                             && isdigit(tokens[1][1])))
                    {
                        stringstream ss;
                        double d;
                        string s="";
                        ss << tokens[1];
                        ss >> d >> s;
                        if (s=="")
                        {
                            // 数値の登録
                            // キーが無ければ作成し，既にあれば上書きする
                            isNumeric = true;
                            bool isSet = setNewVariable(tokens[0], d);
                            if (!isSet)
                                isSet = resetVariable(tokens[0], d);
                        }
                    }
                    if (!isNumeric)
                    {
                        // 文字列の登録
                        // キーが無ければ作成し，既にあれば上書きする
                        bool isSet = setNewVariable(tokens[0], tokens[1]);
                        if (!isSet)
                            isSet = resetVariable(tokens[0], tokens[1]);
                    }
                }
            }
        }
    }
    return true;
}

//======================================================================
void GVManager::print() const
{
    cout << "*** Global Variables ***" << endl;
    if (_maxTime>=100)
    {
        cout << "MAX_TIME=" << _maxTime << endl;
    }
    {
        map<string, string>::const_iterator where;
        for (where=_strings.begin(); where!=_strings.end(); where++)
        {
            cout << (*where).first << ":" << (*where).second << endl;
        }
    }
    {
        map<string, double>::const_iterator where;
        for (where=_numerics.begin(); where!=_numerics.end(); where++)
        {
            cout << (*where).first << "=" << (*where).second << endl;
        }
    }
}
