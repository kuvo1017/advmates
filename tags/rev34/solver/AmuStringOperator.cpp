#include "AmuStringOperator.h"
#include <iostream>

using namespace std;

//======================================================================
void AmuStringOperator::getStrippedString(string* result_str,
                                          const char mark)
{
    string::size_type t;
    for(t=result_str->find(mark);
        t!=string::npos;
        t=result_str->find(mark))
    {
        result_str->erase(t, 1);
    }
}

//======================================================================
void AmuStringOperator::getStrippedString(string* result_str,
                                          const string& mark)
{
    string::size_type t;
    for(t=result_str->find(mark);
        t!=string::npos;
        t=result_str->find(mark))
    {
        result_str->erase(t, mark.size());
    }
}

//======================================================================
void AmuStringOperator::getStrippedStringForEach(string* result_str,
                                                 const string& marks)
{
    string::size_type t;
    for(t=result_str->find_first_of(marks);
        t!=string::npos;
        t=result_str->find_first_of(marks))
    {
        result_str->erase(t, 1);
    }
}

//======================================================================
void AmuStringOperator::getSubstantialString(string* result_str,
                                             const string& comment_mark)
{
    string::size_type t;
    t=result_str->find(comment_mark);
    if(t!=string::npos)
    {
        result_str->erase(t);
    }
}

//======================================================================
void AmuStringOperator::getSubstantialString(string* result_str,
                                             const char comment_mark)
{
    string::size_type t;
    t=result_str->find(comment_mark);
    if(t!=string::npos)
    {
        result_str->erase(t);
    }
}

//======================================================================
void AmuStringOperator::getTokens(vector<string>* result_str,
                                  const string& str,
                                  const char delimiter)
{
    result_str->clear();
    string::size_type begin = 0, end = string::npos;
    end = str.find(delimiter,0);
    if(end != string::npos)
    {
        while(end != string::npos)
        {
            (*result_str).push_back(str.substr(begin, end-begin));
            begin = end+1;
            end = str.find(delimiter, end+1);
        }
        (*result_str).push_back(str.substr(begin, string::npos));
    }
    else
    {
        (*result_str).push_back(str);
    }
}

//======================================================================
void AmuStringOperator::getTokens(vector<string>* result_str, 
                                  const string& str,
                                  const string& delimiter)
{
    result_str->clear();
    string::size_type begin = 0, end = string::npos;
    end = str.find(delimiter,0);
    if(end != string::npos)
    {
        while(end != string::npos)
        {
            (*result_str).push_back(str.substr(begin, end-begin));
            begin = end+1;
            end = str.find(delimiter, end+1);
        }
        (*result_str).push_back(str.substr(begin, string::npos));
    }
    else
    {
        (*result_str).push_back(str);
    }
}

//======================================================================
void AmuStringOperator::getAdjustString(string* result_str)
{
    // コメント
    getSubstantialString(result_str, '#');
    // 読み飛ばし
    getStrippedStringForEach(result_str, " \t\n\r\f");
}
