//単路

#ifndef ROAD_H
#define ROAD_H

#include <Qt>
#include <string>
#include <vector>

#define QT_FATAL_ASSERT

using namespace std;

class Intsec;

//交差点種別（ID大）
#define ROAD_IT_SMALL   0                                   //ID小
#define ROAD_IT_LARGE   1                                   //ID大
#define ROAD_IT_NUM     2                                   //種別数
#define ROAD_IT_NONE    -1                                  //種別なし
#define ROAD_IT_REVERSE(it) (1 - (it))                      //逆転

//車線種別
#define ROAD_LT_OUT     0                                   //単路から出る
#define ROAD_LT_IN      1                                   //単路へ入る
#define ROAD_LT_NUM     2                                   //車線種別

//単路
class Road
{
    int         _id;                                                //単路ID
    Intsec*     _intsec[ROAD_IT_NUM];                               //交差点
    int         _lane[ROAD_IT_NUM][ROAD_LT_NUM];                    //車線数
    bool        _laneFlag[ROAD_IT_NUM][ROAD_LT_NUM];                //車線数設定フラグ、設定漏れチェック用
    int         _error;                                             //エラー種別

public:
    Road(int id, Intsec* intsec1, Intsec* intsec2, int laneDefault);
                                                                    //コンストラクタ
    int id()                                                        //単路ID取得
    {
        return _id;
    }
    Intsec* intsec(int intsecType)                                  //交差点取得
    {
        Q_ASSERT(0 <= intsecType && intsecType < ROAD_IT_NUM);
        return _intsec[intsecType];
    }
    int intsecType(int id);                                         //交差点種別取得、なしなら ROAD_IT_NONE

    void setLane(int intsecType, int laneType, int lane);           //車線数設定、laneチェックなし
    void addLane(int intsecType, int laneType, int lane)            //車線数追加
    {
        Q_ASSERT(0 <= intsecType && intsecType < ROAD_IT_NUM);
        Q_ASSERT(0 <= laneType && laneType < ROAD_LT_NUM);
        setLane(intsecType, laneType, _lane[intsecType][laneType] + lane);
    }
    void addLanes(Intsec* intsec1, int lane1Out, int lane1In,
                                   int lane2Out, int lane2In);
                                                                    //車線数まとめて追加
    int lane(int intsecType, int laneType)                          //車線数取得
    {
        Q_ASSERT(0 <= intsecType && intsecType < ROAD_IT_NUM);
        Q_ASSERT(0 <= laneType && laneType < ROAD_LT_NUM);
        return _lane[intsecType][laneType];
    }
    bool laneFlag(int intsecType, int laneType)                     //車線数設定フラグ取得
    {
        Q_ASSERT(0 <= intsecType && intsecType < ROAD_IT_NUM);
        Q_ASSERT(0 <= laneType && laneType < ROAD_LT_NUM);
        return _laneFlag[intsecType][laneType];
    }

    bool checkError();                                              //エラーチェック、エラーなら true
    void getErrorMessages(string *message, vector<int>* errorCnt, vector<string>* errorStr);
                                                                    //エラーメッセージ取得、複数行
private:
    void getErrorMessage(string *message, vector<int>* errorCnt, vector<string>* errorStr,
                         int id);                                   //エラーメッセージ取得、１行
};

#endif // ROAD_H
