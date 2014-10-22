//交差点

#ifndef INTSEC_H
#define INTSEC_H

#include <Qt>
#include <map>
#include <vector>
#include <string>

#define QT_FATAL_ASSERT

using namespace std;

class Road;
class MergeIntsec;

//stl map 短縮定義
typedef map<int, Road*>                     INTSEC_ROADMAP;
typedef map<int, Road*>::iterator           INTSEC_ROADMAP_IT;
typedef multimap<double, int>               INTSEC_ANGLEMAP;
typedef multimap<double, int>::iterator     INTSEC_ANGLEMAP_IT;

//交差点
//　単路マップはマップクラスが主体、単路の生成や消去はマップクラス、それに合わせる
//　ID が接続先交差点なのは交差点だけから単路を探すため
class Intsec
{
    int             _id;                          	//交差点ID
    double          _mapX, _mapY, _mapZ;            //マップ位置
    int             _firstIdCon;                    //先頭接続交差点ID
    INTSEC_ROADMAP  _roadMap;                       //単路マップ、接続交差点IDと単路情報
    INTSEC_ANGLEMAP _angleMap;                      //角度マップ、角度と接続交差点ID、先頭から反時計回り
    MergeIntsec*    _mergeIntsec;                   //統合交差点
    bool            _border;                        //境界
    bool            _interOp;                       //補間点、DRM 用
    int             _error;                         //エラー種別

public:
    Intsec(int id);                                 //コンストラクタ
    ~Intsec()                                       //デストラクタ
    {
    }
    int id()                                        //交差点ID取得
    {
        return _id;
    }
    void setMapPos(double mx, double my, double mz) //マップ位置設定
    {
        _mapX = mx;
        _mapY = my;
        _mapZ = mz;
    }
    double mapPosX()                                //マップ位置取得
    {
        return _mapX;
    }
    double mapPosY()
    {
        return _mapY;
    }
    double mapPosZ()
    {
        return _mapZ;
    }
    void copyMapPos(Intsec* intsecSource)           //マップ位置コピー
    {
        _mapX = intsecSource->_mapX;
        _mapY = intsecSource->_mapY;
        _mapZ = intsecSource->_mapZ;
    }
    void setMergeIntsec(MergeIntsec* mergeIntsec)   //統合交差点設定／取得
    {
        _mergeIntsec = mergeIntsec;
    }
    MergeIntsec* mergeIntsec()
    {
        return _mergeIntsec;
    }
    void setBorder(bool border)                     //境界設定／取得
    {
        _border = border;
    }
    bool border()
    {
        return _border;
    }
    void setInterOp(bool interOp)                   //補間点、DRM 用設定／取得
    {
        _interOp = interOp;
    }
    bool interOp()
    {
        return _interOp;
    }

    int firstIdCon()                                //先頭接続交差点ID取得、単路なしならIDなし（NOID）
    {
        return _firstIdCon;
    }
    void setFirstIdCon(int idCon)                   //先頭接続交差点ID設定、なし設定不可
    {
        _firstIdCon = idCon;
    }
    void addRoad(int idCon, Road* road);            //単路追加
    Road* road(int idCon);                          //単路取得、なしなら NULL
    int roadNum()                                   //単路数取得
    {
        return (int)_roadMap.size();
    }
    Road* nextRoad(INTSEC_ROADMAP_IT* rmi, bool first=false, int* idCon=NULL);
                                                    //次の単路取得、first なら先頭から、なしなら NULL
    void deleteRoad(int idCon);                     //単路削除、なしなら無視
private:
    double roadAngle(int idSource, int idDest, bool absolute=false);
                                                    //単路角度取得、反時計回りまたは絶対値、Z座標は無視
public:
    Road* nextRoadByAngle(INTSEC_ANGLEMAP_IT* ami, bool first=false, int* idCon=NULL);
                                                    //角度による次の単路取得、first なら先頭から、なしなら NULL
                                                    //　先頭から反時計回り、first で角度計算後すぐに呼ぶ事
                                                    //　先頭以外で同一角度なら ID 昇順

    bool checkError();                              //エラーチェック、エラーなら true
private:
    bool check2LaneError(Road* road1, Road* road2, int intsecType1, int intsecType2);
                                                    //2 単路車線エラーチェック、エラーなら true
    bool checkStraightError(Road* road1, Road* road2, int intsecType1, int intsecType2);
                                                    //直線車線エラーチェック、エラーなら true
public:
    void getErrorMessages(string *message, vector<int>* errorCnt, vector<string>* errorStr);
                                                    //エラーメッセージ取得、複数行
private:
    void getErrorMessage(string *message, vector<int>* errorCnt, vector<string>* errorStr,
                         int id);
                                                    //エラーメッセージ取得、１行
};

#endif // INTSEC_H
