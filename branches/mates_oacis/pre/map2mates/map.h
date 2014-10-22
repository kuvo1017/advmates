//マップ

#ifndef MAP_H
#define MAP_H

#include <Qt>
#include <stdio.h>
#include <vector>
#include <map>

#define QT_FATAL_ASSERT

using namespace std;

class Intsec;
class Road;

#define MAP_NOID            -1              //IDなし
#define MAP_INTSEC_ID_MIN   0               //交差点ID最小／最大（MATES制限）
#define MAP_INTSEC_ID_MAX   999999

//stl map 短縮定義
typedef map<int, Intsec*>               MAP_INTSECMAP;
typedef map<int, Intsec*>::iterator     MAP_INTSECMAP_IT;
typedef map<int, Road*>                 MAP_ROADMAP;
typedef map<int, Road*>::iterator       MAP_ROADMAP_IT;

//マップ
class Map
{
    MAP_INTSECMAP   _intsecMap;                             //交差点マップ（交差点IDと交差点情報）
    MAP_ROADMAP     _roadMap;                               //単路マップ（単路IDと単路情報）
    map<int, int>   _intsecFinalId;                         //交差点最終ID、圧縮したもの

public:
    Map();                                                  //コンストラクタ
    ~Map();                                                 //デストラクタ
    int             intsecNum()                             //交差点数取得
    {
        return ((int)_intsecMap.size());
    }
    void            deleteAll();                            //全削除

    Intsec*         createIntsec(int id=MAP_NOID);          //交差点作成取得、ID 指定なしなら最大 ID の次
                                                            //既存 ID なら NULL
    Intsec*         findIntsec(int id);                     //交差点検索、なしなら NULL
    Intsec*         nextIntsec(MAP_INTSECMAP_IT* imi, bool first=false);
                                                            //次の交差点取得、first なら先頭から、なしなら NULL
    void            deleteIntsec(int id);                   //交差点削除、接続単路も削除、なしなら無視

    Road*           createRoad(Intsec* intsec1, Intsec* intsec2, int defLane, bool* already=NULL);
                                                            //単路作成取得、既存なら既存単路を返す
    void            createRoadPackage(Intsec* intsec1, Intsec* intsec2,
                                      int laneForward, int laneBackward);
                                                            //一括道路作成処理
    Road*           findRoad(int intsecId1, int intsecId2); //単路検索、なしなら NULL
    Road*           nextRoad(MAP_ROADMAP_IT* rmi, bool first=false);
                                                            //次の単路取得、first なら先頭から、なしなら NULL
    void            deleteRoad(int roadId);                 //単路削除、なしなら無視

    void            refine();                               //整形
    void            forceOneLane();                         //強制１レーン
    void            setIntsecFinalId();                     //交差点最終ＩＤ設定
    int             getIntsecFinalId(int id);               //交差点最終ＩＤ取得
};

#endif // MAP_H
