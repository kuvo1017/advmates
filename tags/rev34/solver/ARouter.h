#ifndef __AROUTER_H__
#define __AROUTER_H__

#include <string>
#include <vector>

class RoadMap;
class Intersection;
class Route;
class Section;
class OD;

/// 大域的経路探索を行うクラスの抽象基底クラス。
/**
 * @sa Route
 * @ingroup routing
 */
class ARouter
{
public:
    ARouter(Intersection*, Intersection*, RoadMap*) {};
    virtual ~ARouter(){};

    /// 始点@p start、終点 @p goal、全体の道路ネットワーク@p roadMap を設定する。
    virtual void setTrip(Intersection* start,
                         Intersection* goal,
                         RoadMap* roadMap) = 0;  

    /// 始点@p start、終点 @p goal、OD@p od、全体の道路ネットワーク@p roadMap を設定する。
    virtual void setTrip(Intersection* start,
                         Intersection* goal,
                         OD* od,
                         RoadMap* roadMap) = 0;  

    /// 選好を設定する。
    /** 重み付けに用いられるのでそれぞれの数値はその間の比率のみ効果をなす。
     *  それぞれの数値の意味はサブクラスを参照。*/
    virtual void setParam(const std::vector<double>&) = 0;

    /// 探索を行ない,見つかった複数の経路を返す。
    /**
     *  @param start 探索を開始する交差点。
     *  @param step 処理を打ち切る上限のステップ数
     *  @param route 得られた経路を格納する。 [out]
     *  @note 探索失敗するとrouteにはNULLが与えられる。
     *  @warning 非NULLのrouteは呼び出し側でdeleteすること。
     */
    virtual bool search(const Intersection* start,
                        int step,
                        Route*& route) = 0;

    /// searchの_section指定版
    virtual bool search(const Section* section,
                        const Intersection* start,
                        int step,
                        Route*& route) = 0;

    /// 最後に通過した経由地を指定する。
    /**
     * 最後に通過した経由地を指定する。
     * 指定された交差点が経由地リストになければ何もしない。
     */
    virtual void setLastPassedStopPoint(const std::string passedInter) = 0;
  
    //@}
  
    /// 経路探索の現在のステップ数
    virtual int counter() const = 0;

    /// このクラスに設定された情報を返す
    virtual void printParam() const = 0;

    //  virtual void drawTree() const = 0;

    virtual const Intersection* start() const = 0;

    virtual const Intersection* goal() const = 0;

};
#endif
