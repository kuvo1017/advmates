#ifndef __ROUTER_H__
#define __ROUTER_H__

#include <string>
#include <vector>
#include <map>
#include "ARouter.h"
#include "OD.h"

class NodeAStar;
class Route;
class Intersection;
class Section;
class Tree;
class RoadMap;
class Section;

/** @weakgroup routing */
///@{

/// 大域的経路探索を行うクラスの実装。
class Router : public ARouter
{
public:
    Router(Intersection* start = NULL, Intersection* goal = NULL, RoadMap* = NULL);
    virtual ~Router();

    /// 始点と終点の設定
    virtual void setTrip(Intersection* start, Intersection* goal, RoadMap*);

    /// 始点と終点、それから経由地の設定
    virtual void setTrip(Intersection* start, Intersection* goal, OD* od, RoadMap*);

    /// 経路選択パラメータの設定
    /**
     * このクラスではこのコンテナの要素数は6個まで考慮される。
     * 0番目は距離に関するコスト(大きいと最短経路の方が効用が高くなる)
     * 1番目は時間に関するコスト(大きいと最短時間の方が効用が高くなる)
     * 2番目は交差点での直進に関するコスト(大きいと直進が少ない方が効用が高くなる)
     * 3番目は交差点での左折に関するコスト(大きいと左折が少ない方が効用が高くなる)
     * 4番目は交差点での右折に関するコスト(大きいと右折が少ない方が効用が高くなる)
     * 5番目は道路の広さに関するコスト(大きいと道路が大きいほど効用が高くなる)
     */
    virtual void setParam(const std::vector<double>&);

    /// 探索を行い,見つかった経路を@p routesに格納する。
    /**
     *  @param start 探索を開始する交差点。
     *  @param step 処理を打ち切る上限のステップ数
     *  @param route 得られた経路を格納する。 [out]
     *  @note 探索失敗するとrouteにはNULLが与えられる。
     *  @warning 非NULLのrouteは呼び出し側でdeleteすること。
     */
    virtual bool search(const Intersection* start,
                        int step,
                        Route*& route);

    /// 上記searchのsection指定版
    virtual bool search(const Section* section,
                        const Intersection* start,
                        int step,
                        Route*& route);

    /// 最後に通過した経由地を指定する。
    /**
     * 最後に通過した経由地を指定する。
     * 指定された交差点が経由地リストになければ何もしない。
     */
    virtual void setLastPassedStopPoint(const std::string passedInter);

    virtual int counter() const;

    virtual void printParam() const;

    virtual const Intersection* start() const;

    virtual const Intersection* goal() const;

    //  virtual void drawTree() const;

#ifdef USE_ADDIN
    /// @}
    //======================================================================
    /** @name アドインに関する関数 */
    /// @{

    /// ODを返す
    const OD* od() const;
#endif

protected:
    //_a[i] >= 0
    //_a[0]:最短距離を好む度合い、_a[1]:最短時間を好む度合い,
    //_a[2-4]:直進、左折、右折の順に好む度合い,_a[5]:広い道を好む度合い
    std::vector<double> _a;

    ///全てのリンクにおける車の平均速度の最高値
    static double _vel;
  
    ///探索の対象となる道路地図
    RoadMap* _map;

    ///OD:出発地、目的地、経由地情報
    OD _od;
  
    ///出発地
    Intersection* _start;

    ///目的地
    Intersection* _goal;
  
    ///Tree構造
    Tree* _tree;
  
    ///いくつのステップ進んでいるかを示す。
    int _counter;
  
    /** @name search() から呼び出されるユーティリティ関数群 */
    /// @{
  
    /// スタートノードの作成および設定
    /**
     * @pre setParamの後に行なう必要がある。
     *
     * 指定されたstartを用いてスタートノードを作成する。
     */
    virtual void initNode(const Intersection* start,
                          const Intersection* goal);

    /// スタートノードの作成および設定
    /**
     * @pre setParamの後に行なう必要がある。
     *
     * 指定されたsectionとstartを用いてスタートノードを作成する。
     * sectionを使うことでれーんを逆走するような経路探索を
     * 防ぐことができる。
     */
    virtual void initNode(const Section* section,
                          const Intersection* start,
                          const Intersection* goal);

    /// 探索を行い,見つかった経路を@p routesに格納する。
    /**
     * startからgoalまでの経路を探索し,見つかった経路を@p routesに格納する。
     * 経由地を考慮した経路探索に用いる。
     */
    virtual void searchSegment(const Intersection *start,
                               const Intersection *goal,
                               const Intersection *past,
                               int step,
                               std::vector<Route*>& routes);

    ///ノードを展開する。
    bool expand(NodeAStar*, std::vector<NodeAStar*>*, const Intersection*);

    ///目的地ノードかどうかを判定する。
    bool isGoal(NodeAStar*, const Intersection*);

    ///A*アルゴリズムを終了させて良いか判断
    bool isEnd(const Intersection* goal);

    ///評価関数値gの算出。
    double funcG(NodeAStar*);

    ///評価関数値hの算出。
    double funcH(NodeAStar*, const Intersection*);

    ///評価関数値fの算出。
    double funcF(NodeAStar*, const Intersection*);

    /// @}
};

///@}
#endif
