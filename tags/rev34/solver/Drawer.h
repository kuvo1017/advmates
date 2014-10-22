#ifndef __DRAWER_H__
#define __DRAWER_H__

class RoadEntity;
class Lane;
class Signal;
class Intersection;
class Section;
class RoadMap;

class DetectorUnit;
class Vehicle;

//######################################################################
/// RoadEntity（サブセクション）を描画するクラス
class RoadEntityDrawer
{
public:
    static RoadEntityDrawer& instance();
    virtual void draw(const RoadEntity& entity,
                      bool isSubsectionShape,
                      bool isSubnetwork,
                      bool isSubsectionId,
                      int surfaceMode,
                      bool isSignals) const;
protected:
    RoadEntityDrawer(){};
    RoadEntityDrawer(const RoadEntityDrawer&){};
    virtual ~RoadEntityDrawer(){};

    /// サブセクションの形状を描画する
    virtual void drawShape(const RoadEntity& entity,
                           int surfaceMode) const;

    /// サブネットワークを表示する
    virtual void drawSubnetwork(const RoadEntity& entity,
                                double height,
                                double width) const;

    /// 信号を描画する
    virtual void drawSignals(const RoadEntity& entity) const;

    /// IDを表示する
    virtual void drawId(const RoadEntity& entity) const;
};

//######################################################################
/// Laneを描画するクラス
class LaneDrawer
{
public:
    static LaneDrawer& instance();
    virtual void draw(const Lane& lane, double height, double width) const;
    virtual void drawId(const Lane& lane) const;
protected:
    LaneDrawer(){};
    LaneDrawer(const LaneDrawer&){};
    virtual ~LaneDrawer(){};
};

//######################################################################
/// Signalを描画するクラス
class SignalDrawer
{
public:
    static SignalDrawer& instance();
    virtual void draw(const Intersection& inter) const;
protected:
    SignalDrawer(){};
    SignalDrawer(const SignalDrawer&){};
    virtual ~SignalDrawer(){};
};

//######################################################################
/// Intersectionを描画するクラス
class IntersectionDrawer
{
public:
    static IntersectionDrawer& instance();
    virtual void draw(const Intersection& inter,
                      bool isSectionId,
                      bool isLanes,
                      bool isLaneId,
                      bool isSubsectionShape,
                      bool isSubnetwork,
                      bool isSubsectionId,
                      int surfaceMode,
                      bool isSignals,
                      int connectorIdMode) const;
    virtual void drawSimple(const Intersection& inter,
                            double radius,
                            bool isSectionId) const;

protected:
    IntersectionDrawer(){};
    IntersectionDrawer(const IntersectionDrawer&){};
    virtual ~IntersectionDrawer(){};

    /// サブセクションを描画する
    virtual void drawSubsections(const Intersection& inter,
                                 bool isSubsectionShape,
                                 bool isSubnetwork,
                                 bool isSubsectionId,
                                 int surfaceMode,
                                 bool isSignals) const;
    /// 内部レーンを描画する
    virtual void drawLanes(const Intersection& inter,
                           bool isLaneId) const;

    /// コネクタを描画する
    virtual void drawConnectors(const Intersection& inter,
                                int connectorIdMode) const;

    /// 信号を描画する
    virtual void drawSignals(const Intersection& inter) const;
    /// IDを表示する
    virtual void drawId(const Intersection& inter) const;
};

//######################################################################
/// Sectionを描画するクラス
class SectionDrawer
{
public:
    static SectionDrawer& instance();
    virtual void draw(const Section& section,
                      bool isSectionId,
                      bool isLanes,
                      bool isLaneId,
                      bool isSubsectionShape,
                      bool isSubnetwork,
                      bool isSubsectionId,
                      int surfaceMode,
                      bool isSignals,
                      int connectorIdMode) const;
    virtual void drawSimple(const Section& section,
                            double width) const;
protected:
    SectionDrawer(){};
    SectionDrawer(const SectionDrawer&){};
    virtual ~SectionDrawer(){};

    /// サブセクションを描画する
    virtual void drawSubsections(const Section& section,
                                 bool isSubsectionShape,
                                 bool isSubnetwork,
                                 bool isSubsectionId,
                                 int surfaceMode,
                                 bool isSignals) const;
    /// 内部レーンを描画する
    virtual void drawLanes(const Section& section,
                           bool isLaneId) const;

    /// コネクタを描画する
    virtual void drawConnectors(const Section& section,
                                int connectorIdMode) const;
};

//######################################################################
/// RoadMapを描画するクラス
class RoadMapDrawer
{
public:
    static RoadMapDrawer& instance();
    virtual void draw(const RoadMap&,
                      bool isSimpleMap,
                      bool isSectionId,
                      bool isLanesInter,
                      bool isLanesSection,
                      bool isLaneId,
                      bool isSubsectionShape,
                      bool isSubnetwork,
                      bool isSubsectonId,
                      int surfaceMode,
                      bool isSignals,
                      int connectorIdMode) const;

protected:
    RoadMapDrawer();
    RoadMapDrawer(const RoadMapDrawer&){};
    virtual ~RoadMapDrawer(){};

    const IntersectionDrawer* _intersectionDrawer;
    const SectionDrawer* _sectionDrawer;
};

//######################################################################
/// Detectorを描画するクラス
class DetectorDrawer
{
public:
    static DetectorDrawer& instance();
    virtual void draw(const DetectorUnit& unit) const;

protected:
    DetectorDrawer(){};
    DetectorDrawer(const DetectorDrawer&){};
    virtual ~DetectorDrawer(){};
};

//######################################################################
/// Vehicleを描画するクラス
class VehicleDrawer
{
public:
    static VehicleDrawer& instance();
    virtual void draw(const Vehicle& vehicle,
                      bool isVehicleId) const;
    virtual void drawSimple(const Vehicle& vehicle,
                            bool isVehicleId,
                            double size) const;
protected:
    VehicleDrawer(){};
    VehicleDrawer(const VehicleDrawer&){};
    virtual ~VehicleDrawer(){};
};

#endif //__DRAWER_H__
