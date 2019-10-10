#ifndef CNOID_BASE_POSITION_EDIT_MANAGER_H
#define CNOID_BASE_POSITION_EDIT_MANAGER_H

#include <cnoid/CoordinateFrame>
#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class AbstractPositionEditTarget
{
public:
    virtual std::string getPositionName() const = 0;
    virtual CoordinateFrameId getPreferredBaseFrameId() const = 0;
    virtual CoordinateFrameId getPreferredLocalFrameId() const = 0;
    virtual Position getGlobalPosition() const = 0;
    virtual bool setPosition(
        const Position& T_global,
        const Position& T_frame, CoordinateFrame* baseFrame, CoordinateFrame* localFrame) = 0;
    virtual SignalProxy<void(const Position& T)> sigGlobalPositionChanged() = 0;
    virtual SignalProxy<void()> sigPositionEditTargetExpired() = 0;
};

class CNOID_EXPORT PositionEditManager
{
public:
    static PositionEditManager* instance();

    SignalProxy<bool(AbstractPositionEditTarget* target), LogicalSum> sigPositionEditRequest();
    bool requestPositionEdit(AbstractPositionEditTarget* target);
    AbstractPositionEditTarget* lastPositionEditTarget();

private:
    PositionEditManager();
    ~PositionEditManager();

    class Impl;
    Impl* impl;
};

}

#endif