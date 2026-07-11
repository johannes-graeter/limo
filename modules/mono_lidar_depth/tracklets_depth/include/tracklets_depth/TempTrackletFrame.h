#pragma once
#include <utility>

namespace tracklets_depth{
struct TempTrackletFrame {
    int _keyFrameId;
    std::pair<int, int> _feature;

private:
    TempTrackletFrame() : _keyFrameId(0), _feature(std::make_pair(0, 0)) {
    }

public:
    TempTrackletFrame(const int id, const std::pair<int, int>& feature) : _keyFrameId(id), _feature(feature) {
    }

    virtual ~TempTrackletFrame() = default;
};

struct TempTrackletFrameExp : public TempTrackletFrame {
    std::pair<int, int> _featureLast;

private:
    TempTrackletFrameExp() : TempTrackletFrame(0, std::make_pair(0, 0)), _featureLast(std::make_pair(0, 0)) {
    }

public:
    TempTrackletFrameExp(const int id, const std::pair<int, int>& featureNew, const std::pair<int, int>& featureLast)
            : TempTrackletFrame(id, featureNew), _featureLast(featureLast) {
    }
};
}
