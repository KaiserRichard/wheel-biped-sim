// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "wbr_orb_node/map_points.hpp"
#include "orb_access.hpp"  // Existing native compatibility helper; no ORB ABI change.
#include <algorithm>
#include <mutex>
#include <set>

namespace wbr {
// Upstream has no bounded map accessor. Confine the two extra typed member
// pointers here; re-audit when upgrading ORB (see MAP_POINTS.md).
struct MapMutexMember {
    using type = std::mutex ORB_SLAM3::Map::*;
    friend type member(MapMutexMember);
};
struct MapPointsMember {
    using type = std::set<ORB_SLAM3::MapPoint*> ORB_SLAM3::Map::*;
    friend type member(MapPointsMember);
};
template struct MemberAccess<MapMutexMember, &ORB_SLAM3::Map::mMutexMap>;
template struct MemberAccess<MapPointsMember, &ORB_SLAM3::Map::mspMapPoints>;
}
namespace wbr::orb_node {
// Call only between Track calls on the synchronous System owner. Tracking owns
// reset/clear; ORB mapper/closer still run concurrently. Never retain raw pointers.
// At the audited ORB pin, Atlas::RemoveBadMaps does not delete Map allocations,
// so the selected Map remains alive if a background merge changes the active map.
// Do not re-enter Atlas after locking mMutexMapUpdate: that reverses lock order.
inline bool snapshot_active_map(ORB_SLAM3::System& system, size_t limit, MapSnapshot& out) {
    if (limit < 1 || limit > 200000) throw std::invalid_argument("invalid map point limit");
    auto* atlas = wbr::atlas(system);
    auto* map = atlas->GetCurrentMap();
    if (!map) return false;
    std::unique_lock<std::mutex> update(map->mMutexMapUpdate, std::try_to_lock);
    if (!update.owns_lock()) return false;
    MapSnapshot snapshot;
    snapshot.map_id = map->GetId();
    snapshot.init_kf_id = map->GetInitKFid();
    snapshot.big_change = map->GetLastBigChangeIdx();
    snapshot.map_change = map->GetMapChangeIndex();
    std::vector<ORB_SLAM3::MapPoint*> candidates;
    {
        std::lock_guard<std::mutex> container(map->*member(MapMutexMember{}));
        const auto& points = map->*member(MapPointsMember{});
        snapshot.candidates = points.size();
        snapshot.truncated = points.size() > limit;
        candidates.reserve(std::min(points.size(), limit));
        for (auto* point : points) {
            if (candidates.size() == limit) break;
            candidates.push_back(point);
        }
    }
    // Culling erases pointers but does not delete points at the audited pins.
    // Per-point getters lock their own state; update lock excludes bulk correction
    // and map migration. Newly inserted points can appear in the next snapshot.
    snapshot.points.reserve(candidates.size());
    for (auto* point : candidates) {
        if (!point || point->isBad() || point->GetMap() != map) continue;
        const auto p = point->GetWorldPos();
        const MapXYZ xyz{p.x(), p.y(), p.z()};
        if (finite_point(xyz) && !point->isBad()) snapshot.points.push_back(xyz);
    }
    out = std::move(snapshot);
    return true;  // update lock released before caller serializes/publishes.
}
}  // namespace wbr::orb_node
