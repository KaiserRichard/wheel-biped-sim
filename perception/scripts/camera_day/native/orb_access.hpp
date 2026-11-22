// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <System.h>

// Compatibility with the installed ORB build: Shutdown() only requests finish,
// and System exposes neither thread completion nor atlas IMU initialization.
// Explicit template instantiation may name private members (C++ [temp.explicit]).
// This obtains typed member pointers without changing class layout, upstream
// headers, the library, or using a preprocessor private/public substitution.
// Keep this small and recheck against the recorded System.cc/header hashes.
namespace wbr {
template<class Tag, typename Tag::type Member> struct MemberAccess {
    friend typename Tag::type member(Tag) { return Member; }
};
struct MapperMember {
    using type = ORB_SLAM3::LocalMapping* ORB_SLAM3::System::*;
    friend type member(MapperMember);
};
struct CloserMember {
    using type = ORB_SLAM3::LoopClosing* ORB_SLAM3::System::*;
    friend type member(CloserMember);
};
struct AtlasMember {
    using type = ORB_SLAM3::Atlas* ORB_SLAM3::System::*;
    friend type member(AtlasMember);
};
template struct MemberAccess<MapperMember, &ORB_SLAM3::System::mpLocalMapper>;
template struct MemberAccess<CloserMember, &ORB_SLAM3::System::mpLoopCloser>;
template struct MemberAccess<AtlasMember, &ORB_SLAM3::System::mpAtlas>;
inline ORB_SLAM3::Atlas* atlas(ORB_SLAM3::System& slam) { return slam.*member(AtlasMember{}); }
inline bool workers_finished(ORB_SLAM3::System& slam) {
    auto mapper = slam.*member(MapperMember{});
    auto closer = slam.*member(CloserMember{});
    return mapper->isFinished() && closer->isFinished() && !closer->isRunningGBA();
}
} // namespace wbr
