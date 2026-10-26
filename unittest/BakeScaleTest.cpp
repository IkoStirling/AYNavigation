#include <AYNavigation/StaticBake.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace n = ayt::navigation;

int main() {
    for (int count : {1, 16, 64}) {
        n::StaticBakeWorld bake(81, {.cellSize = 1.0,
            .maxCells = 10000, .maxLinks = 50000});
        for (int index = 0; index < count; ++index) {
            n::BakeRegion region;
            region.id = static_cast<n::RegionId>(index + 1);
            region.contentVersion = 1;
            const double x = index * 10.0;
            region.floor = {{x, 0.0, 0.0}, {x + 6.0, 0.0, 6.0}};
            if (!bake.publishRegion(region))
                throw std::runtime_error("initial scale bake failed");
        }
        auto previous = bake.current();
        n::BakeRegion changed;
        changed.id = 1;
        changed.contentVersion = 2;
        changed.floor = {{0.0, 0.25, 0.0}, {6.0, 0.25, 6.0}};
        const auto begin = std::chrono::steady_clock::now();
        if (!bake.publishRegion(changed))
            throw std::runtime_error("changed scale bake failed");
        const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin).count();
        const auto report = bake.report();
        if (report.regions != static_cast<std::size_t>(count)
            || report.geometryRebakes != 1 || !previous || !bake.current()
            || previous->stamp().version == bake.current()->stamp().version)
            throw std::runtime_error("scale revision contract failed");
        const auto& graph = bake.current()->definition();
        const auto candidateBytes = graph.nodes.capacity() * sizeof(n::Node)
            + graph.links.capacity() * sizeof(n::Link);
        std::cout << "regions=" << count << " nodes=" << report.nodes
            << " links=" << report.links << " changedFragments="
            << report.geometryRebakes << " replaceMicroseconds=" << micros
            << " candidateVectorBytesAtLeast=" << candidateBytes << '\n';
    }
    return 0;
}
