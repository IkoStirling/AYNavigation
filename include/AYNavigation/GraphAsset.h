#pragma once
#include <AYNavigation/Graph.h>
#include <filesystem>
#include <string>
#include <string_view>

namespace ayt::navigation {
enum class AssetError {None,TooLarge,Format,Version,InvalidGraph,Io,Exists};
enum class DiagnosticLevel {Warning,Error};
struct GraphDiagnostic {
    DiagnosticLevel level=DiagnosticLevel::Error;
    std::size_t line=0;
    NodeId node=0;
    LinkId link=0;
    std::string message;
};
struct GraphAsset {
    AssetError error=AssetError::None;
    GraphDefinition definition;
    std::vector<GraphDiagnostic> diagnostics;
    explicit operator bool() const noexcept {return error==AssetError::None;}
};
struct GraphEncoding {
    AssetError error=AssetError::None;
    std::string text;
    std::vector<GraphDiagnostic> diagnostics;
    explicit operator bool() const noexcept {return error==AssetError::None;}
};
inline constexpr std::size_t maxGraphAssetBytes=16*1024*1024;
/// Version1 .nvg authoring text, bounded to16MiB/10000nodes/50000links. Full semantic validation
/// precedes success. Unknown versions/fields/trailing tokens and nonfinite values reject atomically.
/// No external paths, running tasks, persistent Entity IDs or automatic Resource loader are encoded.
GraphAsset decodeGraph(std::string_view text);
/// Canonical stable-ID ordering and round-trippable locale-independent numbers. Invalid definitions
/// produce diagnostics and no bytes. Warnings identify isolated/dead-end nodes, not proof of reachability.
GraphEncoding encodeGraph(const GraphDefinition&);
/// Explicit host-supplied path, no inferred asset root or extension-based trust. Failed reads have no
/// publish side effect; a caller only installs the returned graph after checking the result.
GraphAsset readGraphFile(const std::filesystem::path&);
/// Fully validate/encode before I/O. Create fresh by default; replace=true explicitly permits
/// atomic same-directory replacement. Failed logical validation never touches the prior file.
/// Native file publication failure is reported; no multi-file transaction or crash-durability promise.
AssetError writeGraphFile(const std::filesystem::path&,const GraphDefinition&,bool replace=false);
} // namespace ayt::navigation
