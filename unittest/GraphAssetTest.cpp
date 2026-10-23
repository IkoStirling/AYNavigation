#include "Test.h"
#include "Support.h"
#include <AYNavigation/GraphAsset.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
using namespace ayt::navigation;
namespace {
struct Directory {
    std::filesystem::path path=std::filesystem::temp_directory_path()/std::filesystem::path("aynav-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Directory(){CHECK(std::filesystem::create_directory(path));}
    ~Directory(){std::error_code ignored;std::filesystem::remove_all(path,ignored);}
};
void round_trip_preserves_all_fields_and_canonical_order() {
    auto graph=navtest::graph();graph.stamp={UINT64_MAX,UINT64_MAX};graph.nodes[0].location.region=UINT64_MAX;
    graph.links[0].traversal=9;graph.links[0].requiredCapabilities=UINT64_MAX;graph.links[0].resource=UINT64_MAX;graph.links[0].capacity=7;
    graph.links[0].enabled=false;graph.nodes[0].location.position={.12345678901234567,-0.0,1e-200};
    const auto encoded=encodeGraph(graph);CHECK(encoded);const auto decoded=decodeGraph(encoded.text);CHECK(decoded);
    CHECK(decoded.definition.stamp==graph.stamp);CHECK(decoded.definition.nodes[0].location==graph.nodes[0].location);
    const auto& link=decoded.definition.links[0];CHECK(link.traversal==9&&link.requiredCapabilities==UINT64_MAX&&link.resource==UINT64_MAX&&link.capacity==7&&!link.enabled);
    std::reverse(graph.nodes.begin(),graph.nodes.end());std::reverse(graph.links.begin(),graph.links.end());
    CHECK(encodeGraph(graph).text==encoded.text);CHECK(encodeGraph(decoded.definition).text==encoded.text);
}
void malformed_versions_counts_and_trailing_data_reject_without_definition() {
    const auto text=encodeGraph(navtest::graph()).text;
    CHECK(decodeGraph("AYNavigation.Graph 2").error==AssetError::Version);
    CHECK(decodeGraph("AYNavigation.Graph 1 space 1 1 nodes 10001").error==AssetError::Format);
    CHECK(decodeGraph("AYNavigation.Graph 1 space 1 1 nodes -1").error==AssetError::Format);
    CHECK(!decodeGraph(text+"hidden 2"));CHECK(!decodeGraph(text.substr(0,text.size()/2)));
    CHECK(decodeGraph(std::string(maxGraphAssetBytes+1,' ')).error==AssetError::TooLarge);
    auto bad=text;bad.replace(bad.find("node "),4,"oops");CHECK(decodeGraph(bad).definition.nodes.empty());
    bad=text;bad.insert(5,1,'\0');CHECK(!decodeGraph(bad));
}
void semantic_diagnostics_identify_bad_nodes_links_and_isolation() {
    auto graph=navtest::graph();graph.nodes.push_back(graph.nodes[0]);auto encoded=encodeGraph(graph);
    CHECK(encoded.error==AssetError::InvalidGraph&&encoded.text.empty());CHECK(encoded.diagnostics[0].node==graph.nodes[0].id);
    graph=navtest::graph();graph.links[0].to=999;encoded=encodeGraph(graph);
    CHECK(!encoded&&encoded.diagnostics[0].link==graph.links[0].id);
    graph=navtest::graph();graph.links[1].width=-1;encoded=encodeGraph(graph);CHECK(!encoded&&encoded.diagnostics[0].link==graph.links[1].id);
    graph=navtest::graph();graph.nodes.push_back({999,{1,{5,0,5}}});encoded=encodeGraph(graph);CHECK(encoded);
    CHECK(std::any_of(encoded.diagnostics.begin(),encoded.diagnostics.end(),[](auto& d){return d.node==999&&d.level==DiagnosticLevel::Warning;}));
    graph.nodes[0].location.position.x=std::numeric_limits<double>::infinity();CHECK(!encodeGraph(graph));
}
void native_file_create_replace_and_failures_preserve_previous_bytes() {
    Directory directory;const auto path=directory.path/std::filesystem::path(u8"路径 graph.nvg");auto graph=navtest::graph();
    CHECK(writeGraphFile(path,graph)==AssetError::None);CHECK(readGraphFile(path));const auto original=encodeGraph(readGraphFile(path).definition).text;
    ++graph.stamp.version;CHECK(writeGraphFile(path,graph)==AssetError::Exists);
    CHECK(encodeGraph(readGraphFile(path).definition).text==original);
    auto invalid=graph;invalid.links[0].cost=0;CHECK(writeGraphFile(path,invalid,true)==AssetError::InvalidGraph);
    CHECK(encodeGraph(readGraphFile(path).definition).text==original);
    CHECK(writeGraphFile(path,graph,true)==AssetError::None);CHECK(readGraphFile(path).definition.stamp==graph.stamp);
    CHECK(writeGraphFile(directory.path/"missing"/"graph.nvg",graph)==AssetError::Io);
    CHECK(readGraphFile(directory.path/"absent.nvg").error==AssetError::Io);
    std::size_t files=0;for(const auto& entry:std::filesystem::directory_iterator(directory.path)){(void)entry;++files;}CHECK(files==1);
}
}
TEST_MAIN(
    RUN(round_trip_preserves_all_fields_and_canonical_order);
    RUN(malformed_versions_counts_and_trailing_data_reject_without_definition);
    RUN(semantic_diagnostics_identify_bad_nodes_links_and_isolation);
    RUN(native_file_create_replace_and_failures_preserve_previous_bytes);
)
