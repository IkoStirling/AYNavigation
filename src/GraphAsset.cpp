#include <AYNavigation/GraphAsset.h>
#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace ayt::navigation {
namespace {
struct Tokens {
    std::string_view text;std::size_t offset=0,line=1;
    std::string_view next() {
        while(offset<text.size() && (text[offset]==' '||text[offset]=='\t'||text[offset]=='\r'||text[offset]=='\n')) {
            if(text[offset]=='\n')++line;++offset;
        }
        const auto start=offset;
        while(offset<text.size() && text[offset]!=' '&&text[offset]!='\t'&&text[offset]!='\r'&&text[offset]!='\n')++offset;
        return text.substr(start,offset-start);
    }
    bool word(std::string_view expected){return next()==expected;}
    template<class T> bool number(T& value) {
        const auto token=next();if(token.empty())return false;
        const auto result=std::from_chars(token.data(),token.data()+token.size(),value);
        return result.ec==std::errc{} && result.ptr==token.data()+token.size();
    }
};
GraphAsset failed(AssetError error,std::string message,std::size_t line=0) {
    GraphAsset result;result.error=error;result.diagnostics.push_back({DiagnosticLevel::Error,line,0,0,std::move(message)});return result;
}
std::vector<GraphDiagnostic> validate(const GraphDefinition& definition) {
    std::vector<GraphDiagnostic> diagnostics;
    const auto compiled=GraphSpace::compile(definition);
    if(!compiled) {
        if(!definition.stamp.space||!definition.stamp.version)diagnostics.push_back({DiagnosticLevel::Error,0,0,0,"Space and revision must be nonzero"});
        if(definition.nodes.empty()||definition.nodes.size()>10000||definition.links.size()>50000) {
            diagnostics.push_back({DiagnosticLevel::Error,0,0,0,"Graph requires1..10000 nodes and at most50000 links"});return diagnostics;
        }
        std::set<NodeId> nodes;std::set<LinkId> links;
        std::map<NodeId,Location> positions;std::map<std::uint64_t,std::uint32_t> capacities;
        for(const auto& n:definition.nodes)if(!n.id || !nodes.insert(n.id).second || !n.location.region || !finite(n.location.position))
            diagnostics.push_back({DiagnosticLevel::Error,0,n.id,0,"Invalid or duplicate node identity/location"});
        for(const auto& n:definition.nodes)positions.emplace(n.id,n.location);
        for(const auto& l:definition.links) {
            if(!l.id || !links.insert(l.id).second || !nodes.contains(l.from) || !nodes.contains(l.to))
                diagnostics.push_back({DiagnosticLevel::Error,0,0,l.id,"Invalid/duplicate link identity or missing endpoint"});
            if(!std::isfinite(l.width)||!std::isfinite(l.height)||l.width<=0||l.width>1e9||l.height<=0||l.height>1e9||
                !l.cost||l.cost>1000000000000ULL||bool(l.resource)!=bool(l.capacity))
                diagnostics.push_back({DiagnosticLevel::Error,0,0,l.id,"Invalid link width/height/cost or resource-capacity pair"});
            if(l.from==l.to || (!l.traversal&&positions.contains(l.from)&&positions.contains(l.to)&&
                (positions.at(l.from).region!=positions.at(l.to).region||length(positions.at(l.from).position-positions.at(l.to).position)<=1e-7)))
                diagnostics.push_back({DiagnosticLevel::Error,0,0,l.id,"Invalid link endpoints: ordinary links need distinct positions in one region"});
            if(l.resource) {
                const auto [entry,inserted]=capacities.emplace(l.resource,l.capacity);
                if(!inserted&&entry->second!=l.capacity)diagnostics.push_back({DiagnosticLevel::Error,0,0,l.id,"Shared resource has conflicting capacities"});
            }
        }
        if(diagnostics.empty())diagnostics.push_back({DiagnosticLevel::Error,0,0,0,"Graph rejected: check stamp, budgets, positive costs, dimensions, ordinary-link regions and shared capacities"});
        return diagnostics;
    }
    std::map<NodeId,std::pair<unsigned,unsigned>> degree;
    for(const auto& n:definition.nodes)degree[n.id]={};
    for(const auto& l:definition.links)if(l.enabled){++degree[l.from].second;++degree[l.to].first;}
    for(const auto& [id,d]:degree) {
        if(!d.first&&!d.second)diagnostics.push_back({DiagnosticLevel::Warning,0,id,0,"Isolated node: no enabled links"});
        else if(!d.second)diagnostics.push_back({DiagnosticLevel::Warning,0,id,0,"Directed dead end: no enabled outgoing link"});
    }
    return diagnostics;
}
bool hasError(const std::vector<GraphDiagnostic>& diagnostics) {
    return std::any_of(diagnostics.begin(),diagnostics.end(),[](const auto& d){return d.level==DiagnosticLevel::Error;});
}
template<class T> void append(std::string& text,T value) {
    char buffer[128];const auto result=std::to_chars(buffer,buffer+sizeof(buffer),value);
    if(result.ec!=std::errc{})throw std::runtime_error("Numeric encoding failed");
    text.append(buffer,result.ptr);text+=' ';
}
void location(std::string& text,const Location& value) {
    append(text,value.region);append(text,value.position.x);append(text,value.position.y);append(text,value.position.z);
}
void newline(std::string& text){if(!text.empty()&&text.back()==' ')text.pop_back();text+='\n';}
AssetError publish(const std::filesystem::path& destination,std::string_view text,bool replace) {
    static std::atomic<unsigned long long> sequence=0;
    std::filesystem::path temporary;
#ifdef _WIN32
    HANDLE file=INVALID_HANDLE_VALUE;
    for(int attempt=0;attempt<32 && file==INVALID_HANDLE_VALUE;++attempt) {
        temporary=destination;temporary+=L".tmp."+std::to_wstring(GetCurrentProcessId())+L"."+std::to_wstring(++sequence);
        file=CreateFileW(temporary.c_str(),GENERIC_WRITE,0,nullptr,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(file==INVALID_HANDLE_VALUE && GetLastError()!=ERROR_FILE_EXISTS)return AssetError::Io;
    }
    if(file==INVALID_HANDLE_VALUE)return AssetError::Io;
    DWORD written=0;const bool saved=WriteFile(file,text.data(),DWORD(text.size()),&written,nullptr) && written==text.size() && FlushFileBuffers(file);
    const bool closed=CloseHandle(file)!=0;
    if(!saved||!closed){DeleteFileW(temporary.c_str());return AssetError::Io;}
    if(MoveFileExW(temporary.c_str(),destination.c_str(),replace?MOVEFILE_REPLACE_EXISTING:0))return AssetError::None;
    const auto error=GetLastError();DeleteFileW(temporary.c_str());
    return error==ERROR_ALREADY_EXISTS||error==ERROR_FILE_EXISTS?AssetError::Exists:AssetError::Io;
#else
    int file=-1;
    for(int attempt=0;attempt<32&&file<0;++attempt) {
        temporary=destination;temporary+=".tmp."+std::to_string(getpid())+"."+std::to_string(++sequence);
        file=::open(temporary.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
        if(file<0&&errno!=EEXIST)return AssetError::Io;
    }
    if(file<0)return AssetError::Io;
    std::size_t offset=0;bool saved=true;
    while(offset<text.size()) {const auto count=::write(file,text.data()+offset,text.size()-offset);if(count<=0){saved=false;break;}offset+=std::size_t(count);}
    saved=(::fsync(file)==0)&&saved;saved=(::close(file)==0)&&saved;
    if(!saved){::unlink(temporary.c_str());return AssetError::Io;}
    const int result=replace?::rename(temporary.c_str(),destination.c_str()) : ::link(temporary.c_str(),destination.c_str());
    const int error=errno;if(!replace||result!=0)::unlink(temporary.c_str());
    return result==0?AssetError::None:error==EEXIST?AssetError::Exists:AssetError::Io;
#endif
}
}
GraphAsset decodeGraph(std::string_view text) {
    if(text.size()>maxGraphAssetBytes)return failed(AssetError::TooLarge,"Graph file exceeds16MiB");
    Tokens input{text};unsigned version=0;GraphAsset result;
    if(!input.word("AYNavigation.Graph")||!input.number(version))return failed(AssetError::Format,"Expected AYNavigation.Graph version",input.line);
    if(version!=1)return failed(AssetError::Version,"Unsupported graph version",input.line);
    auto& definition=result.definition;std::size_t count=0;
    if(!input.word("space")||!input.number(definition.stamp.space)||!input.number(definition.stamp.version)||
       !input.word("nodes")||!input.number(count)||count>10000)return failed(AssetError::Format,"Invalid space/nodes header",input.line);
    definition.nodes.resize(count);
    for(auto& node:definition.nodes)if(!input.word("node")||!input.number(node.id)||!input.number(node.location.region)||
        !input.number(node.location.position.x)||!input.number(node.location.position.y)||!input.number(node.location.position.z))
            return failed(AssetError::Format,"Invalid node record",input.line);
    if(!input.word("links")||!input.number(count)||count>50000)return failed(AssetError::Format,"Invalid links header",input.line);
    definition.links.resize(count);
    for(auto& link:definition.links) {
        unsigned enabled=0;
        if(!input.word("link")||!input.number(link.id)||!input.number(link.from)||!input.number(link.to)||
            !input.number(link.width)||!input.number(link.height)||!input.number(link.cost)||!input.number(link.requiredCapabilities)||
            !input.number(link.resource)||!input.number(link.capacity)||!input.number(link.traversal)||!input.number(enabled)||enabled>1)
                return failed(AssetError::Format,"Invalid link record",input.line);
        link.enabled=enabled!=0;
    }
    if(!input.word("end")||!input.next().empty())return failed(AssetError::Format,"Expected end without trailing fields",input.line);
    result.diagnostics=validate(definition);
    if(hasError(result.diagnostics)){result.error=AssetError::InvalidGraph;result.definition={};}
    return result;
}
GraphEncoding encodeGraph(const GraphDefinition& definition) {
    GraphEncoding result;result.diagnostics=validate(definition);
    if(hasError(result.diagnostics)){result.error=AssetError::InvalidGraph;return result;}
    auto sorted=definition;std::sort(sorted.nodes.begin(),sorted.nodes.end(),[](auto& a,auto& b){return a.id<b.id;});
    std::sort(sorted.links.begin(),sorted.links.end(),[](auto& a,auto& b){return a.id<b.id;});
    auto& text=result.text;text="AYNavigation.Graph 1\nspace ";append(text,sorted.stamp.space);append(text,sorted.stamp.version);
    newline(text);text+="nodes ";append(text,sorted.nodes.size());newline(text);
    for(const auto& node:sorted.nodes){text+="node ";append(text,node.id);location(text,node.location);newline(text);}
    text+="links ";append(text,sorted.links.size());newline(text);
    for(const auto& link:sorted.links) {
        text+="link ";append(text,link.id);append(text,link.from);append(text,link.to);append(text,link.width);append(text,link.height);
        append(text,link.cost);append(text,link.requiredCapabilities);append(text,link.resource);append(text,link.capacity);
        append(text,link.traversal);append(text,unsigned(link.enabled));newline(text);
    }
    text+="end\n";
    if(text.size()>maxGraphAssetBytes){result.error=AssetError::TooLarge;result.text.clear();}
    return result;
}
GraphAsset readGraphFile(const std::filesystem::path& path) {
    std::ifstream stream(path,std::ios::binary|std::ios::ate);
    if(!stream)return failed(AssetError::Io,"Cannot open graph file");
    const auto size=stream.tellg();if(size<0)return failed(AssetError::Io,"Cannot read graph file size");
    if(size>std::streamoff(maxGraphAssetBytes))return failed(AssetError::TooLarge,"Graph file exceeds16MiB");
    std::string text(std::size_t(size),'\0');stream.seekg(0);
    if(!stream.read(text.data(),std::streamsize(text.size())))return failed(AssetError::Io,"Cannot read complete graph file");
    // Refuse growth during read rather than silently loading a truncated prefix.
    if(stream.peek()!=std::char_traits<char>::eof())return failed(AssetError::Io,"Graph file changed while reading");
    return decodeGraph(text);
}
AssetError writeGraphFile(const std::filesystem::path& path,const GraphDefinition& definition,bool replace) {
    const auto encoded=encodeGraph(definition);if(!encoded)return encoded.error;
    return publish(path,encoded.text,replace);
}
} // namespace ayt::navigation
