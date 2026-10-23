#include "WorldLab.h"
#include "FileDialog.h"
#include <windowsx.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cwchar>
#include <fstream>
#include <set>

using namespace navlab;
namespace {
struct App {
    WorldLab lab;
    bool editing=false,dragged=false;
    NodeId selected=0,dragging=0;double height=0;
    std::filesystem::path path;
    std::chrono::steady_clock::time_point last=std::chrono::steady_clock::now();
    double accumulator=0;
};
std::wstring wide(const std::string& text) {return {text.begin(),text.end()};}
POINT point(behavior::Point2 value){return {LONG(std::lround(value.x)),LONG(std::lround(value.y))};}
void fill(HDC dc,RECT r,COLORREF color){const auto brush=CreateSolidBrush(color);FillRect(dc,&r,brush);DeleteObject(brush);}
void line(HDC dc,POINT a,POINT b,COLORREF color,int width=1,int style=PS_SOLID) {
    const auto pen=CreatePen(style,width,color);const auto old=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,old);DeleteObject(pen);
}
void circle(HDC dc,POINT p,int radius,COLORREF color,bool hollow=false) {
    const auto pen=CreatePen(PS_SOLID,2,color);const auto brush=hollow?GetStockObject(HOLLOW_BRUSH):CreateSolidBrush(color);
    const auto oldPen=SelectObject(dc,pen),oldBrush=SelectObject(dc,brush);Ellipse(dc,p.x-radius,p.y-radius,p.x+radius,p.y+radius);
    SelectObject(dc,oldPen);SelectObject(dc,oldBrush);DeleteObject(pen);if(!hollow)DeleteObject(brush);
}
void text(HDC dc,int x,int y,const std::wstring& value,COLORREF color=RGB(220,232,242)) {
    SetTextColor(dc,color);TextOutW(dc,x,y,value.c_str(),int(value.size()));
}
bool inside(const App& app,behavior::Point2 p) {
    const auto v=app.lab.cameraViewport();return p.x>=v.x&&p.x<v.x+v.width&&p.y>=v.y&&p.y<v.y+v.height;
}
NodeId pick(const App& app,behavior::Point2 pixel) {
    NodeId result=0;double distance=17;
    for(const auto& node:app.lab.definition.nodes) {
        const auto p=app.lab.screen(node.location.position);const auto d=std::hypot(pixel.x-p.x,pixel.y-p.y);
        if(d<distance){result=node.id;distance=d;}
    }
    return result;
}
Location nodeLocation(const App& app,NodeId id) {
    for(const auto& node:app.lab.definition.nodes)if(node.id==id)return node.location;
    return {};
}
void save(HWND window,App& app,bool saveAs) {
    auto path=saveAs||app.path.empty()?navexample::graphDialog(window,true):std::optional{app.path};
    if(path&&app.lab.save(*path,true))app.path=*path;
}
void load(HWND window,App& app) {
    const auto path=navexample::graphDialog(window,false);
    if(path&&app.lab.load(*path)){app.path=*path;app.selected=0;app.dragging=0;}
}
std::wstring report(const CommandReport& value) {
    if(value.navigationError!=Error::None)return L"No reachable, free goal slot";
    switch(value.behaviorError) {
        case behavior::Error::None:return value.task?L"Accepted task "+std::to_wstring(value.task):L"Stopped";
        case behavior::Error::NotInterruptible:return L"Rejected: landing first";
        case behavior::Error::PriorityTooLow:return L"Rejected: priority too low";
        default:return L"Rejected behavior error "+std::to_wstring(int(value.behaviorError));
    }
}
void paint(HWND window,HDC dc,App& app) {
    RECT r;GetClientRect(window,&r);app.lab.refreshCamera(float(r.right),float(r.bottom));
    fill(dc,r,RGB(14,23,33));SetBkMode(dc,TRANSPARENT);
    const auto font=CreateFontW(17,0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,FIXED_PITCH,L"Consolas");
    const auto oldFont=SelectObject(dc,font);const auto px=[&](Vec3 p){return point(app.lab.screen(p));};
    const auto viewport=app.lab.cameraViewport();
    const int saved=SaveDC(dc);IntersectClipRect(dc,int(viewport.x),int(viewport.y),int(viewport.width),int(viewport.y+viewport.height));
    for(int i=-12;i<=12;++i){line(dc,px({double(i),0,-8}),px({double(i),0,8}),RGB(29,43,57));line(dc,px({-12,0,double(i)}),px({12,0,double(i)}),RGB(29,43,57));}
    std::set<std::uint64_t> labelledResources;
    for(const auto& link:app.lab.definition.links) {
        const auto a=nodeLocation(app,link.from).position,b=nodeLocation(app,link.to).position;
        const auto from=px(a),to=px(b);const auto color=!link.enabled?RGB(80,80,80):link.traversal?RGB(235,180,93):RGB(71,149,132);
        if(!link.traversal&&link.enabled)line(dc,from,to,RGB(26,59,56),std::clamp(int(link.width*viewport.height/40),3,36));
        line(dc,from,to,color,link.resource?4:2,link.traversal?PS_DASH:PS_SOLID);
        const auto middle=px(a*.4+b*.6);const double angle=std::atan2(double(to.y-from.y),double(to.x-from.x));
        line(dc,middle,{LONG(middle.x-8*std::cos(angle-.5)),LONG(middle.y-8*std::sin(angle-.5))},color);
        line(dc,middle,{LONG(middle.x-8*std::cos(angle+.5)),LONG(middle.y-8*std::sin(angle+.5))},color);
        if(link.resource && labelledResources.insert(link.resource).second)text(dc,middle.x,middle.y-28,L"capacity "+std::to_wstring(link.capacity),RGB(178,178,148));
    }
    for(const auto& node:app.lab.definition.nodes) {
        const auto p=px(node.location.position);circle(dc,p,5,RGB(116,208,174));
        if(app.selected==node.id)circle(dc,p,11,RGB(230,245,235),true);
        text(dc,p.x+8,p.y-23,std::to_wstring(node.id),RGB(136,171,156));
    }
    const std::array<COLORREF,3> colors{RGB(102,181,249),RGB(226,137,161),RGB(189,159,244)};
    for(std::size_t i=0;i<app.lab.units.size();++i) {
        const auto id=behavior::unitId(app.lab.units[i]);const auto body=app.lab.motion(id);if(!body)continue;
        const auto nav=app.lab.navigation(id);
        if(nav&&app.lab.unit(id)->active)for(std::size_t s=nav->step;s<nav->route.steps.size();++s)
            line(dc,px(nav->route.steps[s].from.position),px(nav->route.steps[s].to.position),colors[i],1,PS_DOT);
        const auto p=px(body->location.position);circle(dc,p,10,colors[i]);
        if(std::binary_search(app.lab.selection.selected().begin(),app.lab.selection.selected().end(),id))circle(dc,p,16,RGB(226,242,255),true);
        text(dc,p.x-4,p.y-9,std::to_wstring(i+1),RGB(14,23,33));
    }
    if(app.lab.obstacle){circle(dc,px({-2.5,0,0}),13,RGB(229,99,94));text(dc,px({-2.5,0,0}).x-26,px({-2.5,0,0}).y+20,L"Obstacle");}
    if(const auto box=app.lab.gesture.dragBox()) {
        const auto a=point(box->from),b=point(box->to);line(dc,a,{b.x,a.y},RGB(150,205,248));line(dc,{b.x,a.y},b,RGB(150,205,248));
        line(dc,b,{a.x,b.y},RGB(150,205,248));line(dc,{a.x,b.y},a,RGB(150,205,248));
    }
    RestoreDC(dc,saved);fill(dc,{0,0,r.right,140},RGB(25,38,52));fill(dc,{0,r.bottom-200,r.right,r.bottom},RGB(25,38,52));
    text(dc,24,16,L"NAVIGATION / WORLD WORKBENCH",RGB(126,223,186));
    text(dc,24,44,L"Play: drag-select | RMB move | Shift+RMB queue | Ctrl+RMB priority interrupt | W append work | S stop");
    text(dc,24,71,L"E edit/play | F5 save as | Ctrl+S save | F9 open | F8 validate | R reset units | O obstacle");
    text(dc,24,98,app.editing?L"EDIT: click add/select, drag node, Shift-click link, Ctrl+Shift jump, Up/Down height, [ ] width, Delete node":L"PLAY: green corridors / amber jump / thick links reserve capacity; each selected unit receives a separate goal slot");
    text(dc,24,r.bottom-186,wide(app.lab.message),RGB(244,210,140));
    for(std::size_t i=0;i<app.lab.units.size();++i) {
        const auto id=behavior::unitId(app.lab.units[i]);std::wstring row=L"Unit "+std::to_wstring(i+1)+L": "+wide(app.lab.status(id));
        const auto found=std::find_if(app.lab.reports.begin(),app.lab.reports.end(),[&](auto& value){return value.unit==id;});
        if(found!=app.lab.reports.end())row+=L" | "+report(*found);
        text(dc,24,r.bottom-151+int(i)*29,row,colors[i]);
    }
    text(dc,24,r.bottom-51,L"Mode: "+std::wstring(app.editing?L"EDIT":L"PLAY")+L"   selected="+std::to_wstring(app.lab.selection.selected().size())+L"   node="+std::to_wstring(app.selected)+L"   ticks="+std::to_wstring(app.lab.ticks));
    text(dc,24,r.bottom-25,L"Save your graph with F5. Opening a graph resets units and queues. Finish a jump before opening or resetting.",RGB(135,161,184));
    SelectObject(dc,oldFont);DeleteObject(font);
}
LRESULT CALLBACK procedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam) {
    auto* app=reinterpret_cast<App*>(GetWindowLongPtrW(window,GWLP_USERDATA));
    if(message==WM_NCCREATE){app=static_cast<App*>(reinterpret_cast<CREATESTRUCTW*>(lParam)->lpCreateParams);SetWindowLongPtrW(window,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(app));}
    if(!app)return DefWindowProcW(window,message,wParam,lParam);
    const behavior::Point2 pixel{double(GET_X_LPARAM(lParam)),double(GET_Y_LPARAM(lParam))};
    switch(message) {
    case WM_GETMINMAXINFO:reinterpret_cast<MINMAXINFO*>(lParam)->ptMinTrackSize={1200,760};return 0;
    case WM_SIZE:if(LOWORD(lParam))app->lab.refreshCamera(float(LOWORD(lParam)),float(std::max(1,int(HIWORD(lParam)))));break;
    case WM_LBUTTONDOWN:
        if(!inside(*app,pixel))break;
        if(!app->editing) {
            const auto mode=(wParam&MK_CONTROL)?behavior::SelectionMode::Toggle:(wParam&MK_SHIFT)?behavior::SelectionMode::Add:behavior::SelectionMode::Replace;
            if(app->lab.gesture.begin(pixel,mode)==behavior::Error::None)SetCapture(window);
        } else {
            const auto id=pick(*app,pixel);auto candidate=app->lab.definition;
            if(id&&(wParam&MK_SHIFT)&&app->selected&&id!=app->selected) {
                LinkId next=1;for(const auto& link:candidate.links)next=std::max(next,link.id+1);
                Link link;link.id=next;link.from=app->selected;link.to=id;link.width=1.2;link.height=3;link.cost=4;
                if(wParam&MK_CONTROL){link.traversal=1;link.requiredCapabilities=1;link.resource=1000+next;link.capacity=1;}
                candidate.links.push_back(link);app->lab.edit(std::move(candidate));app->selected=id;
            } else if(id){app->selected=id;app->dragging=id;app->dragged=false;SetCapture(window);}
            else if(const auto p=app->lab.ground(pixel,app->height)) {
                NodeId next=1;for(const auto& node:candidate.nodes)next=std::max(next,node.id+1);
                candidate.nodes.push_back({next,{1,*p}});if(app->lab.edit(std::move(candidate)))app->selected=next;
            }
        }break;
    case WM_MOUSEMOVE:
        if(app->lab.gesture.active())(void)app->lab.gesture.move(pixel);
        if(app->dragging&&(wParam&MK_LBUTTON)&&inside(*app,pixel)) {
            auto candidate=app->lab.definition;const auto height=nodeLocation(*app,app->dragging).position.y;
            if(const auto p=app->lab.ground(pixel,height)){for(auto& node:candidate.nodes)if(node.id==app->dragging)node.location.position=*p;app->lab.edit(std::move(candidate));}
        }break;
    case WM_LBUTTONUP:
        if(app->lab.gesture.active())(void)app->lab.gesture.end(pixel,app->lab.candidates(),app->lab.selection);
        app->dragging=0;if(GetCapture()==window)ReleaseCapture();break;
    case WM_CAPTURECHANGED:case WM_KILLFOCUS:app->lab.gesture.cancel();app->dragging=0;app->last=std::chrono::steady_clock::now();app->accumulator=0;break;
    case WM_RBUTTONDOWN:
        if(!app->editing&&inside(*app,pixel)) {
            const auto id=pick(*app,pixel);const auto ground=app->lab.ground(pixel,app->height);
            if(id||ground)app->lab.issue(id?nodeLocation(*app,id):Location{1,*ground},(wParam&MK_CONTROL)?behavior::SubmitMode::Interrupt:(wParam&MK_SHIFT)?behavior::SubmitMode::Append:behavior::SubmitMode::Replace);
        }break;
    case WM_KEYDOWN:
        if(wParam=='E') {
            if(app->lab.idle()){app->editing=!app->editing;app->lab.gesture.cancel();app->lab.message=app->editing?"Editing graph; units remain attached.":"Play mode.";}
            else app->lab.message="Stop current work before editing.";
        } else if(wParam=='S' && (GetKeyState(VK_CONTROL)&0x8000))save(window,*app,false);
        else if(wParam==VK_F5)save(window,*app,true);
        else if(wParam==VK_F9)load(window,*app);
        else if(wParam==VK_F8) {
            const auto checked=encodeGraph(app->lab.definition);app->lab.message=checked?("Graph valid; warnings: "+std::to_string(checked.diagnostics.size())):"Graph invalid.";
            if(!checked.diagnostics.empty()) {
                const auto& first=checked.diagnostics.front();
                app->lab.message+=" Node "+std::to_string(first.node)+" link "+std::to_string(first.link)+": "+first.message;
            }
        } else if(wParam=='R')app->lab.install(app->lab.definition);
        else if(wParam=='O')app->lab.obstacle=!app->lab.obstacle;
        else if(wParam=='S')app->lab.stop();else if(wParam=='W')app->lab.wait();
        else if(app->editing&&(wParam==VK_UP||wParam==VK_DOWN)) {
            app->height=std::clamp(app->height+(wParam==VK_UP?.5:-.5),-8.0,8.0);auto candidate=app->lab.definition;
            for(auto& node:candidate.nodes)if(node.id==app->selected)node.location.position.y=app->height;
            app->lab.edit(std::move(candidate));
        } else if(app->editing&&(wParam==VK_OEM_4||wParam==VK_OEM_6)) {
            auto candidate=app->lab.definition;for(auto& link:candidate.links)if(link.from==app->selected||link.to==app->selected)link.width=std::clamp(link.width+(wParam==VK_OEM_6?.2:-.2),.5,6.0);
            app->lab.edit(std::move(candidate));
        } else if(app->editing&&wParam==VK_DELETE&&app->selected) {
            auto candidate=app->lab.definition;std::erase_if(candidate.nodes,[&](auto& node){return node.id==app->selected;});
            std::erase_if(candidate.links,[&](auto& link){return link.from==app->selected||link.to==app->selected;});
            if(app->lab.edit(std::move(candidate)))app->selected=0;
        }break;
    case WM_TIMER: {
        const auto now=std::chrono::steady_clock::now();app->accumulator+=std::min(.2,std::chrono::duration<double>(now-app->last).count());app->last=now;
        while(app->accumulator>=1.0/30.0){if(!app->editing)app->lab.tick();app->accumulator-=1.0/30.0;}
        break;
    }
    case WM_ERASEBKGND:return 1;
    case WM_PRINTCLIENT:paint(window,reinterpret_cast<HDC>(wParam),*app);return 0;
    case WM_PAINT:{PAINTSTRUCT ps;const auto dc=BeginPaint(window,&ps);RECT r;GetClientRect(window,&r);const auto memory=CreateCompatibleDC(dc);
        const auto bitmap=CreateCompatibleBitmap(dc,r.right,r.bottom);const auto old=SelectObject(memory,bitmap);paint(window,memory,*app);BitBlt(dc,0,0,r.right,r.bottom,memory,0,0,SRCCOPY);
        SelectObject(memory,old);DeleteObject(bitmap);DeleteDC(memory);EndPaint(window,&ps);return 0;}
    case WM_DESTROY:KillTimer(window,1);PostQuitMessage(0);return 0;
    default:return DefWindowProcW(window,message,wParam,lParam);
    }
    InvalidateRect(window,nullptr,FALSE);return 0;
}
bool receipt(HWND window) {
    RECT r;GetClientRect(window,&r);BITMAPINFO info{};info.bmiHeader={sizeof(BITMAPINFOHEADER),r.right,-r.bottom,1,32,BI_RGB,0,0,0,0,0};
    const auto dc=CreateCompatibleDC(nullptr);void* pixels=nullptr;const auto bitmap=CreateDIBSection(dc,&info,DIB_RGB_COLORS,&pixels,nullptr,0);
    if(!bitmap){DeleteDC(dc);return false;}const auto old=SelectObject(dc,bitmap);SendMessageW(window,WM_PRINTCLIENT,reinterpret_cast<WPARAM>(dc),0);
    const bool rendered=GetPixel(dc,10,10)==RGB(25,38,52);const DWORD bytes=DWORD(r.right*r.bottom*4);
    BITMAPFILEHEADER header{0x4d42,DWORD(sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+bytes),0,0,DWORD(sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER))};
    std::ofstream stream("AYNavigationWorldDemo.bmp",std::ios::binary|std::ios::trunc);
    stream.write(reinterpret_cast<const char*>(&header),sizeof(header));stream.write(reinterpret_cast<const char*>(&info.bmiHeader),sizeof(info.bmiHeader));stream.write(static_cast<const char*>(pixels),bytes);
    const bool saved=bool(stream);SelectObject(dc,old);DeleteObject(bitmap);DeleteDC(dc);return rendered&&saved;
}
void click(HWND window,UINT message,behavior::Point2 pixel,WPARAM modifiers=0){const auto p=point(pixel);SendMessageW(window,message,modifiers,MAKELPARAM(p.x,p.y));}
void selectAll(HWND window,App& app) {
    const auto v=app.lab.cameraViewport();click(window,WM_LBUTTONDOWN,{v.x+1,v.y+1});click(window,WM_MOUSEMOVE,{v.x+v.width-1,v.y+v.height-1},MK_LBUTTON);
    click(window,WM_LBUTTONUP,{v.x+v.width-1,v.y+v.height-1});
}
bool smoke(HWND window,App& app) {
    const auto path=std::filesystem::temp_directory_path()/("aynav-world-smoke-"+std::to_string(GetCurrentProcessId())+".nvg");
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove(path,error);}} cleanup{path};
    SendMessageW(window,WM_KEYDOWN,'E',0);const auto count=app.lab.definition.nodes.size();
    click(window,WM_LBUTTONDOWN,app.lab.screen({9,1,3}));
    if(app.lab.definition.nodes.size()!=count+1||!app.lab.save(path,false)||!app.lab.load(path))return false;
    SendMessageW(window,WM_KEYDOWN,'E',0);selectAll(window,app);if(app.lab.selection.selected().size()!=3)return false;
    click(window,WM_RBUTTONDOWN,app.lab.screen({8,1,0}));if(app.lab.reports.size()!=3)return false;
    bool jumped=false,waiting=false;
    for(int i=0;i<5000&&!app.lab.idle();++i) {
        app.lab.tick();
        for(auto handle:app.lab.units) {
            const auto id=behavior::unitId(handle);const auto phase=app.lab.navigation(id)->phase;
            waiting|=phase==Phase::WaitingForLink;
            if(phase==Phase::Traversing&&!jumped) {
                jumped=true;const auto p=app.lab.screen(app.lab.motion(id)->location.position);
                click(window,WM_LBUTTONDOWN,p);click(window,WM_LBUTTONUP,p);SendMessageW(window,WM_KEYDOWN,'S',0);
                if(app.lab.reports.empty()||app.lab.reports[0].behaviorError!=behavior::Error::NotInterruptible)return false;
                selectAll(window,app);if(!receipt(window))return false;
            }
        }
    }
    if(!jumped||!waiting||!app.lab.idle())return false;
    for(auto handle:app.lab.units)if(app.lab.navigation(behavior::unitId(handle))->phase!=Phase::Succeeded)return false;
    SendMessageW(window,WM_KEYDOWN,'R',0);selectAll(window,app);SendMessageW(window,WM_KEYDOWN,'O',0);
    click(window,WM_RBUTTONDOWN,app.lab.screen({8,1,0}));bool blocked=false;
    for(int i=0;i<1000&&!blocked;++i){app.lab.tick();for(auto handle:app.lab.units)blocked|=app.lab.navigation(behavior::unitId(handle))->phase==Phase::Blocked;}
    SendMessageW(window,WM_KEYDOWN,'S',0);return blocked&&app.lab.idle();
}
}
int WINAPI wWinMain(HINSTANCE instance,HINSTANCE,PWSTR command,int show) {
    try {
        SetProcessDPIAware();App app;WNDCLASSW type{};type.lpfnWndProc=procedure;type.hInstance=instance;type.lpszClassName=L"AliyatNavigationWorldLab";type.hCursor=LoadCursorW(nullptr,IDC_ARROW);
        if(!RegisterClassW(&type))return 1;RECT r{0,0,1280,800};AdjustWindowRect(&r,WS_OVERLAPPEDWINDOW,FALSE);
        const auto window=CreateWindowW(type.lpszClassName,L"Aliyat - Navigation World workbench",WS_OVERLAPPEDWINDOW,CW_USEDEFAULT,CW_USEDEFAULT,r.right-r.left,r.bottom-r.top,nullptr,nullptr,instance,&app);
        if(!window)return 1;
        if(std::wcsstr(command,L"--smoke")){const bool result=smoke(window,app);DestroyWindow(window);return result?0:1;}
        ShowWindow(window,show);SetTimer(window,1,16,nullptr);MSG message;
        while(GetMessageW(&message,nullptr,0,0)>0){TranslateMessage(&message);DispatchMessageW(&message);}return 0;
    }catch(const std::exception& error){OutputDebugStringA(error.what());return 1;}
}
