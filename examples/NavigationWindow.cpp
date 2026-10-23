// Interactive in-memory path authoring and navigation; same Core plugins as automated fixtures.
#include "ExampleWorld.h"
#include <windows.h>
#include <windowsx.h>
#include <cwchar>
#include <cmath>
#ifdef AY_NAVIGATION_AUTHORING
#include "FileDialog.h"
#endif
using namespace ayt::navigation;
namespace {
constexpr int top=140,bottom=84;constexpr double scale=38;
struct App {
    navexample::World world;NodeId dragging=0;Vec3 preview;bool moved=false;
};
POINT screen(HWND window,Vec3 p) {
    RECT r;GetClientRect(window,&r);
    return {LONG(r.right*.5+(p.x-p.z)*scale),LONG((top+r.bottom-bottom)*.5+(p.x+p.z)*scale*.35-p.y*scale)};
}
Vec3 location(HWND window,POINT pixel,double height) {
    RECT r;GetClientRect(window,&r);const double a=(pixel.x-r.right*.5)/scale;
    const double b=(pixel.y-(top+r.bottom-bottom)*.5+height*scale)/(scale*.35);
    return {(a+b)*.5,height,(b-a)*.5};
}
bool scene(HWND window,POINT p) {RECT r;GetClientRect(window,&r);return p.x>=0 && p.x<r.right && p.y>=top && p.y<r.bottom-bottom;}
NodeId pick(HWND window,const App& app,POINT point) {
    NodeId best=0;double distance=15;
    for(const auto& node:app.world.definition.nodes) {
        const auto pixel=screen(window,node.location.position);const auto d=std::hypot(double(pixel.x-point.x),double(pixel.y-point.y));
        if(d<distance){distance=d;best=node.id;}
    }
    return best;
}
Vec3 position(const App& app,NodeId id) {
    if(app.dragging==id && app.moved) return app.preview;
    for(const auto& node:app.world.definition.nodes) if(node.id==id) return node.location.position;
    return {};
}
void fill(HDC dc,RECT r,COLORREF color){const auto brush=CreateSolidBrush(color);FillRect(dc,&r,brush);DeleteObject(brush);}
void text(HDC dc,int x,int y,const std::wstring& value,COLORREF color=RGB(222,233,246)) {
    SetTextColor(dc,color);TextOutW(dc,x,y,value.c_str(),int(value.size()));
}
void line(HDC dc,POINT a,POINT b,COLORREF color,int width=1,int style=PS_SOLID) {
    const auto pen=CreatePen(style,width,color);const auto prior=SelectObject(dc,pen);
    MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,prior);DeleteObject(pen);
}
void circle(HDC dc,POINT p,int radius,COLORREF color,bool hollow=false) {
    const auto pen=CreatePen(PS_SOLID,2,color);const auto brush=hollow?GetStockObject(HOLLOW_BRUSH):CreateSolidBrush(color);
    const auto oldPen=SelectObject(dc,pen),oldBrush=SelectObject(dc,brush);
    Ellipse(dc,p.x-radius,p.y-radius,p.x+radius,p.y+radius);
    SelectObject(dc,oldBrush);SelectObject(dc,oldPen);DeleteObject(pen);if(!hollow)DeleteObject(brush);
}
std::wstring phase(Phase p) {
    switch(p){case Phase::Planning:return L"Planning";case Phase::Following:return L"Moving";case Phase::WaitingForLink:return L"Waiting for access";
        case Phase::Traversing:return L"Jumping";case Phase::Blocked:return L"Waiting for clearance";case Phase::Succeeded:return L"Arrived";
        case Phase::Failed:return L"Unable to complete this route";default:return L"Ready";}
}
void paint(HWND window,HDC dc,const App& app) {
    RECT r;GetClientRect(window,&r);fill(dc,r,RGB(15,24,35));SetBkMode(dc,TRANSPARENT);
    const auto font=CreateFontW(-18,0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
    const auto oldFont=SelectObject(dc,font);
    for(int x=-12;x<=12;++x) line(dc,screen(window,{double(x),0,-12}),screen(window,{double(x),0,12}),RGB(25,39,52));
    for(int z=-12;z<=12;++z) line(dc,screen(window,{-12,0,double(z)}),screen(window,{12,0,double(z)}),RGB(25,39,52));
    if(app.world.mode==1) {
        for(const auto& edge:app.world.definition.links) {
            const auto from=position(app,edge.from),to=position(app,edge.to);
            const auto a=screen(window,from),b=screen(window,to);
            if(!edge.traversal) {
                line(dc,a,b,RGB(29,66,64),std::max(2,int(edge.width*scale*.65)));
                line(dc,a,b,RGB(99,218,178),2);
            } else {
                auto prior=a;
                for(int i=1;i<=24;++i) {const double t=double(i)/24;const auto p=from+(to-from)*t+Vec3{0,8*t*(1-t),0};
                    const auto next=screen(window,p);line(dc,prior,next,RGB(241,185,94),2);prior=next;}
            }
            const POINT arrow{LONG(a.x*.35+b.x*.65),LONG(a.y*.35+b.y*.65)};
            const double angle=std::atan2(double(b.y-a.y),double(b.x-a.x));
            line(dc,arrow,{LONG(arrow.x-8*std::cos(angle-.5)),LONG(arrow.y-8*std::sin(angle-.5))},RGB(167,207,194));
            line(dc,arrow,{LONG(arrow.x-8*std::cos(angle+.5)),LONG(arrow.y-8*std::sin(angle+.5))},RGB(167,207,194));
        }
        for(const auto& node:app.world.definition.nodes) {
            const auto p=position(app,node.id);const auto pixel=screen(window,p);
            circle(dc,pixel,6,RGB(112,214,179));if(app.world.selected==node.id)circle(dc,pixel,12,RGB(228,250,239),true);
            wchar_t label[80];swprintf_s(label,L"%llu / h%.1f",static_cast<unsigned long long>(node.id),p.y);
            text(dc,pixel.x+12,pixel.y-12,label,RGB(171,201,190));
        }
    } else if(const auto state=app.world.navigator->state(1);state && !state->route.steps.empty()) {
        for(const auto& step:state->route.steps) line(dc,screen(window,step.from.position),screen(window,step.to.position),RGB(80,147,210),2,PS_DOT);
        circle(dc,screen(window,state->route.goal.position),8,RGB(118,192,249),true);
    }
    if(app.world.obstacles.enabled) {
        const auto p=screen(window,{-4,0,0});circle(dc,p,17,RGB(207,99,115));text(dc,p.x-35,p.y+24,L"Obstacle",RGB(231,151,166));
    }
    const auto body=app.world.motor.body;const auto actor=screen(window,body.location.position);
    line(dc,screen(window,{body.location.position.x,0,body.location.position.z}),actor,RGB(110,148,181),1,PS_DOT);
    circle(dc,actor,10,RGB(111,177,249));circle(dc,actor,15,RGB(205,230,255),true);
    fill(dc,{0,0,r.right,top},RGB(24,37,52));
    text(dc,24,14,L"NAVIGATION LAB",RGB(113,222,184));
    text(dc,24,42,L"1: hand-drawn 3D paths    2: planar movement    3: free 3D flight    R: reset    O: obstacle    S: stop");
    text(dc,24,70,L"Click blank space: add node    Drag node: move    Shift-click: directed link    Ctrl + Shift-click: jump link");
    text(dc,24,98,L"Right-click: destination    Up / Down: node or target height    [ / ]: connected path width    Delete: remove node");
    fill(dc,{0,r.bottom-bottom,r.right,r.bottom},RGB(24,37,52));
    const auto state=app.world.navigator->state(1);
    text(dc,24,r.bottom-72,(state?phase(state->phase):L"Ready")+L"    |    "+app.world.message);
    text(dc,24,r.bottom-45,L"Mode "+std::to_wstring(app.world.mode)+L"   Selected node "+std::to_wstring(app.world.selected)
        +L"   Target height "+std::to_wstring(app.world.editHeight).substr(0,4)+L"   Speed "+std::to_wstring(length(body.velocity)).substr(0,4),RGB(162,185,208));
#ifdef AY_NAVIGATION_AUTHORING
    text(dc,24,r.bottom-22,L"F5: save graph as .nvg   F9: open graph   Green: corridors. Amber: special movement. Blue: unit.",RGB(126,154,179));
#else
    text(dc,24,r.bottom-22,L"Path edits stay in this session. Green: corridors. Amber: special movement. Blue: unit.",RGB(126,154,179));
#endif
    SelectObject(dc,oldFont);DeleteObject(font);
}
LRESULT CALLBACK procedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam) {
    auto* app=reinterpret_cast<App*>(GetWindowLongPtrW(window,GWLP_USERDATA));
    if(message==WM_NCCREATE){app=static_cast<App*>(reinterpret_cast<CREATESTRUCTW*>(lParam)->lpCreateParams);SetWindowLongPtrW(window,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(app));}
    if(!app)return DefWindowProcW(window,message,wParam,lParam);
    const POINT pixel{GET_X_LPARAM(lParam),GET_Y_LPARAM(lParam)};
    switch(message) {
    case WM_GETMINMAXINFO:reinterpret_cast<MINMAXINFO*>(lParam)->ptMinTrackSize={1120,680};return 0;
    case WM_LBUTTONDOWN:
        if(scene(window,pixel) && app->world.mode==1) {
            const auto id=pick(window,*app,pixel);
            if(id && (wParam&MK_SHIFT)) {app->world.connect(app->world.selected,id,bool(wParam&MK_CONTROL));app->world.selected=id;}
            else if(id) {app->world.selected=id;app->dragging=id;app->preview=position(*app,id);app->moved=false;SetCapture(window);}
            else app->world.addNode(location(window,pixel,app->world.editHeight));
        }
        break;
    case WM_MOUSEMOVE:
        if(app->dragging && (wParam&MK_LBUTTON) && scene(window,pixel)) {app->preview=location(window,pixel,position(*app,app->dragging).y);app->moved=true;}
        break;
    case WM_LBUTTONUP:
        if(app->dragging && app->moved)app->world.moveNode(app->dragging,app->preview);
        app->dragging=0;app->moved=false;if(GetCapture()==window)ReleaseCapture();break;
    case WM_CAPTURECHANGED:case WM_KILLFOCUS:app->dragging=0;app->moved=false;break;
    case WM_RBUTTONDOWN:
        if(scene(window,pixel)) {
            if(app->world.mode==1){const auto id=pick(window,*app,pixel);if(id)app->world.go({1,position(*app,id)});}
            else app->world.go({1,location(window,pixel,app->world.mode==2?0:app->world.editHeight)});
        }
        break;
    case WM_KEYDOWN:
#ifdef AY_NAVIGATION_AUTHORING
        if(wParam==VK_F5) {
            if(app->world.mode!=1){app->world.message=L"Switch to authored graph mode before saving.";break;}
            if(const auto path=navexample::graphDialog(window,true))app->world.message=writeGraphFile(*path,app->world.definition,true)==AssetError::None?L"Graph saved.":L"Save failed; prior file retained.";
            break;
        }
        if(wParam==VK_F9){if(const auto path=navexample::graphDialog(window,false))app->world.load(*path);break;}
#endif
        if(wParam>='1' && wParam<='3')app->world.reset(int(wParam-'0'));
        else if(wParam=='R')app->world.reset(app->world.mode);
        else if(wParam=='O')app->world.obstacles.enabled=!app->world.obstacles.enabled;
        else if(wParam=='S') {if(app->world.navigator->canInterrupt(1,app->world.task))app->world.navigator->cancel(1,app->world.task);else app->world.message=L"Landing first; ordinary stop is locked during the jump.";}
        else if(wParam==VK_UP || wParam==VK_DOWN) {
            const double delta=wParam==VK_UP?.5:-.5;app->world.editHeight=std::clamp(app->world.editHeight+delta,-8.0,8.0);
            if(app->world.mode==1 && app->world.selected) {auto p=position(*app,app->world.selected);p.y+=delta;app->world.moveNode(app->world.selected,p);}
        } else if((wParam==VK_OEM_4 || wParam==VK_OEM_6) && app->world.mode==1 && app->world.selected) {
            auto candidate=app->world.definition;for(auto& link:candidate.links)if(link.from==app->world.selected || link.to==app->world.selected)
                link.width=std::clamp(link.width+(wParam==VK_OEM_6?.25:-.25),.5,4.0);app->world.publish(std::move(candidate));
        } else if(wParam==VK_DELETE && app->world.mode==1 && app->world.selected && app->world.definition.nodes.size()>1) {
            auto candidate=app->world.definition;const auto id=app->world.selected;
            std::erase_if(candidate.nodes,[&](const auto& n){return n.id==id;});std::erase_if(candidate.links,[&](const auto& l){return l.from==id || l.to==id;});
            if(app->world.publish(std::move(candidate)))app->world.selected=0;
        }
        break;
    case WM_TIMER:app->world.tick();break;
    case WM_ERASEBKGND:return 1;
    case WM_PRINTCLIENT:paint(window,reinterpret_cast<HDC>(wParam),*app);return 0;
    case WM_PAINT:{PAINTSTRUCT ps;const auto dc=BeginPaint(window,&ps);RECT r;GetClientRect(window,&r);
        const auto memory=CreateCompatibleDC(dc);const auto bitmap=CreateCompatibleBitmap(dc,r.right,r.bottom);const auto prior=SelectObject(memory,bitmap);
        paint(window,memory,*app);BitBlt(dc,0,0,r.right,r.bottom,memory,0,0,SRCCOPY);SelectObject(memory,prior);DeleteObject(bitmap);DeleteDC(memory);EndPaint(window,&ps);return 0;}
    case WM_DESTROY:KillTimer(window,1);PostQuitMessage(0);return 0;
    default:return DefWindowProcW(window,message,wParam,lParam);
    }
    InvalidateRect(window,nullptr,FALSE);return 0;
}
bool renderReceipt(HWND window) {
    RECT r;GetClientRect(window,&r);BITMAPINFO info{};info.bmiHeader={sizeof(BITMAPINFOHEADER),r.right,-r.bottom,1,32,BI_RGB,0,0,0,0,0};
    const auto dc=CreateCompatibleDC(nullptr);void* pixels=nullptr;const auto bitmap=CreateDIBSection(dc,&info,DIB_RGB_COLORS,&pixels,nullptr,0);
    if(!bitmap){DeleteDC(dc);return false;}const auto old=SelectObject(dc,bitmap);SendMessageW(window,WM_PRINTCLIENT,reinterpret_cast<WPARAM>(dc),0);
    const bool rendered=GetPixel(dc,10,10)==RGB(24,37,52);const DWORD bytes=DWORD(r.right*r.bottom*4);
    BITMAPFILEHEADER header{0x4d42,DWORD(sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+bytes),0,0,DWORD(sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER))};
    const auto file=CreateFileW(L"AYNavigationDemo.bmp",GENERIC_WRITE,0,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
    bool saved=file!=INVALID_HANDLE_VALUE;
    if(saved){DWORD count=0;saved=bool(WriteFile(file,&header,sizeof(header),&count,nullptr)) && bool(WriteFile(file,&info.bmiHeader,sizeof(info.bmiHeader),&count,nullptr)) && bool(WriteFile(file,pixels,bytes,&count,nullptr));CloseHandle(file);}
    SelectObject(dc,old);DeleteObject(bitmap);DeleteDC(dc);return rendered && saved;
}
void click(HWND window,UINT message,POINT pixel,WPARAM modifiers=0){SendMessageW(window,message,modifiers,MAKELPARAM(pixel.x,pixel.y));}
bool smoke(HWND window,App& app) {
    click(window,WM_LBUTTONDOWN,screen(window,{-6,0,0}));click(window,WM_LBUTTONUP,screen(window,{-6,0,0}));
    const auto links=app.world.definition.links.size();click(window,WM_LBUTTONDOWN,screen(window,{-2,0,0}),MK_SHIFT);
    if(app.world.definition.links.size()!=links+1)return false;
    const auto nodes=app.world.definition.nodes.size();click(window,WM_LBUTTONDOWN,screen(window,{7,0,4}));
    if(app.world.definition.nodes.size()!=nodes+1)return false;
#ifdef AY_NAVIGATION_AUTHORING
    const auto graphPath=std::filesystem::temp_directory_path()/(L"aynav-single-"+std::to_wstring(GetCurrentProcessId())+L"-"+std::to_wstring(GetTickCount64())+L".nvg");
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove(path,error);}} cleanup{graphPath};
    if(writeGraphFile(graphPath,app.world.definition)!=AssetError::None||!app.world.load(graphPath)||app.world.definition.nodes.size()!=nodes+1)return false;
#endif
    click(window,WM_RBUTTONDOWN,screen(window,{6,1.5,0}));
    bool jumped=false;for(int i=0;i<1000;++i){SendMessageW(window,WM_TIMER,1,0);if(app.world.navigator->state(1)->phase==Phase::Traversing){jumped=true;break;}}
    if(!jumped || app.world.navigator->canInterrupt(1,app.world.task))return false;
    SendMessageW(window,WM_KEYDOWN,'S',0);if(app.world.navigator->state(1)->phase!=Phase::Traversing)return false;
    if(!renderReceipt(window))return false;
    for(int i=0;i<2000 && app.world.navigator->state(1)->phase!=Phase::Succeeded;++i)SendMessageW(window,WM_TIMER,1,0);
    if(app.world.navigator->state(1)->phase!=Phase::Succeeded)return false;
    for(int mode=2;mode<=3;++mode){SendMessageW(window,WM_KEYDOWN,WPARAM('0'+mode),0);
        if(mode==3)for(int i=0;i<4;++i)SendMessageW(window,WM_KEYDOWN,VK_UP,0);
        click(window,WM_RBUTTONDOWN,screen(window,{4,mode==3?2.0:0.0,1}));
        for(int i=0;i<2000 && app.world.navigator->state(1)->phase!=Phase::Succeeded;++i)SendMessageW(window,WM_TIMER,1,0);
        if(app.world.navigator->state(1)->phase!=Phase::Succeeded || std::abs(app.world.motor.body.location.position.y-(mode==3?2:0))>.03)return false;
    }
    return true;
}
}
int WINAPI wWinMain(HINSTANCE instance,HINSTANCE,PWSTR command,int show) {
    SetProcessDPIAware();App app;WNDCLASSW type{};type.lpfnWndProc=procedure;type.hInstance=instance;type.lpszClassName=L"AliyatNavigationLab";type.hCursor=LoadCursorW(nullptr,IDC_ARROW);
    if(!RegisterClassW(&type))return 1;RECT r{0,0,1200,780};AdjustWindowRect(&r,WS_OVERLAPPEDWINDOW,FALSE);
    const auto window=CreateWindowW(type.lpszClassName,L"Aliyat - Navigation lab",WS_OVERLAPPEDWINDOW,CW_USEDEFAULT,CW_USEDEFAULT,r.right-r.left,r.bottom-r.top,nullptr,nullptr,instance,&app);
    if(!window)return 1;if(std::wcsstr(command,L"--smoke")){const bool result=smoke(window,app);DestroyWindow(window);return result?0:1;}
    ShowWindow(window,show);SetTimer(window,1,33,nullptr);MSG message;
    while(GetMessageW(&message,nullptr,0,0)>0){TranslateMessage(&message);DispatchMessageW(&message);}return 0;
}
