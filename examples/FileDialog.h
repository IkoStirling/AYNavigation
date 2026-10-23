#pragma once
#include <windows.h>
#include <commdlg.h>
#include <filesystem>
#include <optional>
namespace navexample {
inline std::optional<std::filesystem::path> graphDialog(HWND owner,bool saving) {
    wchar_t buffer[32768]{};OPENFILENAMEW dialog{};dialog.lStructSize=sizeof(dialog);dialog.hwndOwner=owner;
    dialog.lpstrFilter=L"Navigation graph (*.nvg)\0*.nvg\0All files\0*.*\0";dialog.lpstrFile=buffer;
    dialog.nMaxFile=32768;dialog.lpstrDefExt=L"nvg";dialog.lpstrTitle=saving?L"Save navigation graph":L"Open navigation graph";
    dialog.Flags=OFN_EXPLORER|OFN_NOCHANGEDIR|OFN_PATHMUSTEXIST|(saving?OFN_OVERWRITEPROMPT:OFN_FILEMUSTEXIST);
    if(saving?GetSaveFileNameW(&dialog):GetOpenFileNameW(&dialog))return std::filesystem::path(buffer);
    return {};
}
}
