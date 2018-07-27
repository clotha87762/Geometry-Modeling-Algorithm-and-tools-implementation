#pragma once

#include <Windows.h>
#include <Commdlg.h>
#include <tchar.h>

class OpenFileDialog
{
public:
    OpenFileDialog(void);

    TCHAR*DefaultExtension;
    TCHAR*FileName;
    TCHAR*Filter,
    intFilterIndex,
    intFlags;
    TCHAR*InitialDir,
    HWNDOwner;
    TCHAR*Title;

    bool ShowDialog();
};