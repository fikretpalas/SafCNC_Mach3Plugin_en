{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uWinApi.pas - Win32 API declarations
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  APIs whose signature differs between FPC versions of the Windows unit, or
  that are awkward to call because of var/const parameters, are declared here
  explicitly.
}
unit uWinApi;

{$mode objfpc}{$H+}
{$R-}{$Q-}

interface

uses
  Windows;

const
  CLRRTS_ = 4;
  CLRDTR_ = 6;
  PURGE_ALL_ = $0F;
  ERROR_IO_PENDING_ = 997;
  WAIT_OBJECT_0_ = 0;
  WAIT_TIMEOUT_ = 258;
  GWL_WNDPROC_ = -4;
  MF_STRING_ = $0000;
  MF_BYCOMMAND_ = $0000;
  MF_POPUP_ = $0010;
  MF_BYPOSITION_ = $0400;
  IDC_ARROW_ = 32512;
  ICON_SMALL_ = 0;
  ICON_BIG_ = 1;
  SM_CXICON_ = 11;
  SM_CXSMICON_ = 49;
  PROCESS_POWER_THROTTLING_ = 4;

type
  TOverlapped_ = record
    Internal: DWORD;
    InternalHigh: DWORD;
    Offset: DWORD;
    OffsetHigh: DWORD;
    hEvent: THandle;
  end;
  POverlapped_ = ^TOverlapped_;

  TDCB_ = packed record
    DCBlength: DWORD;
    BaudRate: DWORD;
    Flags: DWORD;
    wReserved: Word;
    XonLim: Word;
    XoffLim: Word;
    ByteSize: Byte;
    Parity: Byte;
    StopBits: Byte;
    XonChar: AnsiChar;
    XoffChar: AnsiChar;
    ErrorChar: AnsiChar;
    EofChar: AnsiChar;
    EvtChar: AnsiChar;
    wReserved1: Word;
  end;

  TCommTimeouts_ = record
    ReadIntervalTimeout: DWORD;
    ReadTotalTimeoutMultiplier: DWORD;
    ReadTotalTimeoutConstant: DWORD;
    WriteTotalTimeoutMultiplier: DWORD;
    WriteTotalTimeoutConstant: DWORD;
  end;

  TPowerThrottling_ = record
    Version: DWORD;
    ControlMask: DWORD;
    StateMask: DWORD;
  end;

  TWndClassW_ = record
    style: UINT;
    lpfnWndProc: Pointer;
    cbClsExtra: Integer;
    cbWndExtra: Integer;
    hInstanceW: HINST;
    hIconW: HICON;
    hCursorW: HCURSOR;
    hbrBackground: HBRUSH;
    lpszMenuName: PWideChar;
    lpszClassName: PWideChar;
  end;

  TPaintStruct_ = record
    dcPaint: HDC;
    fErase: BOOL;
    rcPaint: TRect;
    fRestore: BOOL;
    fIncUpdate: BOOL;
    rgbReserved: array[0..31] of Byte;
  end;

  TDrawItemStruct_ = record
    CtlType: UINT;
    CtlID: UINT;
    itemID: UINT;
    itemAction: UINT;
    itemState: UINT;
    hwndItem: HWND;
    dcItem: HDC;
    rcItem: TRect;
    itemData: PtrUInt;
  end;
  PDrawItemStruct_ = ^TDrawItemStruct_;

  TBmiHeader_ = record
    biSize: DWORD;
    biWidth: LongInt;
    biHeight: LongInt;
    biPlanes: Word;
    biBitCount: Word;
    biCompression: DWORD;
    biSizeImage: DWORD;
    biXPelsPerMeter: LongInt;
    biYPelsPerMeter: LongInt;
    biClrUsed: DWORD;
    biClrImportant: DWORD;
  end;

  TIconInfo_ = record
    fIcon: BOOL;
    xHotspot: DWORD;
    yHotspot: DWORD;
    hbmMask: HBITMAP;
    hbmColor: HBITMAP;
  end;

{ kernel32 }
function w_CreateFileA(name: PAnsiChar; access, share: DWORD; sa: Pointer;
  disp, flags: DWORD; tmpl: THandle): THandle; stdcall; external 'kernel32.dll' name 'CreateFileA';
function w_ReadFile(h: THandle; buf: Pointer; n: DWORD; nRead: PDWORD;
  ov: POverlapped_): BOOL; stdcall; external 'kernel32.dll' name 'ReadFile';
function w_WriteFile(h: THandle; buf: Pointer; n: DWORD; nWritten: PDWORD;
  ov: POverlapped_): BOOL; stdcall; external 'kernel32.dll' name 'WriteFile';
function w_GetOverlappedResult(h: THandle; ov: POverlapped_; var n: DWORD;
  wait: BOOL): BOOL; stdcall; external 'kernel32.dll' name 'GetOverlappedResult';
function w_CancelIo(h: THandle): BOOL; stdcall; external 'kernel32.dll' name 'CancelIo';
function w_CreateEventW(sa: Pointer; manualReset, initialState: BOOL;
  name: PWideChar): THandle; stdcall; external 'kernel32.dll' name 'CreateEventW';
function w_GetCommState(h: THandle; dcb: Pointer): BOOL; stdcall; external 'kernel32.dll' name 'GetCommState';
function w_SetCommState(h: THandle; dcb: Pointer): BOOL; stdcall; external 'kernel32.dll' name 'SetCommState';
function w_SetCommTimeouts(h: THandle; t: Pointer): BOOL; stdcall; external 'kernel32.dll' name 'SetCommTimeouts';
function w_SetupComm(h: THandle; inQ, outQ: DWORD): BOOL; stdcall; external 'kernel32.dll' name 'SetupComm';
function w_PurgeComm(h: THandle; flags: DWORD): BOOL; stdcall; external 'kernel32.dll' name 'PurgeComm';
function w_EscapeCommFunction(h: THandle; func: DWORD): BOOL; stdcall; external 'kernel32.dll' name 'EscapeCommFunction';
function w_QueryPerformanceCounter(var v: Int64): BOOL; stdcall; external 'kernel32.dll' name 'QueryPerformanceCounter';
function w_QueryPerformanceFrequency(var v: Int64): BOOL; stdcall; external 'kernel32.dll' name 'QueryPerformanceFrequency';

{ winmm }
function w_timeBeginPeriod(p: UINT): UINT; stdcall; external 'winmm.dll' name 'timeBeginPeriod';
function w_timeEndPeriod(p: UINT): UINT; stdcall; external 'winmm.dll' name 'timeEndPeriod';

{ user32 }
function w_SetWindowLongA(wnd: HWND; idx: Integer; v: LongInt): LongInt; stdcall; external 'user32.dll' name 'SetWindowLongA';
function w_SetWindowLongW(wnd: HWND; idx: Integer; v: LongInt): LongInt; stdcall; external 'user32.dll' name 'SetWindowLongW';
function w_GetWindowLongA(wnd: HWND; idx: Integer): LongInt; stdcall; external 'user32.dll' name 'GetWindowLongA';
function w_GetWindowLongW(wnd: HWND; idx: Integer): LongInt; stdcall; external 'user32.dll' name 'GetWindowLongW';
function w_CallWindowProcA(prev: Pointer; wnd: HWND; msg: UINT; wp: WPARAM;
  lp: LPARAM): LRESULT; stdcall; external 'user32.dll' name 'CallWindowProcA';
function w_CallWindowProcW(prev: Pointer; wnd: HWND; msg: UINT; wp: WPARAM;
  lp: LPARAM): LRESULT; stdcall; external 'user32.dll' name 'CallWindowProcW';
function w_DefWindowProcW(wnd: HWND; msg: UINT; wp: WPARAM; lp: LPARAM): LRESULT; stdcall; external 'user32.dll' name 'DefWindowProcW';
function w_EnumWindows(fn: Pointer; lp: LPARAM): BOOL; stdcall; external 'user32.dll' name 'EnumWindows';
function w_GetWindowThreadProcessId(wnd: HWND; pid: PDWORD): DWORD; stdcall; external 'user32.dll' name 'GetWindowThreadProcessId';
function w_SetTimer(wnd: HWND; id: UINT_PTR; elapse: UINT; fn: Pointer): UINT_PTR; stdcall; external 'user32.dll' name 'SetTimer';
function w_KillTimer(wnd: HWND; id: UINT_PTR): BOOL; stdcall; external 'user32.dll' name 'KillTimer';
function w_GetMenuStringA(menu: HMENU; item: UINT; buf: PAnsiChar; maxCount: Integer;
  flags: UINT): Integer; stdcall; external 'user32.dll' name 'GetMenuStringA';
function w_AppendMenuA(menu: HMENU; flags: UINT; id: UINT_PTR; text: PAnsiChar): BOOL; stdcall; external 'user32.dll' name 'AppendMenuA';
function w_DeleteMenu(menu: HMENU; position, flags: UINT): BOOL; stdcall; external 'user32.dll' name 'DeleteMenu';
function w_MessageBoxW(wnd: HWND; text, caption: PWideChar; t: UINT): Integer; stdcall; external 'user32.dll' name 'MessageBoxW';
function w_RegisterClassW(wc: Pointer): ATOM; stdcall; external 'user32.dll' name 'RegisterClassW';
function w_UnregisterClassW(cls: PWideChar; inst: HINST): BOOL; stdcall; external 'user32.dll' name 'UnregisterClassW';
function w_CreateWindowExW(exStyle: DWORD; cls, title: PWideChar; style: DWORD;
  x, y, w, h: Integer; parent: HWND; menu: HMENU; inst: HINST;
  param: Pointer): HWND; stdcall; external 'user32.dll' name 'CreateWindowExW';
function w_SetWindowTextW(wnd: HWND; text: PWideChar): BOOL; stdcall; external 'user32.dll' name 'SetWindowTextW';
function w_SendMessageW(wnd: HWND; msg: UINT; wp: WPARAM; lp: LPARAM): LRESULT; stdcall; external 'user32.dll' name 'SendMessageW';
function w_GetDC(wnd: HWND): HDC; stdcall; external 'user32.dll' name 'GetDC';
function w_ReleaseDC(wnd: HWND; dc: HDC): Integer; stdcall; external 'user32.dll' name 'ReleaseDC';
function w_LoadCursorW(inst: HINST; name: PWideChar): HCURSOR; stdcall; external 'user32.dll' name 'LoadCursorW';
function w_AdjustWindowRectEx(r: PRect; style: DWORD; menu: BOOL; exStyle: DWORD): BOOL; stdcall; external 'user32.dll' name 'AdjustWindowRectEx';
function w_GetWindowRect(wnd: HWND; r: PRect): BOOL; stdcall; external 'user32.dll' name 'GetWindowRect';
function w_ReleaseCapture: BOOL; stdcall; external 'user32.dll' name 'ReleaseCapture';
function w_BeginPaint(wnd: HWND; ps: Pointer): HDC; stdcall; external 'user32.dll' name 'BeginPaint';
function w_EndPaint(wnd: HWND; ps: Pointer): BOOL; stdcall; external 'user32.dll' name 'EndPaint';
function w_InvalidateRect(wnd: HWND; r: PRect; erase: BOOL): BOOL; stdcall; external 'user32.dll' name 'InvalidateRect';
function w_FillRect(dc: HDC; r: PRect; br: HBRUSH): Integer; stdcall; external 'user32.dll' name 'FillRect';
function w_FrameRect(dc: HDC; r: PRect; br: HBRUSH): Integer; stdcall; external 'user32.dll' name 'FrameRect';
function w_DrawTextW(dc: HDC; text: PWideChar; count: Integer; r: PRect;
  fmt: UINT): Integer; stdcall; external 'user32.dll' name 'DrawTextW';
function w_CreateIconIndirect(ii: Pointer): HICON; stdcall; external 'user32.dll' name 'CreateIconIndirect';

{ gdi32 }
function w_CreateFontW(h, w, esc, ori, weight: Integer; italic, underline, strikeOut,
  charSet, outPrecision, clipPrecision, quality, pitchAndFamily: DWORD;
  face: PWideChar): HFONT; stdcall; external 'gdi32.dll' name 'CreateFontW';
function w_CreateBitmap(w, h: Integer; planes, bitCount: UINT; bits: Pointer): HBITMAP; stdcall; external 'gdi32.dll' name 'CreateBitmap';
function w_SetDIBitsToDevice(dc: HDC; xDest, yDest: Integer; w, h: DWORD;
  xSrc, ySrc: Integer; startScan, lines: UINT; bits: Pointer; bmi: Pointer;
  colorUse: UINT): Integer; stdcall; external 'gdi32.dll' name 'SetDIBitsToDevice';

{ APIs loaded at run time (may be missing on old Windows versions) }
function TickMs64: QWord;
function SetProcessInformation_(h: THandle; infoClass: Integer; info: Pointer; size: DWORD): Boolean;
function DwmSetWindowAttribute_(wnd: HWND; attr: DWORD; value: Pointer; size: DWORD): LongInt;

implementation

type
  TGetTickCount64Fn = function: QWord; stdcall;
  TSetProcessInformationFn = function(h: THandle; infoClass: Integer; info: Pointer; size: DWORD): BOOL; stdcall;
  TDwmSetWindowAttributeFn = function(wnd: HWND; attr: DWORD; value: Pointer; size: DWORD): LongInt; stdcall;

var
  pGetTickCount64: TGetTickCount64Fn = nil;
  pSetProcessInformation: TSetProcessInformationFn = nil;
  pDwmSetWindowAttribute: TDwmSetWindowAttributeFn = nil;
  gDwmLoaded: Boolean = False;

function TickMs64: QWord;
begin
  if Assigned(pGetTickCount64) then
    Result := pGetTickCount64()
  else
    Result := GetTickCount;
end;

function SetProcessInformation_(h: THandle; infoClass: Integer; info: Pointer; size: DWORD): Boolean;
begin
  Result := False;
  if Assigned(pSetProcessInformation) then
    Result := pSetProcessInformation(h, infoClass, info, size);
end;

function DwmSetWindowAttribute_(wnd: HWND; attr: DWORD; value: Pointer; size: DWORD): LongInt;
var
  lib: HMODULE;
begin
  if not gDwmLoaded then
  begin
    gDwmLoaded := True;
    lib := LoadLibraryA('dwmapi.dll');
    if lib <> 0 then
      pDwmSetWindowAttribute := TDwmSetWindowAttributeFn(GetProcAddress(lib, 'DwmSetWindowAttribute'));
  end;
  if Assigned(pDwmSetWindowAttribute) then
    Result := pDwmSetWindowAttribute(wnd, attr, value, size)
  else
    Result := -1;
end;

procedure LoadKernelFunctions;
var
  k: HMODULE;
begin
  k := GetModuleHandleA('kernel32.dll');
  if k = 0 then Exit;
  pGetTickCount64 := TGetTickCount64Fn(GetProcAddress(k, 'GetTickCount64'));
  pSetProcessInformation := TSetProcessInformationFn(GetProcAddress(k, 'SetProcessInformation'));
end;

initialization
  LoadKernelFunctions;

end.
