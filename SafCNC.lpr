{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : SafCNC.lpr - plugin entry point and the functions exported to Mach3
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  This plugin turns an Arduino Uno or Nano running the SafCNC firmware into a
  USB motion controller for Mach3. Mach3 plans the trajectory; the plugin cuts
  it into 5 ms slices and sends the step counts to the card over USB serial
  (115200 baud). The card replays them with a 32 kHz DDA timer.

  OTHER CARDS
    The plugin is not limited to the Arduino. Any card that speaks the same
    packet protocol can be used. An Arduino Uno / Nano cannot go beyond about
    30 kHz; an STM32 card can reach about 100 kHz (500 steps per 5 ms slice).
    The STM32 side has not been tested. The card reports its working speed to
    the plugin at start-up; if the Mach3 axis velocities exceed it, the plugin
    warns and does not link the card until the velocities are lowered.

  BUILD
    Mach3 is a 32-bit program, so this DLL must be built for i386-win32.
    A 64-bit build cannot be loaded. At start-up the plugin also verifies the
    Mach3 data layout and disables itself if it does not match.
      - Compiler : Lazarus / Free Pascal 3.2.x, target i386-win32
      - Install  : copy SafCNC.dll into the Mach3\PlugIns folder

  SAFETY
    Every packet carries a sequence counter and a checksum. The card stops on
    its own if the link is silent for 200 ms. Limit, probe and the card's own
    E-Stop input are handled on the card, independently of the PC.
    Always use a hardware E-Stop that cuts motor and spindle power.
}
library SafCNC;

{$mode objfpc}{$H+}
{$codepage utf8}
{$R-}{$Q-}

{$IFNDEF CPU386}
  {$FATAL SafCNC.dll must be built as 32-bit (i386-win32) - Mach3 is a 32-bit program.}
{$ENDIF}

uses
  Windows, SysUtils, Math,
  uWinApi, uMessages, uCardProtocol, uMainProgram, uSerialLink,
  uCardWindow, uCardLayer, uDevice;

const
  START_MSG_DELAY = 20;
  RESET_MSG_DELAY = 3;
  { Mouse clicks on the window frame and messages that start a Windows modal loop }
  WM_NCLBUTTONDOWN_ = $00A1;
  WM_NCRBUTTONDOWN_ = $00A4;
  WM_NCMBUTTONDOWN_ = $00A7;
  WM_NCLBUTTONDBLCLK_ = $00A3;
  WM_NCRBUTTONDBLCLK_ = $00A6;
  WM_NCMBUTTONDBLCLK_ = $00A9;
  WM_ENTERMENULOOP_ = $0211;
  WM_ENTERSIZEMOVE_ = $0231;
  WM_EXITSIZEMOVE_ = $0232;
  WM_SIZING_ = $0214;
  WM_MOVING_ = $0216;

var
  gDevice: TDevice = nil;
  gEnabled: Boolean = False;
  gMenuId: Integer = 0;
  gMenuHandle: HMENU = 0;
  gMenuWnd: HWND = 0;
  gMenuCandidate: HWND = 0;
  gHookWnd: HWND = 0;
  gOldWndProc: Pointer = nil;
  gHookUnicode: Boolean = False;
  gMoveLoop: Boolean = False;
  gMoveRect: TRect;
  gPluginNameA: AnsiString = '';

{ ================= Menu and window hook ================= }

{ Opens the SafCNC card status window. }
procedure OpenStatus;
var
  m: UnicodeString;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.ShowForm(gMenuWnd);
  except
    on e: Exception do
    begin
      m := ErrStatusWindow + UnicodeString(e.ClassName) + ': ' + UnicodeString(e.Message);
      w_MessageBoxW(0, PWideChar(m), PWideChar(PluginName), MB_OK);
    end;
  end;
end;

{ Window procedure hook on the Mach3 main window: catches clicks on our menu
  item and protects the jog from clicks on the window frame and the menu bar. }
function MenuHook(wnd: HWND; msg: UINT; wp: WPARAM; lp: LPARAM): LRESULT; stdcall;
var
  busy: Boolean;
begin
  if (msg = WM_COMMAND) and (gMenuId <> 0) and (lp = 0) and
     ((LongInt(wp) and $FFFF) = (gMenuId and $FFFF)) then
  begin
    OpenStatus;
    Exit(0);
  end;
  { Clicking the title bar, the frame or the menu bar makes Windows block the
    Mach3 window for a while (and dragging moves it), so no jog slices can be
    produced. While a jog key is held or the jog is still running, such a click
    stops the jog with deceleration and is not passed on to Windows: the window
    does not move and is never blocked. Keyboard-started window moves and menus
    (Alt) cannot be cancelled here, so they only stop the jog. }
  if (msg = WM_NCLBUTTONDOWN_) or (msg = WM_NCRBUTTONDOWN_) or (msg = WM_NCMBUTTONDOWN_) or
     (msg = WM_NCLBUTTONDBLCLK_) or (msg = WM_NCRBUTTONDBLCLK_) or (msg = WM_NCMBUTTONDBLCLK_) then
  begin
    busy := False;
    try
      if gEnabled and (gDevice <> nil) then busy := gDevice.WindowInteraction;
    except
    end;
    if busy then Exit(0);
  end
  else if (msg = WM_ENTERMENULOOP_) or (msg = WM_ENTERSIZEMOVE_) then
  begin
    if msg = WM_ENTERSIZEMOVE_ then
    begin
      gMoveLoop := False;
      if w_GetWindowRect(wnd, @gMoveRect) then gMoveLoop := True;
    end;
    try
      if gEnabled and (gDevice <> nil) then gDevice.WindowInteraction;
    except
    end;
  end
  else if msg = WM_EXITSIZEMOVE_ then
    gMoveLoop := False
  { A key was pressed while the window is being moved or resized with the mouse.
    Windows would move the window with the arrow keys and the jog would start
    from inside the move loop. The window is put back where it was, the move
    is cancelled and the jog stays locked until all keys are released. }
  else if ((msg = WM_MOVING_) or (msg = WM_SIZING_)) and gMoveLoop and (lp <> 0) then
  begin
    busy := False;
    try
      if gEnabled and (gDevice <> nil) then busy := gDevice.CheckWindowMoveKeys;
    except
    end;
    if busy then
    begin
      PRect(lp)^ := gMoveRect;
      gMoveLoop := False;
      w_ReleaseCapture;
      Exit(1);
    end;
  end;
  if gHookUnicode then
    Result := w_CallWindowProcW(gOldWndProc, wnd, msg, wp, lp)
  else
    Result := w_CallWindowProcA(gOldWndProc, wnd, msg, wp, lp);
end;

{ Subclasses the Mach3 main window; its messages go to MenuHook first. }
procedure InstallHook(wnd: HWND);
begin
  if (gHookWnd <> 0) or (wnd = 0) then Exit;
  gHookUnicode := IsWindowUnicode(wnd);
  if gHookUnicode then
    gOldWndProc := Pointer(PtrInt(w_SetWindowLongW(wnd, GWL_WNDPROC_, LongInt(PtrInt(@MenuHook)))))
  else
    gOldWndProc := Pointer(PtrInt(w_SetWindowLongA(wnd, GWL_WNDPROC_, LongInt(PtrInt(@MenuHook)))));
  if gOldWndProc <> nil then gHookWnd := wnd;
end;

{ Restores the original window procedure (unless someone hooked the window after us). }
procedure RemoveHook;
var
  cur: Pointer;
begin
  if gHookWnd = 0 then Exit;
  if gHookUnicode then
    cur := Pointer(PtrInt(w_GetWindowLongW(gHookWnd, GWL_WNDPROC_)))
  else
    cur := Pointer(PtrInt(w_GetWindowLongA(gHookWnd, GWL_WNDPROC_)));
  if cur = Pointer(@MenuHook) then
  begin
    if gHookUnicode then
      w_SetWindowLongW(gHookWnd, GWL_WNDPROC_, LongInt(PtrInt(gOldWndProc)))
    else
      w_SetWindowLongA(gHookWnd, GWL_WNDPROC_, LongInt(PtrInt(gOldWndProc)));
  end;
  gHookWnd := 0;
  gOldWndProc := nil;
end;

{ Position of the "PlugIn Control" menu; -1 if it is not found. }
function FindPlugInMenuPos(menu: HMENU): Integer;
var
  n, i: Integer;
  buf: array[0..255] of AnsiChar;
  s: AnsiString;
begin
  Result := -1;
  n := GetMenuItemCount(menu);
  for i := 0 to n - 1 do
  begin
    FillChar(buf, SizeOf(buf), 0);
    if w_GetMenuStringA(menu, UINT(i), @buf[0], 255, MF_BYPOSITION_) > 0 then
    begin
      s := LowerCase(StringReplace(AnsiString(PAnsiChar(@buf[0])), '&', '', [rfReplaceAll]));
      if Pos('plugin', s) > 0 then Exit(i);
    end;
  end;
end;

{ EnumWindows callback: finds the visible Mach3 main window that has a menu bar. }
function EnumMainWindow(wnd: HWND; lp: LPARAM): BOOL; stdcall;
var
  pid: DWORD;
  menu: HMENU;
begin
  Result := True;
  try
    pid := 0;
    w_GetWindowThreadProcessId(wnd, @pid);
    if pid <> GetCurrentProcessId then Exit;
    if not IsWindowVisible(wnd) then Exit;
    menu := GetMenu(wnd);
    if menu = 0 then Exit;
    if gMenuCandidate = 0 then gMenuCandidate := wnd;
    if FindPlugInMenuPos(menu) >= 0 then
    begin
      gMenuWnd := wnd;
      Result := False;
    end;
  except
  end;
end;

{ Adds the SafCNC item to the PlugIn Control menu (creates its own menu if that one is missing). }
procedure AddPlugInMenu;
var
  bar, sub: HMENU;
  menuPos: Integer;
  itemName, mainName: AnsiString;
begin
  if (gMenuId <> 0) or (not Assigned(GetMenuRange)) then Exit;
  gMenuId := GetMenuRange(1);
  if gMenuId = 0 then Exit;
  gMenuWnd := 0;
  gMenuCandidate := 0;
  w_EnumWindows(@EnumMainWindow, 0);
  if gMenuWnd = 0 then gMenuWnd := gMenuCandidate;
  if gMenuWnd = 0 then Exit;
  bar := GetMenu(gMenuWnd);
  sub := 0;
  menuPos := FindPlugInMenuPos(bar);
  if menuPos >= 0 then sub := GetSubMenu(bar, menuPos);
  itemName := AnsiString(MenuName);
  mainName := AnsiString(MainMenuName);
  if sub <> 0 then
  begin
    gMenuHandle := sub;
    w_AppendMenuA(sub, MF_STRING_, UINT_PTR(gMenuId), PAnsiChar(itemName));
  end
  else
  begin
    gMenuHandle := CreatePopupMenu;
    w_AppendMenuA(gMenuHandle, MF_STRING_, UINT_PTR(gMenuId), PAnsiChar(itemName));
    w_AppendMenuA(bar, MF_POPUP_, UINT_PTR(gMenuHandle), PAnsiChar(mainName));
  end;
  DrawMenuBar(gMenuWnd);
  InstallHook(gMenuWnd);
end;

{ Removes our menu item and the window hook when the plugin stops. }
procedure RemovePlugInMenu;
begin
  RemoveHook;
  if (gMenuId <> 0) and (gMenuHandle <> 0) then
  begin
    w_DeleteMenu(gMenuHandle, UINT(gMenuId), MF_BYCOMMAND_);
    if gMenuWnd <> 0 then DrawMenuBar(gMenuWnd);
  end;
  gMenuId := 0;
  gMenuHandle := 0;
end;

{ ================= Functions exported to Mach3 ================= }

{ The Set... functions below receive and store the Mach3 callback pointers
  (buttons, DROs, LEDs, G-code). }
procedure exp_SetGetMenuRange(func: Pointer); cdecl;
begin
  GetMenuRange := TIntShort(func);
end;

procedure exp_SetDoButton(func: Pointer); cdecl;
begin
  DoButton := TVoidShort(func);
end;

procedure exp_SetSetDRO(func: Pointer); cdecl;
begin
  SetDRO := TVoidShortDouble(func);
end;

procedure exp_SetGetDRO(func: Pointer); cdecl;
begin
  GetDRO := TDoubleShort(func);
end;

procedure exp_SetGetLED(func: Pointer); cdecl;
begin
  GetLED := TBoolShort(func);
end;

procedure exp_SetSetLED(func: Pointer); cdecl;
begin
  SetLED := TVoidShortBool(func);
end;

procedure exp_SetCode(func: Pointer); cdecl;
begin
  Code := TVoidPChar(func);
end;

{ Called first by Mach3. Stores the Mach3 data block pointers and checks that the
  memory layout matches the 32-bit build. If it does not, the plugin stays disabled. }
function exp_InitControl(oEngine, oSetup, oMainPlanner, oView: Pointer): Boolean; cdecl;
var
  m: UnicodeString;
begin
  Result := True;
  try
    Engine := oEngine;
    SetupBlock := oSetup;
    MainPlanner := oMainPlanner;
    ViewBlock := oView;
    gEnabled := LayoutValid and (oEngine <> nil) and (oMainPlanner <> nil) and (oView <> nil);
    if not gEnabled then
    begin
      m := PluginName + ': ' + ErrStructure;
      w_MessageBoxW(0, PWideChar(m), PWideChar(PluginName), MB_OK);
      Exit;
    end;
    if gDevice = nil then gDevice := TDevice.Create;
    gDevice.InitControl;
  except
    gEnabled := False;
  end;
end;

{ Receives the active Mach3 profile name and returns the plugin name. }
function exp_SetProName(name: PAnsiChar): PAnsiChar; cdecl;
begin
  try
    if name <> nil then ProfileName := AnsiString(name) else ProfileName := '';
  except
  end;
  if gPluginNameA = '' then gPluginNameA := AnsiString(PluginName);
  Result := PAnsiChar(gPluginNameA);
end;

{ Called when Mach3 has finished starting up: connects to the card and adds the menu. }
procedure exp_PostInitControl; cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then
    begin
      DeferStatusMessage(START_MSG_DELAY);
      gDevice.PostInitControl;
      AddPlugInMenu;
    end;
  except
  end;
end;

procedure exp_Config; cdecl;
begin
end;

{ Called when Mach3 closes: stops the card and releases the resources. }
procedure exp_StopPlug; cdecl;
begin
  try
    if gDevice <> nil then gDevice.Shutdown;
    RemovePlugInMenu;
    if gDevice <> nil then gDevice.FreeForm;
  except
  end;
end;

{ Called by Mach3 on every update cycle. This is the plugin's main service loop. }
procedure exp_Update; cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then
    begin
      gDevice.Update;
      gDevice.UpdateForm;
      StatusMessageService;
    end;
  except
  end;
end;

{ Mach3 menu notification: opens the status window when our item is selected. }
procedure exp_Notify(id: Integer); cdecl;
begin
  try
    if (gMenuId <> 0) and (id = gMenuId) then OpenStatus;
  except
  end;
end;

{ G4 dwell command. }
procedure exp_DoDwell(dwellTime: Double); cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.Dwell(dwellTime);
  except
  end;
end;

{ The Mach3 Reset button was pressed (entering or leaving E-Stop). }
procedure exp_Reset; cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then
    begin
      DeferStatusMessage(RESET_MSG_DELAY);
      gDevice.Reset;
    end;
  except
  end;
end;

{ Jog key press / release, Purge, Probe and Home requests from Mach3. }
procedure exp_JogOn(axis, dir: SmallInt; speed: Double); cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.JogOn(axis, dir, speed);
  except
  end;
end;

procedure exp_JogOff(axis: SmallInt); cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.JogOff(axis);
  except
  end;
end;

procedure exp_Purge(flags: SmallInt); cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.Purge(flags);
  except
  end;
end;

procedure exp_Probe; cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.Probe;
  except
  end;
end;

procedure exp_Home(axis: SmallInt); cdecl;
begin
  try
    if gEnabled and (gDevice <> nil) then gDevice.Home(axis);
  except
  end;
end;

exports
  exp_SetGetMenuRange name 'SetGetMenuRange',
  exp_SetDoButton name 'SetDoButton',
  exp_SetSetDRO name 'SetSetDRO',
  exp_SetGetDRO name 'SetGetDRO',
  exp_SetGetLED name 'SetGetLED',
  exp_SetSetLED name 'SetSetLED',
  exp_SetCode name 'SetCode',
  exp_InitControl name 'InitControl',
  exp_SetProName name 'SetProName',
  exp_PostInitControl name 'PostInitControl',
  exp_Config name 'Config',
  exp_StopPlug name 'StopPlug',
  exp_Update name 'Update',
  exp_Notify name 'Notify',
  exp_DoDwell name 'DoDwell',
  exp_Reset name 'Reset',
  exp_JogOn name 'JogOn',
  exp_JogOff name 'JogOff',
  exp_Purge name 'Purge',
  exp_Probe name 'Probe',
  exp_Home name 'Home';

begin
  { Mach3 (MSVC) runs with floating point exceptions masked. The FPC runtime may
    unmask them while the DLL is loaded; they are all masked here so that Mach3 is
    not disturbed. NaN / infinite values are checked separately with IsNan / IsInfinite. }
  SetExceptionMask([exInvalidOp, exDenormalized, exZeroDivide, exOverflow,
    exUnderflow, exPrecision]);
end.
