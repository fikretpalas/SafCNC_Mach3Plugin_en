{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uCardWindow.pas - card status window
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  The window opened from PlugIn Control > SafCNC: link lamp, port and baud
  rate. A plain Win32 window with a dark theme; the LCL is not used.
  The axis velocity warning uses the same theme: it opens when the Mach3
  velocities exceed the card limit and is closed with OK.
}
unit uCardWindow;

{$mode objfpc}{$H+}
{$codepage utf8}
{$R-}{$Q-}

interface

uses
  Windows, SysUtils, Math, uWinApi, uMessages, uMainProgram;

type
  { Information shown in the status window }
  TCardInfo = record
    Linked: Boolean;
    Present: Boolean;
    PortName: UnicodeString;
    Baud: DWord;
    HasFb: Boolean;
    Lamp: Byte;
  end;

  TCardInfoFn = function: TCardInfo of object;

  TLampMode = (lmUnknown, lmBlink1Hz, lmSteady, lmBlink4Hz);

  TRgb = record
    R, G, B: Byte;
  end;

  { Round status lamp with an anti-aliased edge.
    Steady, or blinking at 1 Hz or 4 Hz, depending on the card state. }
  TVirtualLamp = class
  private
    FSize: Integer;
    FMode: TLampMode;
    FLit: Boolean;
    FCounter: Integer;
    FDirty: Boolean;
    FPixels: array of DWord;
    procedure ComputeState;
    procedure SetMode(val: TLampMode);
  public
    constructor Create(sizePx: Integer);
    procedure Tick;
    procedure ApplyCardCode(codeVal: Byte);
    function NeedsRedraw: Boolean;
    procedure Draw(dc: HDC; x, y: Integer; bgColor: DWord);
    class function CreateLitIcon(sizePx: Integer): HICON;
  end;

const
  LAMP_TICK_MS = 125;

procedure ShowCardWindow(owner: HWND; getInfo: TCardInfoFn);
procedure UpdateCardWindow;
procedure FreeCardWindow;

{ Opens the axis velocity warning (does nothing if it is already open). The
  window is not modal and always stays on top; Mach3 keeps running its own loop. }
procedure ShowSpeedWarning(const txt: UnicodeString);
function SpeedWarningOpen: Boolean;
procedure CloseSpeedWarning;

implementation

const
  WND_CLASS: UnicodeString = 'SafCNCCardWnd';
  FONT_FACE: UnicodeString = 'Segoe UI';
  CLS_STATIC: UnicodeString = 'STATIC';
  CLR_GREEN = $007DD73C;
  CLR_ORANGE = $000078C8;
  CLR_RED = $005046EB;
  CLR_BACKGROUND = $0016110F;
  CLR_SURFACE = $00231B18;
  CLR_BORDER = $004E3D36;
  CLR_TEXT = $00F0E6E0;
  CLR_ACCENT = $00FFC800;
  ID_LAMP = 130;
  ID_TIMER = 1;
  LAMP_SIZE = 22;
  SS_OWNERDRAW_ = $0000000D;
  DT_SINGLELINE_ = $0020;
  DT_NOPREFIX_ = $0800;
  LINE_COUNT = 2;
  PNL_X = 6;
  PNL_W = 360;
  PNL_PROP_Y = 6;
  PNL_PROP_H = 80;
  REFRESH_MS = 200;
  CLIENT_W = 372;
  CLIENT_H = PNL_PROP_Y + PNL_PROP_H + 6;
  W_CLASS: UnicodeString = 'SafCNCSpeedWnd';
  CLS_BUTTON: UnicodeString = 'BUTTON';
  ID_OK = 140;
  W_CLIENT_W = 480;
  W_TEXT_Y = 30;
  BTN_W = 120;
  BTN_H = 30;
  BS_OWNERDRAW_ = $0000000B;
  ODT_BUTTON_ = 4;
  ODS_SELECTED_ = $0001;
  BN_CLICKED_ = 0;
  DT_CENTER_ = $0001;
  DT_VCENTER_ = $0004;
  DT_WORDBREAK_ = $0010;
  DT_CALCRECT_ = $0400;
  WS_EX_TOPMOST_ = $0008;
  SM_CXSCREEN_ = 0;
  SM_CYSCREEN_ = 1;
  MB_TOPMOST_ = $00040000;

var
  gBrBackground: HBRUSH = 0;
  gBrSurface: HBRUSH = 0;
  gBrBorder: HBRUSH = 0;
  gWnd: HWND = 0;
  gFontN: HFONT = 0;
  gFontB: HFONT = 0;
  gClassReady: Boolean = False;
  gLbls: array[0..LINE_COUNT - 1] of HWND;
  gLamp: TVirtualLamp = nil;
  gLampWnd: HWND = 0;
  gIconSmall: HICON = 0;
  gIconBig: HICON = 0;
  gShown: array[0..LINE_COUNT - 1] of UnicodeString;
  gStatusColor: DWord = CLR_RED;
  gLastRefresh: QWord = 0;
  gGetInfo: TCardInfoFn;
  gBrAccent: HBRUSH = 0;
  gWarnWnd: HWND = 0;
  gWarnText: UnicodeString = '';
  gWarnTextH: Integer = 0;
  gWarnClassReady: Boolean = False;

{ ================= TVirtualLamp ================= }

function MakeRgb(r, g, b: Integer): TRgb;
begin
  Result.R := Byte(r);
  Result.G := Byte(g);
  Result.B := Byte(b);
end;

function Blend(const c1, c2: TRgb; a: Double): TRgb;
begin
  if a <= 0 then Exit(c1);
  if a >= 1 then Exit(c2);
  Result := MakeRgb(
    Round(c1.R + (c2.R - c1.R) * a),
    Round(c1.G + (c2.G - c1.G) * a),
    Round(c1.B + (c2.B - c1.B) * a));
end;

function Clamp01(v: Double): Double;
begin
  if v < 0 then Exit(0);
  if v > 1 then Exit(1);
  Result := v;
end;

{ Draws the lamp into a pixel buffer (soft edge, highlight). }
procedure Render(var dest: array of DWord; n: Integer; const baseColor: TRgb;
  active: Boolean; const bgRgb: TRgb; withAlpha: Boolean);
var
  white, bright, dark, fill, c: TRgb;
  cx, cy, r, hx, hy, sx, sy, px, py, dist, coverage, hd, rim, sd, sp: Double;
  ix, iy: Integer;
  a, v: DWord;
begin
  white := MakeRgb(255, 255, 255);
  if active then bright := Blend(baseColor, white, 0.6)
  else bright := Blend(baseColor, white, 0.2);
  dark := Blend(baseColor, MakeRgb(0, 0, 0), 0.6);
  cx := n / 2.0;
  cy := n / 2.0;
  r := n / 2.0 - 1.5;
  hx := cx - r * 0.3;
  hy := cy - r * 0.35;
  sx := cx - r * 0.35;
  sy := cy - r * 0.4;
  for iy := 0 to n - 1 do
    for ix := 0 to n - 1 do
    begin
      px := ix + 0.5;
      py := iy + 0.5;
      dist := Sqrt(Sqr(px - cx) + Sqr(py - cy));
      coverage := Clamp01(r - dist + 0.5);
      fill := bgRgb;
      if coverage > 0 then
      begin
        hd := Sqrt(Sqr(px - hx) + Sqr(py - hy)) / r;
        fill := Blend(bright, baseColor, Clamp01(hd * 1.3));
        rim := Clamp01((dist - (r - 2.0)) / 2.0);
        fill := Blend(fill, dark, rim * 0.85);
        if active then
        begin
          sd := Sqrt(Sqr(px - sx) + Sqr(py - sy));
          sp := Clamp01(1.5 - sd / (r * 0.3));
          fill := Blend(fill, white, sp * 0.85);
        end;
      end;
      if withAlpha then
      begin
        a := DWord(Round(coverage * 255));
        if a = 0 then v := 0
        else v := (a shl 24) or (DWord(fill.R) shl 16) or (DWord(fill.G) shl 8) or fill.B;
        dest[iy * n + ix] := v;
      end
      else
      begin
        c := Blend(bgRgb, fill, coverage);
        v := (DWord(c.R) shl 16) or (DWord(c.G) shl 8) or c.B;
        dest[(n - 1 - iy) * n + ix] := v;
      end;
    end;
end;

constructor TVirtualLamp.Create(sizePx: Integer);
begin
  inherited Create;
  if sizePx < 8 then sizePx := 8;
  FSize := sizePx;
  SetLength(FPixels, FSize * FSize);
  FMode := lmUnknown;
  FLit := False;
  FCounter := 0;
  FDirty := True;
end;

procedure TVirtualLamp.ComputeState;
var
  lampOn: Boolean;
begin
  case FMode of
    lmSteady: lampOn := True;
    lmBlink1Hz: lampOn := FCounter < 4;
    lmBlink4Hz: lampOn := (FCounter mod 2) = 0;
  else
    lampOn := False;
  end;
  if lampOn <> FLit then
  begin
    FLit := lampOn;
    FDirty := True;
  end;
end;

procedure TVirtualLamp.Tick;
begin
  FCounter := (FCounter + 1) mod 8;
  ComputeState;
end;

procedure TVirtualLamp.SetMode(val: TLampMode);
begin
  if FMode = val then Exit;
  FMode := val;
  FCounter := 0;
  ComputeState;
  FDirty := True;
end;

procedure TVirtualLamp.ApplyCardCode(codeVal: Byte);
begin
  case codeVal of
    0: SetMode(lmBlink1Hz);
    1: SetMode(lmSteady);
    2: SetMode(lmBlink4Hz);
    6: SetMode(lmBlink4Hz);
  else
    SetMode(lmUnknown);
  end;
end;

function TVirtualLamp.NeedsRedraw: Boolean;
begin
  Result := FDirty;
  FDirty := False;
end;

procedure TVirtualLamp.Draw(dc: HDC; x, y: Integer; bgColor: DWord);
var
  bgRgb, baseColor: TRgb;
  active: Boolean;
  bmi: TBmiHeader_;
begin
  if (dc = 0) or (Length(FPixels) = 0) then Exit;
  bgRgb := MakeRgb(bgColor and $FF, (bgColor shr 8) and $FF, (bgColor shr 16) and $FF);
  active := FLit and (FMode <> lmUnknown);
  if FMode = lmUnknown then baseColor := MakeRgb(80, 86, 98)
  else if FLit then baseColor := MakeRgb(0, 255, 0)
  else baseColor := MakeRgb(0, 60, 0);
  Render(FPixels, FSize, baseColor, active, bgRgb, False);
  FillChar(bmi, SizeOf(bmi), 0);
  bmi.biSize := SizeOf(TBmiHeader_);
  bmi.biWidth := FSize;
  bmi.biHeight := FSize;
  bmi.biPlanes := 1;
  bmi.biBitCount := 32;
  bmi.biCompression := 0;
  w_SetDIBitsToDevice(dc, x, y, FSize, FSize, 0, 0, 0, FSize, @FPixels[0], @bmi, 0);
end;

class function TVirtualLamp.CreateLitIcon(sizePx: Integer): HICON;
var
  colors: array of DWord;
  masks: array of Byte;
  bmColor, bmMask: HBITMAP;
  ii: TIconInfo_;
begin
  if sizePx < 8 then sizePx := 8;
  SetLength(colors, sizePx * sizePx);
  Render(colors, sizePx, MakeRgb(0, 255, 0), True, MakeRgb(0, 0, 0), True);
  SetLength(masks, ((sizePx + 15) div 16) * 2 * sizePx);
  FillChar(masks[0], Length(masks), 0);
  bmColor := w_CreateBitmap(sizePx, sizePx, 1, 32, @colors[0]);
  bmMask := w_CreateBitmap(sizePx, sizePx, 1, 1, @masks[0]);
  Result := 0;
  if (bmColor <> 0) and (bmMask <> 0) then
  begin
    FillChar(ii, SizeOf(ii), 0);
    ii.fIcon := True;
    ii.hbmMask := bmMask;
    ii.hbmColor := bmColor;
    Result := w_CreateIconIndirect(@ii);
  end;
  if bmColor <> 0 then DeleteObject(bmColor);
  if bmMask <> 0 then DeleteObject(bmMask);
end;

{ ================= Window ================= }

procedure SetShown(i: Integer; const s: UnicodeString);
begin
  if (gLbls[i] <> 0) and (gShown[i] <> s) then
  begin
    gShown[i] := s;
    w_SetWindowTextW(gLbls[i], PWideChar(gShown[i]));
  end;
end;

function OrDash(const s: UnicodeString): UnicodeString;
begin
  if s = '' then Result := '-' else Result := s;
end;

procedure RefreshInfo;
var
  inf: TCardInfo;
  st: UnicodeString;
begin
  if (gWnd = 0) or (not Assigned(gGetInfo)) then Exit;
  inf := gGetInfo();
  if inf.Linked then
  begin
    st := FormConnected;
    gStatusColor := CLR_GREEN;
  end
  else if inf.Present then
  begin
    st := FormUsbReady;
    gStatusColor := CLR_ORANGE;
  end
  else
  begin
    st := FormCardNotFound;
    gStatusColor := CLR_RED;
  end;
  SetShown(0, st);
  SetShown(1, FormatText(FormPortFormat, [OrDash(inf.PortName), UnicodeString(IntToStr(inf.Baud))]));
  if gLamp <> nil then
  begin
    if inf.HasFb then gLamp.ApplyCardCode(inf.Lamp)
    else gLamp.ApplyCardCode(255);
    if (gLampWnd <> 0) and gLamp.NeedsRedraw then
      w_InvalidateRect(gLampWnd, nil, False);
  end;
end;

procedure HideWin;
begin
  if gWnd <> 0 then ShowWindow(gWnd, SW_HIDE);
end;

procedure MakeBrushes;
begin
  if gBrBackground <> 0 then Exit;
  gBrBackground := CreateSolidBrush(CLR_BACKGROUND);
  gBrSurface := CreateSolidBrush(CLR_SURFACE);
  gBrBorder := CreateSolidBrush(CLR_BORDER);
end;

procedure FreeBrushes;
begin
  if gBrBackground <> 0 then begin DeleteObject(gBrBackground); gBrBackground := 0; end;
  if gBrSurface <> 0 then begin DeleteObject(gBrSurface); gBrSurface := 0; end;
  if gBrBorder <> 0 then begin DeleteObject(gBrBorder); gBrBorder := 0; end;
end;

procedure DarkTitle(wnd: HWND);
var
  v: DWord;
begin
  try
    v := 1;
    if DwmSetWindowAttribute_(wnd, 20, @v, 4) <> 0 then
      DwmSetWindowAttribute_(wnd, 19, @v, 4);
    v := $00331B0E; DwmSetWindowAttribute_(wnd, 35, @v, 4);
    v := CLR_TEXT;  DwmSetWindowAttribute_(wnd, 36, @v, 4);
    v := $00784426; DwmSetWindowAttribute_(wnd, 34, @v, 4);
  except
  end;
end;

procedure PaintPanel(dc: HDC; x, y, w, h: Integer; const txt: UnicodeString);
var
  r: TRect;
  oldF: HGDIOBJ;
begin
  r.Left := x;
  r.Top := y;
  r.Right := x + w;
  r.Bottom := y + h;
  w_FillRect(dc, @r, gBrSurface);
  w_FrameRect(dc, @r, gBrBorder);
  SetBkMode(dc, TRANSPARENT);
  SetTextColor(dc, CLR_ACCENT);
  oldF := SelectObject(dc, gFontB);
  Inc(r.Left, 10);
  Inc(r.Top, 4);
  w_DrawTextW(dc, PWideChar(txt), -1, @r, DT_SINGLELINE_ or DT_NOPREFIX_);
  SelectObject(dc, oldF);
end;

procedure PaintWin(wnd: HWND);
var
  ps: TPaintStruct_;
  dc: HDC;
begin
  dc := w_BeginPaint(wnd, @ps);
  try
    PaintPanel(dc, PNL_X, PNL_PROP_Y, PNL_W, PNL_PROP_H, FormCardStatus);
  finally
    w_EndPaint(wnd, @ps);
  end;
end;

{ Window procedure of the status window }
function CardWndProc(wnd: HWND; msg: UINT; wp: WPARAM; lp: LPARAM): LRESULT; stdcall;
var
  di: PDrawItemStruct_;
begin
  try
    case msg of
      WM_CLOSE:
        begin
          HideWin;
          Exit(0);
        end;
      WM_TIMER:
        begin
          if (wp = ID_TIMER) and (gLamp <> nil) then
          begin
            gLamp.Tick;
            if (gLampWnd <> 0) and gLamp.NeedsRedraw then
              w_InvalidateRect(gLampWnd, nil, False);
          end;
          Exit(0);
        end;
      WM_PAINT:
        begin
          PaintWin(wnd);
          Exit(0);
        end;
      WM_DRAWITEM:
        begin
          di := PDrawItemStruct_(lp);
          if (di <> nil) and (di^.CtlID = ID_LAMP) and (gLamp <> nil) then
            gLamp.Draw(di^.dcItem, di^.rcItem.Left, di^.rcItem.Top, CLR_SURFACE);
          Exit(1);
        end;
      WM_CTLCOLORSTATIC:
        begin
          if HWND(lp) = gLbls[0] then SetTextColor(HDC(wp), gStatusColor)
          else SetTextColor(HDC(wp), CLR_TEXT);
          SetBkColor(HDC(wp), CLR_SURFACE);
          Exit(LRESULT(gBrSurface));
        end;
    end;
  except
  end;
  Result := w_DefWindowProcW(wnd, msg, wp, lp);
end;

function MkChild(exStyle: DWord; const cls, txt: UnicodeString; style: DWord;
  x, y, w, h, id: Integer; font: HFONT): HWND;
begin
  Result := w_CreateWindowExW(exStyle, PWideChar(cls), PWideChar(txt),
    WS_CHILD or WS_VISIBLE or WS_CLIPSIBLINGS or style,
    x, y, w, h, gWnd, HMENU(id), HInstance, nil);
  if (Result <> 0) and (font <> 0) then
    w_SendMessageW(Result, WM_SETFONT, WPARAM(font), 1);
end;

procedure SetupIcons(wnd: HWND);
begin
  if gIconSmall = 0 then gIconSmall := TVirtualLamp.CreateLitIcon(GetSystemMetrics(SM_CXSMICON_));
  if gIconBig = 0 then gIconBig := TVirtualLamp.CreateLitIcon(GetSystemMetrics(SM_CXICON_));
  if gIconSmall <> 0 then w_SendMessageW(wnd, WM_SETICON, ICON_SMALL_, LPARAM(gIconSmall));
  if gIconBig <> 0 then w_SendMessageW(wnd, WM_SETICON, ICON_BIG_, LPARAM(gIconBig));
end;

procedure MakeFonts;
begin
  if gFontN = 0 then
    gFontN := w_CreateFontW(-12, 0, 0, 0, 400, 0, 0, 0, 1, 0, 0, 0, 0, PWideChar(FONT_FACE));
  if gFontB = 0 then
    gFontB := w_CreateFontW(-12, 0, 0, 0, 700, 0, 0, 0, 1, 0, 0, 0, 0, PWideChar(FONT_FACE));
end;

procedure FreeIcons;
begin
  if gIconSmall <> 0 then begin DestroyIcon(gIconSmall); gIconSmall := 0; end;
  if gIconBig <> 0 then begin DestroyIcon(gIconBig); gIconBig := 0; end;
end;

{ Creates the status window and its controls. }
procedure CreateCardWindow(owner: HWND);
var
  wc: TWndClassW_;
  style: DWord;
  r, mainWnd: TRect;
  posX, posY, i: Integer;
begin
  MakeBrushes;
  if not gClassReady then
  begin
    FillChar(wc, SizeOf(wc), 0);
    wc.lpfnWndProc := @CardWndProc;
    wc.hInstanceW := HInstance;
    wc.hCursorW := w_LoadCursorW(0, PWideChar(PtrUInt(IDC_ARROW_)));
    wc.hbrBackground := gBrBackground;
    wc.lpszClassName := PWideChar(WND_CLASS);
    if w_RegisterClassW(@wc) = 0 then Exit;
    gClassReady := True;
  end;
  MakeFonts;
  style := WS_CAPTION or WS_SYSMENU or WS_CLIPCHILDREN;
  r.Left := 0;
  r.Top := 0;
  r.Right := CLIENT_W;
  r.Bottom := CLIENT_H;
  w_AdjustWindowRectEx(@r, style, False, 0);
  posX := 100;
  posY := 100;
  if (owner <> 0) and Boolean(w_GetWindowRect(owner, @mainWnd)) then
  begin
    posX := mainWnd.Left + 200;
    posY := mainWnd.Top + 100;
  end;
  gWnd := w_CreateWindowExW(0, PWideChar(WND_CLASS), PWideChar(CardFormTitle), style,
    posX, posY, r.Right - r.Left, r.Bottom - r.Top, owner, 0, HInstance, nil);
  if gWnd = 0 then Exit;
  DarkTitle(gWnd);
  SetupIcons(gWnd);
  if gLamp = nil then gLamp := TVirtualLamp.Create(LAMP_SIZE);
  gLampWnd := MkChild(0, CLS_STATIC, '', SS_OWNERDRAW_, 16, PNL_PROP_Y + 24, LAMP_SIZE, LAMP_SIZE, ID_LAMP, 0);
  gLbls[0] := MkChild(0, CLS_STATIC, '', 0, 44, PNL_PROP_Y + 26, 312, 18, 0, gFontB);
  gLbls[1] := MkChild(0, CLS_STATIC, '', 0, 16, PNL_PROP_Y + 56, 340, 18, 0, gFontN);
  w_SetTimer(gWnd, ID_TIMER, LAMP_TICK_MS, nil);
  for i := 0 to LINE_COUNT - 1 do gShown[i] := #1;
end;

procedure KillChildren;
var
  i: Integer;
begin
  for i := 0 to LINE_COUNT - 1 do gLbls[i] := 0;
  gLampWnd := 0;
end;

procedure ShowCardWindow(owner: HWND; getInfo: TCardInfoFn);
var
  i: Integer;
begin
  gGetInfo := getInfo;
  if gWnd = 0 then CreateCardWindow(owner);
  if gWnd = 0 then Exit;
  if not IsWindowVisible(gWnd) then
  begin
    for i := 0 to LINE_COUNT - 1 do gShown[i] := #1;
    RefreshInfo;
    ShowWindow(gWnd, SW_SHOWNORMAL);
  end;
  SetForegroundWindow(gWnd);
end;

procedure UpdateCardWindow;
var
  t: QWord;
begin
  if (gWnd = 0) or (not IsWindowVisible(gWnd)) then Exit;
  t := TickMs64;
  if t - gLastRefresh < REFRESH_MS then Exit;
  gLastRefresh := t;
  RefreshInfo;
end;

procedure FreeCardWindow;
begin
  CloseSpeedWarning;
  if gWnd <> 0 then
  begin
    w_KillTimer(gWnd, ID_TIMER);
    DestroyWindow(gWnd);
    gWnd := 0;
  end;
  FreeAndNil(gLamp);
  KillChildren;
  FreeIcons;
  if gFontN <> 0 then begin DeleteObject(gFontN); gFontN := 0; end;
  if gFontB <> 0 then begin DeleteObject(gFontB); gFontB := 0; end;
  if gClassReady then
  begin
    w_UnregisterClassW(PWideChar(WND_CLASS), HInstance);
    gClassReady := False;
  end;
  if gWarnClassReady then
  begin
    w_UnregisterClassW(PWideChar(W_CLASS), HInstance);
    gWarnClassReady := False;
  end;
  if gBrAccent <> 0 then begin DeleteObject(gBrAccent); gBrAccent := 0; end;
  FreeBrushes;
  gGetInfo := nil;
end;

{ ================= Axis velocity warning ================= }

procedure PaintWarning(wnd: HWND);
var
  ps: TPaintStruct_;
  dc: HDC;
  r: TRect;
  oldF: HGDIOBJ;
begin
  dc := w_BeginPaint(wnd, @ps);
  try
    PaintPanel(dc, PNL_X, PNL_PROP_Y, W_CLIENT_W - 2 * PNL_X, W_TEXT_Y + gWarnTextH + 12, SpeedPanel);
    r.Left := PNL_X + 10;
    r.Top := PNL_PROP_Y + W_TEXT_Y;
    r.Right := W_CLIENT_W - PNL_X - 10;
    r.Bottom := r.Top + gWarnTextH;
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, CLR_TEXT);
    oldF := SelectObject(dc, gFontN);
    w_DrawTextW(dc, PWideChar(gWarnText), -1, @r, DT_WORDBREAK_ or DT_NOPREFIX_);
    SelectObject(dc, oldF);
  finally
    w_EndPaint(wnd, @ps);
  end;
end;

{ Themed OK button: surface colour, border colour while pressed, accent frame. }
procedure PaintButton(di: PDrawItemStruct_);
var
  r: TRect;
  oldF: HGDIOBJ;
begin
  r := di^.rcItem;
  if (di^.itemState and ODS_SELECTED_) <> 0 then
    w_FillRect(di^.dcItem, @r, gBrBorder)
  else
    w_FillRect(di^.dcItem, @r, gBrSurface);
  w_FrameRect(di^.dcItem, @r, gBrAccent);
  SetBkMode(di^.dcItem, TRANSPARENT);
  SetTextColor(di^.dcItem, CLR_TEXT);
  oldF := SelectObject(di^.dcItem, gFontB);
  w_DrawTextW(di^.dcItem, PWideChar(SpeedButtonOk), -1, @r,
    DT_CENTER_ or DT_VCENTER_ or DT_SINGLELINE_ or DT_NOPREFIX_);
  SelectObject(di^.dcItem, oldF);
end;

{ Window procedure of the warning window: OK or close closes it. }
function WarningWndProc(wnd: HWND; msg: UINT; wp: WPARAM; lp: LPARAM): LRESULT; stdcall;
var
  di: PDrawItemStruct_;
begin
  try
    case msg of
      WM_CLOSE:
        begin
          CloseSpeedWarning;
          Exit(0);
        end;
      WM_PAINT:
        begin
          PaintWarning(wnd);
          Exit(0);
        end;
      WM_DRAWITEM:
        begin
          di := PDrawItemStruct_(lp);
          if (di <> nil) and (di^.CtlType = ODT_BUTTON_) then
          begin
            PaintButton(di);
            Exit(1);
          end;
        end;
      WM_COMMAND:
        begin
          if ((LongInt(wp) and $FFFF) = ID_OK) and
             (((LongInt(wp) shr 16) and $FFFF) = BN_CLICKED_) then
            CloseSpeedWarning;
          Exit(0);
        end;
    end;
  except
  end;
  Result := w_DefWindowProcW(wnd, msg, wp, lp);
end;

procedure ShowSpeedWarning(const txt: UnicodeString);
var
  wc: TWndClassW_;
  style: DWord;
  r: TRect;
  dc: HDC;
  oldF: HGDIOBJ;
  w, h, btnY: Integer;
begin
  if gWarnWnd <> 0 then Exit;
  gWarnText := txt;
  MakeBrushes;
  MakeFonts;
  if gBrAccent = 0 then gBrAccent := CreateSolidBrush(CLR_ACCENT);
  if not gWarnClassReady then
  begin
    FillChar(wc, SizeOf(wc), 0);
    wc.lpfnWndProc := @WarningWndProc;
    wc.hInstanceW := HInstance;
    wc.hCursorW := w_LoadCursorW(0, PWideChar(PtrUInt(IDC_ARROW_)));
    wc.hbrBackground := gBrBackground;
    wc.lpszClassName := PWideChar(W_CLASS);
    if w_RegisterClassW(@wc) <> 0 then gWarnClassReady := True;
  end;
  { Text height; the window size follows it }
  r.Left := 0;
  r.Top := 0;
  r.Right := W_CLIENT_W - 2 * PNL_X - 20;
  r.Bottom := 0;
  dc := w_GetDC(0);
  oldF := SelectObject(dc, gFontN);
  w_DrawTextW(dc, PWideChar(gWarnText), -1, @r, DT_CALCRECT_ or DT_WORDBREAK_ or DT_NOPREFIX_);
  SelectObject(dc, oldF);
  w_ReleaseDC(0, dc);
  gWarnTextH := r.Bottom - r.Top;
  btnY := PNL_PROP_Y + W_TEXT_Y + gWarnTextH + 12 + 10;
  style := WS_CAPTION or WS_SYSMENU or WS_CLIPCHILDREN;
  r.Left := 0;
  r.Top := 0;
  r.Right := W_CLIENT_W;
  r.Bottom := btnY + BTN_H + 10;
  w_AdjustWindowRectEx(@r, style, False, WS_EX_TOPMOST_);
  w := r.Right - r.Left;
  h := r.Bottom - r.Top;
  if gWarnClassReady then
    gWarnWnd := w_CreateWindowExW(WS_EX_TOPMOST_, PWideChar(W_CLASS), PWideChar(SpeedTitle),
      style, (GetSystemMetrics(SM_CXSCREEN_) - w) div 2, (GetSystemMetrics(SM_CYSCREEN_) - h) div 2,
      w, h, 0, 0, HInstance, nil);
  if gWarnWnd = 0 then
  begin
    { The window could not be created: show the same warning in a Windows message box. }
    w_MessageBoxW(0, PWideChar(SpeedPanel + #13#10#13#10 + gWarnText),
      PWideChar(SpeedTitle), MB_OK or MB_ICONWARNING or MB_TOPMOST_);
    Exit;
  end;
  DarkTitle(gWarnWnd);
  SetupIcons(gWarnWnd);
  w_CreateWindowExW(0, PWideChar(CLS_BUTTON), nil,
    WS_CHILD or WS_VISIBLE or WS_TABSTOP or BS_OWNERDRAW_,
    W_CLIENT_W - PNL_X - BTN_W, btnY, BTN_W, BTN_H,
    gWarnWnd, HMENU(ID_OK), HInstance, nil);
  ShowWindow(gWarnWnd, SW_SHOWNORMAL);
  SetForegroundWindow(gWarnWnd);
end;

function SpeedWarningOpen: Boolean;
begin
  Result := gWarnWnd <> 0;
end;

procedure CloseSpeedWarning;
var
  wnd: HWND;
begin
  wnd := gWarnWnd;
  gWarnWnd := 0;
  if wnd <> 0 then DestroyWindow(wnd);
end;

end.
