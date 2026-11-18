{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uDevice.pas - top-level controller
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  Connects the Mach3 events (motion, jog, dwell, probe, reset) to the card.
  Reads the planned moves from the Mach3 trajectory buffer and hands them to
  TCardLayer as 5 ms slices.
}
unit uDevice;

{$mode objfpc}{$H+}
{$codepage utf8}
{$R-}{$Q-}

interface

uses
  Windows, SysUtils, Math, uWinApi, uMessages, uCardProtocol, uMainProgram, uCardLayer;

const
  JOG_POSITIVE_DIR = 0;
  JOG_SPEED_DIVISOR = 1.0;
  JOG_FALLBACK_SPEED = 1000.0;
  JOG_TIMER_MS = 10;
  SINGLE_AXIS_JOG = False;

type
  TDevice = class
  private
    FHost: TMainProgram;
    FCard: TCardLayer;
    FTrajCounter: DWord;
    FJogActive: array[0..AXIS_COUNT - 1] of Boolean;
    FJogSign: array[0..AXIS_COUNT - 1] of Integer;
    FJogRate: array[0..AXIS_COUNT - 1] of Double;
    FJogTrajMark: DWord;
    FLastTick: QWord;
    FKeyCount: array[0..AXIS_COUNT - 1] of Integer;
    FJogLock: Boolean;
    FFocusLock: Boolean;
    FJogTickAt: QWord;
    FWasReset: Boolean;
    FStuckSince: QWord;
    FDwellActive: Boolean;
    FDwellStart: QWord;
    FDwellMs: QWord;
    FStartupStep: Integer;
    FStepIdx: TAxisReal;
    FRates: TAxisReal;
    FFlagPending: array[0..6] of Boolean;
    FNoKeySince: QWord;

    procedure StartupService;
    procedure SendHoldingMovement;
    procedure StartJogTimer;
    procedure StopJogTimer;
    procedure IntegrateJog;
    procedure ClearJogFlags(axis: Integer);
    procedure ClearPendingFlags(immediate: Boolean);
    function JogInProgress: Boolean;
    procedure CheckFocus;
    procedure StopAllJog;
    function AnyKeyDown: Boolean;
    procedure ServiceDwell;
    function SequenceBusy: Boolean;
    procedure DiscardMotion;
    procedure CheckJogLock;
    procedure AbortJog;
    procedure CheckJogKeys;
    procedure CheckStuckJog;
  public
    constructor Create;
    destructor Destroy; override;

    procedure InitControl;
    procedure PostInitControl;
    procedure Shutdown;
    procedure ShowForm(owner: HWND);
    procedure UpdateForm;
    procedure FreeForm;
    procedure Update;
    procedure JogOn(axis, direction: SmallInt; speedNorm: Double);
    procedure JogTick;
    procedure JogOff(axis: SmallInt);
    procedure Dwell(time: Double);
    procedure Probe;
    procedure Home(axis: SmallInt);
    procedure Purge(flags: SmallInt);
    procedure Reset;
    function WindowInteraction: Boolean;
    function CheckWindowMoveKeys: Boolean;
  end;

implementation

var
  gJogDevice: TDevice = nil;
  gJogTimerId: UINT_PTR = 0;

{ 10 ms Windows timer for smooth, lag-free motion while jogging. }
procedure JogTimerProc(wnd: HWND; msg: UINT; id: UINT_PTR; time: DWORD); stdcall;
begin
  try
    if gJogDevice <> nil then gJogDevice.JogTick;
  except
  end;
end;

{ True if the keyboard focus is in a Mach3 window. }
function ProgramInForeground: Boolean;
var
  wnd: HWND;
  pid: DWORD;
begin
  Result := False;
  wnd := GetForegroundWindow;
  if wnd = 0 then Exit;
  pid := 0;
  w_GetWindowThreadProcessId(wnd, @pid);
  Result := pid = GetCurrentProcessId;
end;

{ True if any keyboard key is held down. }
function KeyboardKeyDown: Boolean;
var
  k: Integer;
begin
  Result := False;
  for k := 8 to 254 do
    if GetAsyncKeyState(k) < 0 then Exit(True);
end;

constructor TDevice.Create;
begin
  inherited Create;
  FLastTick := TickMs64;
  FStartupStep := -1;
  FHost := TMainProgram.Create;
  FCard := TCardLayer.Create(FHost);
  gJogDevice := Self;
end;

destructor TDevice.Destroy;
begin
  StopJogTimer;
  if gJogDevice = Self then gJogDevice := nil;
  FCard.Free;
  FHost.Free;
  inherited Destroy;
end;

procedure TDevice.InitControl;
begin
  SetCBool(ViewPrinterOn, False);
end;

procedure TDevice.PostInitControl;
begin
  FStartupStep := 0;
  FCard.Startup;
end;

{ Runs once after start-up: puts Mach3 into reset (E-Stop).
  The coordinates are not zeroed at start-up. }
procedure TDevice.StartupService;
begin
  if FStartupStep < 0 then Exit;
  Inc(FStartupStep);
  if FStartupStep = 1 then
  begin
    if not ProgramInReset then FHost.PressResetButton;
    FStartupStep := -1;
  end;
end;

procedure TDevice.Shutdown;
begin
  StopJogTimer;
  FCard.Shutdown;
end;

procedure TDevice.ShowForm(owner: HWND);
begin
  FCard.ShowForm(owner);
end;

procedure TDevice.UpdateForm;
begin
  FCard.UpdateForm;
end;

procedure TDevice.FreeForm;
begin
  FCard.FreeForm;
end;

procedure TDevice.Update;
begin
  ServiceDwell;
  StartupService;
  CheckJogLock;
  if ProgramSimMode then Exit;
  if ProgramInReset then
  begin
    if not FWasReset then
    begin
      FWasReset := True;
      StopAllJog;
      ClearPendingFlags(True);
      DiscardMotion;
      if AnyKeyDown then FJogLock := True;
    end;
    FCard.UpdatePosition;
    Exit;
  end;
  FWasReset := False;
  CheckFocus;
  CheckJogKeys;
  SendHoldingMovement;
  if (gJogTimerId <> 0) and FCard.FastJog and (TickMs64 - FJogTickAt > 100) then
    FCard.FastJog := False;
  IntegrateJog;
  FCard.UpdatePosition;
  ClearPendingFlags(False);
  CheckStuckJog;
end;

{ Moves new trajectory points from the Mach3 buffer to the card for as long as
  the card buffer has room. }
procedure TDevice.SendHoldingMovement;
var
  idx: Integer;
  trajPoint: TGMoves;
begin
  { A move that Mach3 plans before the jog has finished (deceleration included)
    starts from the wrong position: Mach3 does not know the position while
    jogging. Such points are not sent to the card; they are discarded and the
    user is warned. When the jog has ended Mach3 syncs the position, and the
    command is planned correctly when it is given again. }
  if JogInProgress then
  begin
    if EngTrajHead^ <> EngTrajIndex^ then
    begin
      EngTrajIndex^ := EngTrajHead^;
      FCard.Purge;
      FHost.GcodeFinished;
      { When Mach3 starts a move it drops the jog on its own side but sends no
        JogOff. The jog is stopped here instead; the keys must be released
        before a new jog can start. }
      AbortJog;
      FJogLock := True;
      StatusMessage(MsgMoveDuringJog);
    end;
    Exit;
  end;
  while (EngTrajHead^ <> EngTrajIndex^) and FCard.CanAccept do
  begin
    Inc(FTrajCounter);
    idx := EngTrajIndex^ and $FFF;
    EngDisplayLine^ := EngTrajID(idx)^;
    trajPoint := PlnMovement(idx)^;
    FHost.MoveSteps(trajPoint, FStepIdx);
    FCard.GcodeSlice(FStepIdx);
    EngTrajIndex^ := (idx + 1) and $FFF;
  end;
  if (EngTrajHead^ = EngTrajIndex^) and CBool(PlnBufferEnding) then
    FCard.FinalizeMoves;
end;

{ Starts a jog on one axis. Jogging is refused in reset, without a card, or when
  the opposite direction keys of the same axis are pressed together. }
procedure TDevice.JogOn(axis, direction: SmallInt; speedNorm: Double);
var
  newDir, i: Integer;
  rate: Double;
begin
  if ProgramSimMode then Exit;
  if (axis >= 0) and (axis < AXIS_COUNT) and (FKeyCount[axis] < 4) then Inc(FKeyCount[axis]);
  if ProgramInReset then
  begin
    FJogLock := True;
    StatusMessage(MsgNoJogInReset);
    Exit;
  end;
  if not FCard.Linked then
  begin
    FJogLock := True;
    StatusMessage(MsgCardNotConnected);
    Exit;
  end;
  if (axis < 0) or (axis >= AXIS_COUNT) then Exit;
  if FJogLock then Exit;
  if direction = JOG_POSITIVE_DIR then newDir := 1 else newDir := -1;
  if FJogActive[axis] and (FJogSign[axis] <> newDir) then
  begin
    FJogLock := True;
    StopAllJog;
    StatusMessage(MsgOppositeKeys, MSG_OPPOSITE_KEYS);
    Exit;
  end;
  if (Engine <> nil) and (EngPtr(ENG_JOG_BUSY)^ <> 0) then Exit;
  if EngTrajHead^ <> EngTrajIndex^ then Exit;
  if SINGLE_AXIS_JOG then
  begin
    for i := 0 to AXIS_COUNT - 1 do
      if (i <> axis) and FJogActive[i] then Exit;
  end;
  if (speedNorm <= 0) or IsNan(speedNorm) then speedNorm := 1.0;
  if speedNorm > 1.0 then speedNorm := speedNorm / 100.0;
  rate := speedNorm * PlnVelocity(axis)^ * PlnStepsPerAxis(axis)^ / JOG_SPEED_DIVISOR;
  if IsNan(rate) or IsInfinite(rate) or (rate < 1.0) then
    rate := JOG_FALLBACK_SPEED * speedNorm;
  FJogSign[axis] := newDir;
  FJogRate[axis] := rate;
  FJogTrajMark := FTrajCounter;
  FJogActive[axis] := True;
  { Tell Mach3 that the external device is moving: Mach3 treats the device as
    "busy" until the jog has ended and the position is synced. }
  if MainPlanner <> nil then SetCBool(PlnExternalStill, False);
  StartJogTimer;
  IntegrateJog;
end;

procedure TDevice.StartJogTimer;
begin
  if gJogTimerId = 0 then
  begin
    gJogTimerId := w_SetTimer(0, 0, JOG_TIMER_MS, @JogTimerProc);
    FJogTickAt := TickMs64;
  end;
  FCard.FastJog := gJogTimerId <> 0;
end;

procedure TDevice.StopJogTimer;
begin
  if gJogTimerId <> 0 then
  begin
    w_KillTimer(0, gJogTimerId);
    gJogTimerId := 0;
  end;
  if FCard <> nil then FCard.FastJog := False;
end;

procedure TDevice.JogTick;
var
  pressed: Boolean;
  i: Integer;
begin
  FJogTickAt := TickMs64;
  if ProgramSimMode or ProgramInReset then
  begin
    StopJogTimer;
    Exit;
  end;
  CheckFocus;
  CheckJogKeys;
  pressed := False;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then pressed := True;
  if pressed then IntegrateJog
  else FCard.JogService;
  { The hand-over to Mach3 (sync) is NOT done here: the final step count is written
    to Mach3 only in Update, and the hand-over follows right after that write. }
  if (not pressed) and (not FCard.JogBusy) then StopJogTimer;
end;

{ Converts the jog speed of every active axis into steps for the elapsed time. }
procedure TDevice.IntegrateJog;
var
  current: QWord;
  dt: Double;
  any: Boolean;
  i: Integer;
begin
  current := TickMs64;
  dt := (current - FLastTick) / 1000.0;
  FLastTick := current;
  if dt > 0.5 then dt := 0.5;
  if FJogTrajMark <> FTrajCounter then
  begin
    FJogTrajMark := FTrajCounter;
    Exit;
  end;
  any := False;
  for i := 0 to AXIS_COUNT - 1 do
  begin
    if FJogActive[i] then
    begin
      FRates[i] := FJogSign[i] * FJogRate[i];
      any := True;
    end
    else
      FRates[i] := 0;
  end;
  if any then FCard.JogAdvance(FRates, dt);
end;

{ A jog key was released: stops the axis and clears the Mach3 jog flags. }
procedure TDevice.JogOff(axis: SmallInt);
begin
  if (axis >= 0) and (axis < AXIS_COUNT) and (FKeyCount[axis] > 0) then Dec(FKeyCount[axis]);
  if not AnyKeyDown then
  begin
    FJogLock := False;
    FFocusLock := False;
    ClearStatusMessage(MSG_OPPOSITE_KEYS);
  end;
  if (axis < 0) or (axis > 6) then Exit;
  if axis < AXIS_COUNT then FJogActive[axis] := False;
  if not (FJogActive[0] or FJogActive[1] or FJogActive[2] or FJogActive[3]) then
  begin
    FCard.JogStop;
    FCard.JogService;
  end;
  { Do not clear the flags yet: they are cleared when the machine has slowed
    down and stopped. }
  FFlagPending[axis] := True;
end;

{ Is a jog key held, is the machine still decelerating from a jog, or has Mach3
  not been told yet that the jog is over? }
function TDevice.JogInProgress: Boolean;
var
  i: Integer;
begin
  Result := True;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then Exit;
  for i := 0 to 6 do
    if FFlagPending[i] then Exit;
  if FCard.JogBusy then Exit;
  Result := False;
end;

{ Clears the pending jog flags when the jog has completely finished (or right away
  if immediate = True). This way Mach3 syncs the position only after the
  deceleration steps have been executed too. }
procedure TDevice.ClearPendingFlags(immediate: Boolean);
var
  i: Integer;
  pending: Boolean;
begin
  pending := False;
  for i := 0 to 6 do
    if FFlagPending[i] then pending := True;
  if not pending then Exit;
  if not immediate then
  begin
    for i := 0 to AXIS_COUNT - 1 do
      if FJogActive[i] then Exit;      { another axis is still jogging }
    if not FCard.MotionStopped then Exit;
  end;
  for i := 0 to 6 do
    if FFlagPending[i] then
    begin
      FFlagPending[i] := False;
      ClearJogFlags(i);
    end;
  { The jog is over and the position is synced: the external device is now at rest. }
  FHost.GcodeFinished;
end;

{ Tells Mach3 that the axis has finished jogging. }
procedure TDevice.ClearJogFlags(axis: Integer);
var
  counter, x: Integer;
begin
  SetCBool(EngAxisDec(axis), True);
  SetCBool(EngAxisJogging(axis), False);
  counter := 0;
  for x := 0 to 5 do
    if not CBool(EngAxisJogging(x)) then Inc(counter);
  if counter >= 6 then
  begin
    SetCBool(EngJogging, False);
    SetCBool(PlnJogging, False);
    SetCBool(EngSync, True);
  end;
end;

{ Safety: if Mach3 loses the keyboard focus during a jog, all jogging stops
  (the key release event would never arrive). }
procedure TDevice.CheckFocus;
var
  jogActive: Boolean;
  i: Integer;
begin
  if ProgramInForeground then
  begin
    if FFocusLock and (not KeyboardKeyDown) then
    begin
      FFocusLock := False;
      FJogLock := False;
      ClearStatusMessage(MSG_OPPOSITE_KEYS);
    end;
    Exit;
  end;
  jogActive := AnyKeyDown;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then jogActive := True;
  if not jogActive then Exit;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then FFlagPending[i] := True;   { hand-over happens in Update, when the machine has stopped }
  StopAllJog;
  FCard.JogService;
  FillChar(FKeyCount, SizeOf(FKeyCount), 0);
  FJogLock := True;
  FFocusLock := True;
end;

procedure TDevice.StopAllJog;
var
  i: Integer;
begin
  for i := 0 to AXIS_COUNT - 1 do FJogActive[i] := False;
  FCard.JogStop;
end;

function TDevice.AnyKeyDown: Boolean;
var
  i: Integer;
begin
  Result := False;
  for i := 0 to AXIS_COUNT - 1 do
    if FKeyCount[i] > 0 then Exit(True);
end;

{ Starts a G4 dwell. Mach3 may give the time in seconds or in milliseconds. }
procedure TDevice.Dwell(time: Double);
var
  ms: Double;
begin
  if (Engine = nil) or (MainPlanner = nil) then Exit;
  ms := time;
  if PlnPtr(PLN_DWELL_IN_MS)^ = 0 then ms := ms * 1000.0;
  if (ms < 0) or IsNan(ms) then ms := 0;
  if ms > 4294967295.0 then ms := 4294967295.0;
  FDwellMs := QWord(Trunc(ms));
  FDwellStart := TickMs64;
  FDwellActive := True;
  PLongInt(EngPtr(ENG_DWELL_FLAG))^ := 1;
end;

procedure TDevice.ServiceDwell;
begin
  if not FDwellActive then Exit;
  if TickMs64 - FDwellStart > FDwellMs then
  begin
    FDwellActive := False;
    PLongInt(EngPtr(ENG_DWELL_FLAG))^ := 0;
  end;
end;

function TDevice.SequenceBusy: Boolean;
begin
  Result := FDwellActive or FJogActive[0] or FJogActive[1] or FJogActive[2] or FJogActive[3];
end;

{ G31 probe request: checks that the probe is enabled and the machine is ready. }
procedure TDevice.Probe;
begin
  if Engine = nil then Exit;
  if EngPtr(ENG_PROBE_ON)^ = 0 then
  begin
    StatusMessage(MsgProbeNotEnabled);
    PLongInt(EngPtr(ENG_PROBE_STATE))^ := 0;
    if MainPlanner <> nil then PLongInt(PlnPtr(PLN_PROBE_FLAG))^ := 0;
    Exit;
  end;
  if ProgramSimMode then Exit;
  if ProgramInReset then
  begin
    StatusMessage(MsgNoProbeInReset);
    Exit;
  end;
  if SequenceBusy then
    StatusMessage(MsgProbeWait);
  { The probing move itself is not executed by the card in this version. }
end;

procedure TDevice.Home(axis: SmallInt);
begin
  { Homing is not executed by the card in this version. }
end;

{ Stops the jog on every active axis; the hand-over to Mach3 happens in Update,
  when the machine has stopped. }
procedure TDevice.AbortJog;
var
  i: Integer;
begin
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then
    begin
      FJogActive[i] := False;
      FFlagPending[i] := True;
    end;
  FCard.JogStop;
  FCard.JogService;
  FillChar(FKeyCount, SizeOf(FKeyCount), 0);
end;

{ The window frame was clicked while jogging (see MenuHook). The jog is stopped
  with deceleration and, as with opposite keys, stays locked until every jog
  key has been released. Returns True while a jog key is held or the jog is
  still running; the click is then ignored. }
function TDevice.WindowInteraction: Boolean;
var
  i: Integer;
  jogActive: Boolean;
begin
  jogActive := False;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then jogActive := True;
  Result := jogActive or FCard.JogBusy or AnyKeyDown;
  if not (jogActive or FCard.JogBusy) then Exit;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then
    begin
      FJogActive[i] := False;
      FFlagPending[i] := True;
    end;
  FCard.JogStop;
  FCard.JogStopRamp;
  if jogActive then
  begin
    FJogLock := True;
    StatusMessage(MsgJogWindowStop, MSG_OPPOSITE_KEYS);
  end;
end;

{ Called while the window is being moved or resized. If any keyboard key is
  held, stops the jog, locks it until all keys are released and returns True
  so that the move is cancelled. }
function TDevice.CheckWindowMoveKeys: Boolean;
begin
  Result := KeyboardKeyDown;
  if not Result then Exit;
  WindowInteraction;
  if not FJogLock then
  begin
    FJogLock := True;
    StatusMessage(MsgJogWindowMove, MSG_OPPOSITE_KEYS);
  end;
end;

{ A jog key pressed while a text box (MDI, DRO) has the focus can leave Mach3
  in the "jogging" state without a matching JogOn / JogOff, and Mach3 then
  refuses every new jog. If Mach3 still reports a jog while no jog is running,
  the machine is at rest and no key or mouse button has been held for 200 ms,
  the Mach3 jog state is cleared, just as a reset would do. }
procedure TDevice.CheckStuckJog;
const
  STUCK_JOG_MS = 200;
var
  i: Integer;
  stuck: Boolean;
  tNow: QWord;
begin
  if Engine = nil then Exit;
  stuck := CBool(EngJogging);
  for i := 0 to 5 do
    if CBool(EngAxisJogging(i)) then stuck := True;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then stuck := False;
  for i := 0 to 6 do
    if FFlagPending[i] then stuck := False;
  if stuck then
    if FCard.JogBusy or (not FCard.MotionStopped) or (EngTrajHead^ <> EngTrajIndex^) or
       KeyboardKeyDown or (GetAsyncKeyState(VK_LBUTTON) < 0) then stuck := False;
  if not stuck then
  begin
    FStuckSince := 0;
    Exit;
  end;
  tNow := TickMs64;
  if FStuckSince = 0 then
  begin
    FStuckSince := tNow;
    Exit;
  end;
  if tNow - FStuckSince < STUCK_JOG_MS then Exit;
  FStuckSince := 0;
  for i := 0 to 5 do
    if CBool(EngAxisJogging(i)) then ClearJogFlags(i);
  if CBool(EngJogging) then
  begin
    SetCBool(EngJogging, False);
    SetCBool(PlnJogging, False);
    SetCBool(EngSync, True);
  end;
  FHost.GcodeFinished;
end;

{ SAFETY: do not rely on the JogOff notification from Mach3 alone. In some cases
  (e.g. Goto Zero during a jog) Mach3 sends no JogOff and the machine would run
  on forever. If no keyboard key and no mouse button has been held for 30 ms
  while a jog is active, the jog is stopped. }
procedure TDevice.CheckJogKeys;
const
  NO_KEY_MS = 30;
var
  i: Integer;
  anyActive, keyHeld: Boolean;
  tNow: QWord;
begin
  anyActive := False;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then anyActive := True;
  if not anyActive then
  begin
    FNoKeySince := 0;
    Exit;
  end;
  keyHeld := KeyboardKeyDown or (GetAsyncKeyState(VK_LBUTTON) < 0) or
    (GetAsyncKeyState(VK_RBUTTON) < 0) or (GetAsyncKeyState(VK_MBUTTON) < 0);
  if keyHeld then
  begin
    FNoKeySince := 0;
    Exit;
  end;
  tNow := TickMs64;
  if FNoKeySince = 0 then
  begin
    FNoKeySince := tNow;
    Exit;
  end;
  if tNow - FNoKeySince < NO_KEY_MS then Exit;
  FNoKeySince := 0;
  AbortJog;
end;

{ The jog lock normally opens when every JogOn has been matched by a JogOff. In
  some cases (e.g. Goto Zero started while a key is held) Mach3 sends no JogOff;
  the counter never reaches zero and the lock would stay forever. The real keyboard
  and mouse state is therefore checked too: when nothing is held, the lock and the
  counters are reset. }
procedure TDevice.CheckJogLock;
var
  i: Integer;
begin
  if not (FJogLock or FFocusLock or AnyKeyDown) then Exit;
  if not ProgramInForeground then Exit;          { without focus this is handled by CheckFocus }
  if KeyboardKeyDown then Exit;
  if (GetAsyncKeyState(VK_LBUTTON) < 0) or (GetAsyncKeyState(VK_RBUTTON) < 0) or
     (GetAsyncKeyState(VK_MBUTTON) < 0) then Exit;   { the on-screen jog button may be held with the mouse }
  for i := 0 to AXIS_COUNT - 1 do
    if FJogActive[i] then Exit;
  FillChar(FKeyCount, SizeOf(FKeyCount), 0);
  FJogLock := False;
  FFocusLock := False;
  ClearStatusMessage(MSG_OPPOSITE_KEYS);
end;

{ Purge, called by Mach3 to discard an interrupted move (Stop / E-Stop). }
procedure TDevice.Purge(flags: SmallInt);
begin
  DiscardMotion;
end;

{ Clears what is left of an interrupted move:
  1. resets the G-code state in the card layer,
  2. discards the points left in the Mach3 trajectory buffer (TrajIndex := TrajHead),
  3. tells Mach3 that the external device is at rest (ExternalStill := True).
  Without this, Mach3 waits for the old move to finish after an E-Stop and a
  new move (e.g. Goto Zero) never starts. }
procedure TDevice.DiscardMotion;
begin
  FCard.Purge;
  if Engine <> nil then EngTrajIndex^ := EngTrajHead^;
  FHost.GcodeFinished;
end;

{ Mach3 Reset button: stops jogging and dwell, then links or unlinks the card. }
procedure TDevice.Reset;
begin
  StopAllJog;
  if AnyKeyDown then FJogLock := True;
  if FDwellActive then
  begin
    FDwellActive := False;
    if Engine <> nil then PLongInt(EngPtr(ENG_DWELL_FLAG))^ := 0;
  end;
  FCard.ToggleConnection;
end;

end.
