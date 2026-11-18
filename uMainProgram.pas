{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uMainProgram.pas - access to Mach3 internals
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  The shared Engine and MainPlanner memory blocks, the Mach3 callback
  functions, the status line and the Ports & Pins signal tables.
  The plugin keeps no persistent settings and writes nothing to the registry.
  The ENG_ / PLN_ values are offsets inside the Engine and MainPlanner blocks
  of the 32-bit Mach3 release this plugin was written for.
}
unit uMainProgram;

{$mode objfpc}{$H+}
{$codepage utf8}
{$R-}{$Q-}

interface

uses
  Windows, SysUtils, Math, uWinApi, uMessages, uCardProtocol;

const
  AXIS_COUNT = 4;

  { ---- Mach3 memory layout ---- }
  EX_VMS = 1;
  ENG_TRAJ_STRIDE = 40;
  ENG_TRAJ_ID = 28;
  ENG_TRAJ_INDEX = 163840;
  ENG_TRAJ_HEAD = 163844;
  ENG_AXIS = 163920;
  ENG_AXIS_STRIDE = 96;
  AX_INDEX = 0;
  AX_DEC = 34;
  AX_JOGGING = 36;
  ENG_JOGGING = 163908;
  ENG_SYNC = 163910;
  ENG_DISPLAYLINE = 166500;
  PLN_MOVEMENTS = 1177752;
  GMOVES_SIZE = 288;
  GM_EX = 32;
  GM_SX = 80;
  PLN_JOGGING = 801965;
  PLN_STEPSPERAXIS = 586904;
  PLN_VELOCITIES = 586952;
  PLN_ACCELERATION = 587000;
  PLN_EXTERNALPULSERATES = 2384280;
  PLN_EXTERNALSTILL = 2384336;
  PLN_EXTERNALTYPE = 2384726;
  PLN_BUFFERENDING = 2745176;
  PLN_EXTIME = 2745184;
  PLN_EXBUFFERHI = 2745192;
  VIEW_PRINTERON = 133936;

  { ---- Reset, messages and signal tables ---- }
  RESET_REASON_WATCHDOG = 3;
  RESET_REASON_LIMIT = 4;
  RESET_REASON_CARD = 6;
  MSG_GENERAL = 0;
  MSG_CONNECTION = 1;
  MSG_OPPOSITE_KEYS = 2;
  ENG_RESET_FLAG = $289DF;
  ENG_RESET_REASON = $28B50;
  ENG_OVERRIDE_FLAG = $28B35;
  ENG_INSIG_BASE = $282F0;
  ENG_INSIG_COUNT = 53;
  ENG_INSIG_STRIDE = 16;
  ENG_LIMIT_SIGNALS = 18;
  ENG_OUTSIG_BASE = $28640;
  ENG_OUTSIG_COUNT = 30;
  ENG_OUTSIG_STRIDE = 5;
  ENG_SIM_MODE = $28D6E;
  ENG_PROBE_ON = $28450;
  ENG_PROBE_STATE = $28A5C;
  ENG_DWELL_FLAG = $28A4C;
  ENG_JOG_BUSY = $28044;
  PLN_EXT_DEVICE_FLAG = $246188;
  PLN_FLAG_29E376 = $29E376;
  PLN_FLAG_2462F0 = $2462F0;
  PLN_PROBE_FLAG = $11DDA0;
  PLN_DWELL_IN_MS = $11E03D;
  CARD_PORT = 3;
  OEM_RESET = 1021;

type
  { One trajectory point in the Mach3 planner buffer (288 bytes in 32-bit Mach3).
    ex..ec are the axis positions (steps). }
  TGMoves = packed record
    MoveType: LongInt;
    Pad0: LongInt;
    cx, cy, cz: Double;
    ex, ey, ez, ea, eb, ec: Double;
    sx, sy, sz, sa, sb, sc: Double;
    Pad1: array[0..143] of Byte;
    Time: Double;
    Stop: Byte;
    Pad2: array[0..6] of Byte;
  end;
  PGMoves = ^TGMoves;

  { Mach3 callback functions (cdecl) }
  TVoidShort = procedure(code: SmallInt); cdecl;
  TIntShort = function(count: SmallInt): Integer; cdecl;
  TDoubleShort = function(code: SmallInt): Double; cdecl;
  TVoidShortDouble = procedure(code: SmallInt; value: Double); cdecl;
  TBoolShort = function(code: SmallInt): ByteBool; cdecl;
  TVoidShortBool = procedure(code: SmallInt; value: ByteBool); cdecl;
  TVoidPChar = procedure(text: PAnsiChar); cdecl;

  { Methods that read and write the Mach3 state }
  TMainProgram = class
  public
    function StepUnit(ax: Integer): Double;
    function Accel(ax: Integer): Double;
    function HasPosition: Boolean;
    function ReadPosition(ax: Integer): Integer;
    procedure WritePosition(ax, val: Integer);
    function HasResetInfo: Boolean;
    function InReset: Boolean;
    procedure DoReset(cause: Integer);
    procedure PressResetButton;
    function OutputMask(out covered: Byte): Byte;
    function TransferInputs(inputBits: Byte): Boolean;
    function LimitStop: Boolean;
    function SpindlePWM: Byte;
    procedure GcodeFinished;
    procedure ConfigureDriver(kernelKHz, fifoHigh: Integer; sliceSec: Double);
    procedure ShowMessage(const msgText: UnicodeString);
    procedure TimedMessage(const msgText: UnicodeString; msgId: Integer; durationMs: DWord);
    procedure RemoveMessage(msgId: Integer);
    procedure MoveSteps(const trajPoint: TGMoves; var stepCount: array of Double);
  end;

var
  DoButton: TVoidShort = nil;
  GetMenuRange: TIntShort = nil;
  GetDRO: TDoubleShort = nil;
  SetDRO: TVoidShortDouble = nil;
  GetLED: TBoolShort = nil;
  SetLED: TVoidShortBool = nil;
  Code: TVoidPChar = nil;
  ProfileName: AnsiString = '';
  Engine: Pointer = nil;
  SetupBlock: Pointer = nil;
  MainPlanner: Pointer = nil;
  ViewBlock: Pointer = nil;

{ ---- Mach3 memory access ---- }
function LayoutValid: Boolean;
function OfsPtr(b: Pointer; ofs: PtrInt): PByte; inline;
function CBool(p: PByte): Boolean; inline;
procedure SetCBool(p: PByte; value: Boolean); inline;
function EngTrajIndex: PLongInt;
function EngTrajHead: PLongInt;
function EngDisplayLine: PLongInt;
function EngTrajID(idx: Integer): PLongInt;
function EngAxisIndex(axis: Integer): PLongInt;
function EngAxisDec(axis: Integer): PByte;
function EngAxisJogging(axis: Integer): PByte;
function EngJogging: PByte;
function EngSync: PByte;
function PlnMovement(idx: Integer): PGMoves;
function PlnExBufferHi: PLongInt;
function PlnExTime: PDouble;
function PlnExternalType: PSmallInt;
function PlnExternalPulseRate(axis: Integer): PDouble;
function PlnExternalStill: PByte;
function PlnBufferEnding: PByte;
function PlnJogging: PByte;
function PlnVelocity(axis: Integer): PDouble;
function PlnAcceleration(axis: Integer): PDouble;
function PlnStepsPerAxis(axis: Integer): PDouble;
function ViewPrinterOn: PByte;

{ ---- Status line and state queries ---- }
function EngPtr(ofs: Integer): PByte;
function PlnPtr(ofs: Integer): PByte;
function ProgramInReset: Boolean;
function ProgramSimMode: Boolean;
procedure StatusMessage(const msgText: UnicodeString; msgId: Integer = MSG_GENERAL;
  durationMs: DWord = 0);
procedure ClearStatusMessage(msgId: Integer);
procedure DeferStatusMessage(refresh: Integer);
procedure StatusMessageService;

implementation

var
  gLayoutOk: Boolean = False;
  gMsgId: Integer = MSG_GENERAL;
  gMsgEnd: QWord = 0;
  gMsgDefer: Integer = 0;
  gMsgWaiting: Boolean = False;
  gMsgPending: UnicodeString = '';
  gMsgPendingMs: DWord = 0;

{ ================= Mach3 memory access ================= }

function ComputeLayout: Boolean;
var
  g: TGMoves;
begin
  Result := (SizeOf(Pointer) = 4) and
    (SizeOf(TGMoves) = GMOVES_SIZE) and
    (PtrUInt(@g.ex) - PtrUInt(@g) = GM_EX) and
    (PtrUInt(@g.sx) - PtrUInt(@g) = GM_SX);
end;

function LayoutValid: Boolean;
begin
  Result := gLayoutOk;
end;

function OfsPtr(b: Pointer; ofs: PtrInt): PByte; inline;
begin
  Result := PByte(PtrUInt(b) + PtrUInt(ofs));
end;

function CBool(p: PByte): Boolean; inline;
begin
  Result := p^ <> 0;
end;

procedure SetCBool(p: PByte; value: Boolean); inline;
begin
  if value then p^ := 1 else p^ := 0;
end;

function EngTrajIndex: PLongInt;
begin
  Result := PLongInt(OfsPtr(Engine, ENG_TRAJ_INDEX));
end;

function EngTrajHead: PLongInt;
begin
  Result := PLongInt(OfsPtr(Engine, ENG_TRAJ_HEAD));
end;

function EngDisplayLine: PLongInt;
begin
  Result := PLongInt(OfsPtr(Engine, ENG_DISPLAYLINE));
end;

function EngTrajID(idx: Integer): PLongInt;
begin
  Result := PLongInt(OfsPtr(Engine, (idx and $FFF) * ENG_TRAJ_STRIDE + ENG_TRAJ_ID));
end;

function EngAxisIndex(axis: Integer): PLongInt;
begin
  Result := PLongInt(OfsPtr(Engine, ENG_AXIS + axis * ENG_AXIS_STRIDE + AX_INDEX));
end;

function EngAxisDec(axis: Integer): PByte;
begin
  Result := OfsPtr(Engine, ENG_AXIS + axis * ENG_AXIS_STRIDE + AX_DEC);
end;

function EngAxisJogging(axis: Integer): PByte;
begin
  Result := OfsPtr(Engine, ENG_AXIS + axis * ENG_AXIS_STRIDE + AX_JOGGING);
end;

function EngJogging: PByte;
begin
  Result := OfsPtr(Engine, ENG_JOGGING);
end;

function EngSync: PByte;
begin
  Result := OfsPtr(Engine, ENG_SYNC);
end;

function PlnMovement(idx: Integer): PGMoves;
begin
  Result := PGMoves(OfsPtr(MainPlanner, PLN_MOVEMENTS + (idx and $FFF) * GMOVES_SIZE));
end;

function PlnExBufferHi: PLongInt;
begin
  Result := PLongInt(OfsPtr(MainPlanner, PLN_EXBUFFERHI));
end;

function PlnExTime: PDouble;
begin
  Result := PDouble(OfsPtr(MainPlanner, PLN_EXTIME));
end;

function PlnExternalType: PSmallInt;
begin
  Result := PSmallInt(OfsPtr(MainPlanner, PLN_EXTERNALTYPE));
end;

function PlnExternalPulseRate(axis: Integer): PDouble;
begin
  Result := PDouble(OfsPtr(MainPlanner, PLN_EXTERNALPULSERATES + axis * 8));
end;

function PlnExternalStill: PByte;
begin
  Result := OfsPtr(MainPlanner, PLN_EXTERNALSTILL);
end;

function PlnBufferEnding: PByte;
begin
  Result := OfsPtr(MainPlanner, PLN_BUFFERENDING);
end;

function PlnJogging: PByte;
begin
  Result := OfsPtr(MainPlanner, PLN_JOGGING);
end;

function PlnVelocity(axis: Integer): PDouble;
begin
  Result := PDouble(OfsPtr(MainPlanner, PLN_VELOCITIES + axis * 8));
end;

function PlnAcceleration(axis: Integer): PDouble;
begin
  Result := PDouble(OfsPtr(MainPlanner, PLN_ACCELERATION + axis * 8));
end;

function PlnStepsPerAxis(axis: Integer): PDouble;
begin
  Result := PDouble(OfsPtr(MainPlanner, PLN_STEPSPERAXIS + axis * 8));
end;

function ViewPrinterOn: PByte;
begin
  Result := OfsPtr(ViewBlock, VIEW_PRINTERON);
end;

{ ================= Status line and state queries ================= }

function EngPtr(ofs: Integer): PByte;
begin
  Result := OfsPtr(Engine, ofs);
end;

function PlnPtr(ofs: Integer): PByte;
begin
  Result := OfsPtr(MainPlanner, ofs);
end;

function EngReady: Boolean;
begin
  Result := (Engine <> nil) and LayoutValid;
end;

function ProgramInReset: Boolean;
begin
  Result := EngReady and (EngPtr(ENG_RESET_FLAG)^ <> 0);
end;

function ProgramSimMode: Boolean;
begin
  Result := (Engine <> nil) and (EngPtr(ENG_SIM_MODE)^ <> 0);
end;

function OverrideActive: Boolean;
begin
  Result := EngReady and (EngPtr(ENG_OVERRIDE_FLAG)^ <> 0);
end;

{ Writes to the Mach3 status line. Mach3 shows a G-code comment there, so
  ( ) and commas are removed. }
procedure WriteMessage(const msgText: UnicodeString);
var
  s: UnicodeString;
  a: AnsiString;
  i: Integer;
  c: WideChar;
begin
  if not Assigned(Code) then Exit;
  s := '';
  for i := 1 to Length(msgText) do
  begin
    c := msgText[i];
    if (c <> '(') and (c <> ')') and (c <> ',') and (c >= ' ') then
      s := s + c;
  end;
  if s = '' then s := ' ';
  a := AnsiString('(' + s + ')');
  try
    Code(PAnsiChar(a));
  except
  end;
end;

procedure ScheduleMessage(durationMs: DWord);
begin
  if durationMs > 0 then
    gMsgEnd := TickMs64 + durationMs
  else
    gMsgEnd := 0;
end;

procedure StatusMessage(const msgText: UnicodeString; msgId: Integer; durationMs: DWord);
begin
  gMsgId := msgId;
  if gMsgDefer > 0 then
  begin
    gMsgWaiting := True;
    gMsgPending := msgText;
    gMsgPendingMs := durationMs;
    gMsgEnd := 0;
    Exit;
  end;
  WriteMessage(msgText);
  ScheduleMessage(durationMs);
end;

procedure ClearStatusMessage(msgId: Integer);
begin
  if (msgId = MSG_GENERAL) or (gMsgId <> msgId) then Exit;
  StatusMessage('', MSG_GENERAL, 0);
end;

procedure DeferStatusMessage(refresh: Integer);
begin
  if refresh > gMsgDefer then gMsgDefer := refresh;
end;

{ Called on every update cycle: shows deferred messages and clears expired ones. }
procedure StatusMessageService;
begin
  if gMsgDefer > 0 then
  begin
    Dec(gMsgDefer);
    if (gMsgDefer = 0) and gMsgWaiting then
    begin
      gMsgWaiting := False;
      WriteMessage(gMsgPending);
      ScheduleMessage(gMsgPendingMs);
    end;
    Exit;
  end;
  if (gMsgEnd <> 0) and (TickMs64 >= gMsgEnd) then
  begin
    gMsgEnd := 0;
    gMsgId := MSG_GENERAL;
    WriteMessage('');
  end;
end;

{ Helpers for the Mach3 input signal table (Ports & Pins) }
function InputRegister(idx: Integer): PByte;
begin
  Result := EngPtr(ENG_INSIG_BASE + idx * ENG_INSIG_STRIDE);
end;

function InputPin(idx: Integer): Integer;
var
  e: PByte;
begin
  e := InputRegister(idx);
  if (e[0] <> 0) and (e[2] = CARD_PORT) and (e[1] >= 1) and (e[1] <= INPUT_PIN_COUNT) then
    Result := e[1]
  else
    Result := 0;
end;

function InputActiveLow(idx: Integer): Boolean;
var
  e: PByte;
begin
  e := InputRegister(idx);
  Result := e[3] <> 0;
end;

function AxisLimit(idx: Integer): Boolean;
begin
  Result := (idx < ENG_LIMIT_SIGNALS) and ((idx mod 3) <> 2);
end;

function HomeOnSamePin(idx, pin: Integer): Boolean;
begin
  Result := InputPin((idx div 3) * 3 + 2) = pin;
end;

{ ================= TMainProgram ================= }

function TMainProgram.StepUnit(ax: Integer): Double;
begin
  if MainPlanner = nil then Exit(0);
  Result := PlnStepsPerAxis(ax)^;
  if IsNan(Result) or IsInfinite(Result) or (Result <= 0) then Result := 0;
end;

function TMainProgram.Accel(ax: Integer): Double;
begin
  if MainPlanner = nil then Exit(0);
  Result := PlnAcceleration(ax)^;
  if IsNan(Result) or IsInfinite(Result) or (Result <= 0) then Result := 0;
end;

function TMainProgram.HasPosition: Boolean;
begin
  Result := Engine <> nil;
end;

function TMainProgram.ReadPosition(ax: Integer): Integer;
begin
  Result := EngAxisIndex(ax)^;
end;

procedure TMainProgram.WritePosition(ax, val: Integer);
begin
  EngAxisIndex(ax)^ := val;
end;

function TMainProgram.HasResetInfo: Boolean;
begin
  Result := EngReady;
end;

function TMainProgram.InReset: Boolean;
begin
  Result := ProgramInReset;
end;

procedure TMainProgram.DoReset(cause: Integer);
begin
  if not EngReady then Exit;
  EngPtr(ENG_RESET_FLAG)^ := 1;
  PLongInt(EngPtr(ENG_RESET_REASON))^ := cause;
end;

{ Presses the Mach3 Reset button (puts Mach3 into E-Stop). }
procedure TMainProgram.PressResetButton;
begin
  if not Assigned(DoButton) then Exit;
  try
    DoButton(OEM_RESET);
  except
  end;
end;

{ Reads the spindle and coolant states from Mach3 and builds the card output byte. }
function TMainProgram.OutputMask(out covered: Byte): Byte;
var
  i, pin: Integer;
  e: PByte;
  bit: Byte;
  isOn: Boolean;
begin
  Result := 0;
  covered := 0;
  if not EngReady then Exit;
  for i := 0 to ENG_OUTSIG_COUNT - 1 do
  begin
    e := EngPtr(ENG_OUTSIG_BASE + i * ENG_OUTSIG_STRIDE);
    if (e[0] = 0) or (e[2] <> CARD_PORT) then Continue;
    pin := e[1];
    if (pin < 1) or (pin > OUTPUT_PIN_COUNT) then Continue;
    bit := Byte(1 shl (pin - 1));
    covered := covered or bit;
    if e[4] <> 0 then isOn := e[3] = 0 else isOn := e[3] <> 0;
    if isOn then Result := Result or bit
    else Result := Result and not bit;
  end;
end;

{ Copies the card inputs (limits, probe, buttons) to the Mach3 input signals.
  Returns True if a limit switch requires a reset. }
function TMainProgram.TransferInputs(inputBits: Byte): Boolean;
var
  i, pin: Integer;
  e: PByte;
  check, active, clean: Boolean;
begin
  Result := False;
  if not EngReady then Exit;
  check := (not ProgramInReset) and (not OverrideActive);
  for i := 0 to ENG_INSIG_COUNT - 1 do
  begin
    pin := InputPin(i);
    if pin = 0 then Continue;
    e := InputRegister(i);
    active := ((inputBits shr (pin - 1)) and 1) <> 0;
    if e[3] <> 0 then active := not active;
    if active then e[8] := 1 else e[8] := 0;
    if check and active and AxisLimit(i) and (not HomeOnSamePin(i, pin)) then
      Result := True;
  end;
  if OverrideActive then
  begin
    clean := True;
    for i := 0 to ENG_LIMIT_SIGNALS - 1 do
    begin
      if not AxisLimit(i) then Continue;
      e := InputRegister(i);
      if (e[0] <> 0) and (e[8] <> 0) then clean := False;
    end;
    if clean then
    begin
      EngPtr(ENG_OVERRIDE_FLAG)^ := 0;
      StatusMessage(MsgOverrideExited);
    end;
  end;
end;

{ Should the limit switches stop the card immediately (not in limit override mode)? }
function TMainProgram.LimitStop: Boolean;
var
  i, pin: Integer;
  limits, others, bit: Byte;
begin
  Result := False;
  if (not EngReady) or OverrideActive then Exit;
  limits := 0;
  others := 0;
  for i := 0 to ENG_INSIG_COUNT - 1 do
  begin
    pin := InputPin(i);
    if (pin = 0) or (pin > LIMIT_PIN_COUNT) then Continue;
    bit := Byte(1 shl (pin - 1));
    if AxisLimit(i) and (not InputActiveLow(i)) and (not HomeOnSamePin(i, pin)) then
      limits := limits or bit
    else
      others := others or bit;
  end;
  Result := (limits <> 0) and (others = 0);
end;

function TMainProgram.SpindlePWM: Byte;
begin
  Result := 255;
end;

procedure TMainProgram.GcodeFinished;
begin
  if MainPlanner <> nil then SetCBool(PlnExternalStill, True);
end;

{ Tells the Mach3 planner that an external motion device is in use and sets
  the slice time and the buffer length. }
procedure TMainProgram.ConfigureDriver(kernelKHz, fifoHigh: Integer; sliceSec: Double);
var
  hz: Double;
  i: Integer;
begin
  if MainPlanner = nil then Exit;
  hz := kernelKHz * 1000.0;
  for i := 0 to 6 do PlnExternalPulseRate(i)^ := hz;
  PlnExBufferHi^ := fifoHigh;
  PlnExTime^ := sliceSec;
  PlnExternalType^ := EX_VMS;
  PlnPtr(PLN_EXT_DEVICE_FLAG)^ := 1;
  PlnPtr(PLN_FLAG_29E376)^ := 0;
  PlnPtr(PLN_FLAG_2462F0)^ := 1;
end;

procedure TMainProgram.ShowMessage(const msgText: UnicodeString);
begin
  StatusMessage(msgText);
end;

procedure TMainProgram.TimedMessage(const msgText: UnicodeString; msgId: Integer; durationMs: DWord);
begin
  StatusMessage(msgText, msgId, durationMs);
end;

procedure TMainProgram.RemoveMessage(msgId: Integer);
begin
  ClearStatusMessage(msgId);
end;

{ Returns the step positions of one trajectory point. }
procedure TMainProgram.MoveSteps(const trajPoint: TGMoves; var stepCount: array of Double);
begin
  if High(stepCount) >= 0 then stepCount[0] := trajPoint.ex;
  if High(stepCount) >= 1 then stepCount[1] := trajPoint.ey;
  if High(stepCount) >= 2 then stepCount[2] := trajPoint.ez;
  if High(stepCount) >= 3 then stepCount[3] := trajPoint.ea;
end;

initialization
  gLayoutOk := ComputeLayout;

end.
