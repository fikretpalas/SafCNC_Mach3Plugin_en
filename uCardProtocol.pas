{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uCardProtocol.pas - packet format shared with the Arduino firmware
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  PC -> card: state packets (8 bytes) and move packets (10 bytes).
  Card -> PC: one status packet (8 bytes) every 50 ms, and an info packet
  (K_INFO, 8 bytes) with the card's speed limit at power-up, after a link
  timeout and on the first K_RESET of a new connection.
  Every packet: command, sequence counter, data and a 16-bit checksum
  (the sum of all bytes is zero).
}
unit uCardProtocol;

{$mode objfpc}{$H+}
{$R-}{$Q-}

interface

const
  UNO_BAUD = 115200;
  UNO_STATUS_LEN = 8;
  UNO_MOVE_LEN = 10;
  UNO_MAX_LEN = 10;
  UNO_AXES = 3;
  UNO_SLICE_SEC = 0.005;
  UNO_SUBTICKS = 160;
  // Maximum steps per axis in one 5 ms slice: 160 = 32 kHz (Arduino Uno / Nano).
  // The card reports its own limit in the K_INFO packet and the plugin uses that
  // value (CardStepMax below); these constants are used until K_INFO arrives.
  // An Arduino Uno / Nano itself cannot go beyond about 30 kHz.
  UNO_STEP_MAX = 160;
  // The protocol never goes above 500 steps per slice = 100 kHz (an STM32 card).
  // A card that reports more is limited to this value.
  UNO_STEP_CEILING = 500;
  // Step rate of the ceiling (200 slices per second): 100 kHz. Mach3 is given
  // this while no card is connected, so Motor Tuning can be set up to it.
  UNO_CEILING_HZ = UNO_STEP_CEILING * 200;
  UNO_FIFO_MAX = 199;
  UNO_FAST_SLICE = 32;

  { Packet commands; the card reports its state with the same codes }
  K_INFO = 64;     // card -> PC only: card limits (see DecodeInfo)
  K_RESET = 65;
  K_READY = 66;
  K_ACTIVE = 67;
  K_MOVE = 68;

  { Reset reason codes }
  H_POWERON = 0;
  H_ESTOP = 1;
  H_SEQ = 2;
  H_CHECKSUM = 3;
  H_BADCMD = 4;
  H_BADLEN = 5;
  H_TIMEOUT = 6;
  H_OVERFLOW = 7;
  H_STEPLIMIT = 8;
  H_LOCALESTOP = 9;
  H_BADSTATE = 10;
  H_LIMIT = 11;
  H_PROBE = 12;
  H_UNDERRUN = 13;
  H_MAX = 13;

  { Output byte bits (PC -> card) }
  O_SPINDLE_EN = $01;
  O_SPINDLE_DIR = $02;
  O_FLOOD = $04;
  O_MIST = $08;
  O_LIMSTOP = $10;
  O_PRBSTOP = $20;
  O_UNDERRUN = $40;
  O_OUTPUTS = $0F;
  O_MASK = $7F;

  { Input byte bits (card -> PC) }
  I_LIMIT_X = $01;
  I_LIMIT_Y = $02;
  I_LIMIT_Z = $04;
  I_HOLD = $08;
  I_START = $10;
  I_ESTOP = $20;
  I_PROBE = $40;
  I_MASK = $7F;

  OUTPUT_PIN_COUNT = 4;
  INPUT_PIN_COUNT = 7;
  LIMIT_PIN_COUNT = 3;

type
  TPacket = array[0..UNO_MAX_LEN - 1] of Byte;
  TAxisInt = array[0..UNO_AXES - 1] of LongInt;

  { Decoded status packet received from the card }
  TUnoStatus = record
    State: Byte;
    Counter: Byte;
    Error: Byte;
    SliceCount: Word;
    SubStep: Byte;
    Fifo: Byte;
    Spindle: Byte;
    Outputs: Byte;
    Inputs: Byte;
  end;

  { Card speed from the K_INFO packet }
  TUnoInfo = record
    StepLimit: Integer;   // max steps per axis per 5 ms slice (firmware STEP_LIMIT)
    SubTicks: Integer;    // step ticks per slice (firmware SUBTICKS)
    MaxStepHz: Integer;   // max step rate per axis in Hz = StepLimit x 200
    FromCard: Boolean;    // False = defaults, the card has not reported yet
  end;

var
  { Limits in use. Written only by the Mach3 thread (TCardLayer), and only
    while the card is in RESET, because the card sends K_INFO only then. }
  CardStepMax: Integer = UNO_STEP_MAX;
  CardSubTicks: Integer = UNO_SUBTICKS;

function EmptyStatus: TUnoStatus;
function DefaultInfo: TUnoInfo;
function DecodeInfo(const p: TPacket; out info: TUnoInfo): Boolean;
procedure ApplyCardInfo(const info: TUnoInfo);
{ Denominator of the sub-tick in the RESET status packet (see DecodeInfo). }
function SubStepScale: Integer;
function PacketLength(k: Byte; toCard: Boolean): Integer;
procedure AddChecksum(var p: TPacket; len: Integer);
function ChecksumOk(const p: TPacket; len: Integer): Boolean;
procedure StatePacket(var p: TPacket; k, h, spindle, output: Byte);
procedure MovePacket(var p: TPacket; x, y, z: Integer);
function PacketSteps(const p: TPacket; ax: Integer): Integer;
function DecodePacket(const p: TPacket; out d: TUnoStatus): Boolean;
function PartialStep(stepCount: Integer; subStep: Byte): Integer;
function StatusLampCode(state: Byte): Byte;

implementation

function EmptyStatus: TUnoStatus;
begin
  FillChar(Result, SizeOf(Result), 0);
end;

function DefaultInfo: TUnoInfo;
begin
  Result.StepLimit := UNO_STEP_MAX;
  Result.SubTicks := UNO_SUBTICKS;
  Result.MaxStepHz := Round(UNO_STEP_MAX / UNO_SLICE_SEC);
  Result.FromCard := False;
end;

{ K_INFO (card -> PC, 8 bytes):
    p[2..3] = STEP_LIMIT, p[4..5] = SUBTICKS (both 16-bit, big-endian).
  Arduino Uno / Nano: 160 / 160 = 32 kHz; an STM32 card up to 500 = 100 kHz.
  The step limit is never taken above UNO_STEP_CEILING, nor above the
  sub-tick count (the card's DDA makes at most one step per tick). }
function DecodeInfo(const p: TPacket; out info: TUnoInfo): Boolean;
var
  steps, ticks: Integer;
begin
  info := DefaultInfo;
  Result := False;
  if p[0] <> K_INFO then Exit;
  if not ChecksumOk(p, UNO_STATUS_LEN) then Exit;
  steps := (Integer(p[2]) shl 8) or p[3];
  ticks := (Integer(p[4]) shl 8) or p[5];
  if (steps < 1) or (ticks < 1) then Exit;
  if steps > UNO_STEP_CEILING then steps := UNO_STEP_CEILING;
  if steps > ticks then steps := ticks;
  info.StepLimit := steps;
  info.SubTicks := ticks;
  info.MaxStepHz := Round(steps / UNO_SLICE_SEC);
  info.FromCard := True;
  Result := True;
end;

procedure ApplyCardInfo(const info: TUnoInfo);
begin
  CardStepMax := info.StepLimit;
  CardSubTicks := info.SubTicks;
end;

{ The sub-tick in the RESET status packet is one byte. A card with more than
  255 sub-ticks sends it scaled to 0..255 (sub * 256 / SUBTICKS). }
function SubStepScale: Integer;
begin
  if CardSubTicks > 255 then Result := 256 else Result := CardSubTicks;
end;

function PacketLength(k: Byte; toCard: Boolean): Integer;
begin
  case k of
    K_INFO:
      if toCard then Result := 0 else Result := UNO_STATUS_LEN;
    K_RESET, K_READY, K_ACTIVE:
      Result := UNO_STATUS_LEN;
    K_MOVE:
      if toCard then Result := UNO_MOVE_LEN else Result := UNO_STATUS_LEN;
  else
    Result := 0;
  end;
end;

function Sum16(const p: TPacket; len: Integer): Word;
var
  s: DWord;
  i: Integer;
begin
  s := 0;
  for i := 0 to len - 3 do
    s := s + p[i];
  Result := Word(s and $FFFF);
end;

procedure AddChecksum(var p: TPacket; len: Integer);
var
  m: Word;
begin
  m := Word((DWord($10000) - Sum16(p, len)) and $FFFF);
  p[len - 2] := Byte(m shr 8);
  p[len - 1] := Byte(m and $FF);
end;

function ChecksumOk(const p: TPacket; len: Integer): Boolean;
var
  m: DWord;
begin
  m := (DWord(p[len - 2]) shl 8) or p[len - 1];
  Result := ((DWord(Sum16(p, len)) + m) and $FFFF) = 0;
end;

procedure StatePacket(var p: TPacket; k, h, spindle, output: Byte);
begin
  FillChar(p, SizeOf(p), 0);
  p[0] := k;
  if k = K_RESET then
  begin
    if h > H_MAX then h := H_POWERON;
    p[2] := h;
  end
  else
  begin
    p[3] := spindle;
    p[4] := output and O_MASK;
  end;
end;

function ClampSteps(v: Integer): Integer;
begin
  if v > CardStepMax then Exit(CardStepMax);
  if v < -CardStepMax then Exit(-CardStepMax);
  Result := v;
end;

procedure WriteSteps(var p: TPacket; i, v: Integer);
var
  w: Word;
begin
  w := Word(SmallInt(ClampSteps(v)));
  p[i] := Byte(w shr 8);
  p[i + 1] := Byte(w and $FF);
end;

procedure MovePacket(var p: TPacket; x, y, z: Integer);
begin
  FillChar(p, SizeOf(p), 0);
  p[0] := K_MOVE;
  WriteSteps(p, 2, x);
  WriteSteps(p, 4, y);
  WriteSteps(p, 6, z);
end;

function PacketSteps(const p: TPacket; ax: Integer): Integer;
var
  w: Word;
begin
  w := Word((Word(p[2 + ax * 2]) shl 8) or p[3 + ax * 2]);
  Result := SmallInt(w);
end;

function DecodePacket(const p: TPacket; out d: TUnoStatus): Boolean;
begin
  d := EmptyStatus;
  Result := False;
  if p[0] = K_INFO then Exit;   // not a status packet, see DecodeInfo
  if PacketLength(p[0], False) <> UNO_STATUS_LEN then Exit;
  if not ChecksumOk(p, UNO_STATUS_LEN) then Exit;
  d.State := p[0];
  d.Counter := p[1];
  if p[0] = K_RESET then
  begin
    d.Error := p[2];
    d.SliceCount := Word((Word(p[3]) shl 8) or p[4]);
    d.SubStep := p[5];
  end
  else
  begin
    d.Fifo := p[2];
    if d.Fifo > UNO_FIFO_MAX then d.Fifo := UNO_FIFO_MAX;
    d.Spindle := p[3];
    d.Outputs := p[4] and O_MASK;
    d.Inputs := p[5] and I_MASK;
  end;
  Result := True;
end;

{ Number of steps executed in an interrupted slice, up to sub-tick subStep. }
function PartialStep(stepCount: Integer; subStep: Byte): Integer;
begin
  if stepCount >= 0 then
    Result := (stepCount * subStep) div SubStepScale
  else
    Result := -((-stepCount * subStep) div SubStepScale);
end;

{ Lamp colour code for the status window. }
function StatusLampCode(state: Byte): Byte;
begin
  case state of
    K_RESET: Result := 0;
    K_READY, K_ACTIVE: Result := 1;
    K_MOVE: Result := 6;
  else
    Result := 255;
  end;
end;

end.
