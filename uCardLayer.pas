{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uCardLayer.pas - logic layer between Mach3 and the serial link
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  Connection state, position counting, G-code and jog slices, outputs,
  card alarms and the data shown in the status window.
}
unit uCardLayer;

{$mode objfpc}{$H+}
{$codepage utf8}
{$R-}{$Q-}

interface

uses
  Windows, SysUtils, Math, uWinApi, uMessages, uCardProtocol, uMainProgram,
  uSerialLink, uCardWindow;

const
  EX_BUFFER_HI = 200;
  BUFFER_PACKETS = 40;
  MAX_SLICES_PER_UPDATE = 100;
  JOG_BUFFER_PACKETS = 14;
  JOG_STOP_PACKETS = 180;
  END_WAIT_MS = 300;
  CARD_WAIT_MS = 3000;
  CARD_SEARCH_MS = 8000;
  CONNECT_MSG_MS = 3000;
  ALARM_RESET_ENABLED = True;
  SPINDLE_CCW_ENABLES = True;
  UNDERRUN_CHECK = True;

type
  TConnState = (csSearching, csConnected, csLost, csNotFound);
  TAxisReal = array[0..AXIS_COUNT - 1] of Double;
  TAxisSteps = array[0..AXIS_COUNT - 1] of LongInt;

  TCardLayer = class
  private
    FHost: TMainProgram;
    FPort: TSerialLink;
    FMoveTick: Double;
    FSteps: TAxisSteps;
    FRequested: TAxisReal;
    FLastWritten: TAxisSteps;
    FLinked: Boolean;
    FWantLink: Boolean;
    FWaiting: Boolean;
    FWaitStart: DWord;
    FConnState: TConnState;
    FConnStart: DWord;
    FPortNo: DWord;
    FJogRunning: Boolean;
    FJogStopping: Boolean;
    FJogVel: TAxisReal;
    FJogT0: DWord;
    FJogSent: Integer;
    FGcodeActive: Boolean;
    FEndWaiting: Boolean;
    FEndSince: DWord;
    FMaxAhead: Integer;
    FPerUpdate: Integer;
    FOutput: Byte;
    FSpindle: Byte;
    FSentOutput: Byte;
    FSentSpindle: Byte;
    FSt: TUnoStatus;
    FStNo: DWord;
    FSliceDelta: TAxisReal;
    FSliceSteps: TAxisSteps;
    FJogD: TAxisReal;
    FFastJog: Boolean;
    FLastSliceStatusNo: DWord;
    FLastSliceStatus: TUnoStatus;
    FInfoNo: DWord;
    FMaxStepHz: Integer;
    FCardInfo: TUnoInfo;
    FCardInfoKnown: Boolean;

    function InReset: Boolean;
    function MoveOk: Boolean;
    function BoardMoving: Boolean;
    function WinGetInfo: TCardInfo;
    function JogAccel(axis: Integer): Double;
    function UnderrunSafe: Boolean;
    function ControlBits: Byte;
    procedure RefreshOut;
    procedure SendOutputs;
    procedure SyncOutputs;
    procedure ConfigureKernel(stepHz: Integer);
    procedure CheckCardInfo;
    function VelocitiesOk(warn: Boolean): Boolean;
    procedure TryLink;
    procedure CheckConnection;
    procedure StartSearch(const msgText: UnicodeString);
    procedure ConnectionMessages;
    procedure DropLink(cause: Byte);
    procedure EnterReset(reason: Integer);
    procedure PressResetButton;
    procedure CheckCardAlarm;
    procedure ApplyStop;
    function ApplySlice(const d: array of Double): Boolean;
    procedure JogReset;
    procedure NextJogVelocity(const rates: array of Double);
    procedure JogRun(const rates: array of Double);
    procedure ServiceJogStop;
    function HasResidualSteps: Boolean;
  public
    constructor Create(AHost: TMainProgram);
    destructor Destroy; override;

    procedure ShowForm(owner: HWND);
    procedure UpdateForm;
    procedure FreeForm;
    procedure Startup;
    procedure Shutdown;
    procedure ToggleConnection;
    function CanAccept: Boolean;
    procedure GcodeSlice(const stepCount: array of Double);
    procedure FinalizeMoves;
    procedure Purge;
    procedure JogAdvance(const rates: array of Double; dt: Double);
    procedure JogService;
    function JogBusy: Boolean;
    function MotionStopped: Boolean;
    procedure JogStop;
    procedure JogStopRamp;
    procedure UpdatePosition;

    property Linked: Boolean read FLinked;
    property FastJog: Boolean read FFastJog write FFastJog;
  end;

implementation

const
  ZERO_RATES: TAxisReal = (0, 0, 0, 0);

function RoundPosition(d: Double; defaultValue: Integer): Integer;
begin
  if IsNan(d) or IsInfinite(d) or (d > 2147483646.0) or (d < -2147483647.0) then
    Exit(defaultValue);
  Result := Floor(d + 0.5);
end;

function NowTick: DWord;
begin
  Result := DWord(TickMs64 and $FFFFFFFF);
end;

{ Signed difference between two DWord time stamps (wrap-around safe). }
function Elapsed(tNow, tThen: DWord): LongInt;
begin
  Result := LongInt(DWord(tNow - tThen));
end;

constructor TCardLayer.Create(AHost: TMainProgram);
begin
  inherited Create;
  FHost := AHost;
  FMoveTick := UNO_SLICE_SEC;
  FMaxAhead := BUFFER_PACKETS;
  FPort := TSerialLink.Create;
end;

destructor TCardLayer.Destroy;
begin
  FPort.Free;
  inherited Destroy;
end;

function TCardLayer.InReset: Boolean;
begin
  Result := FHost.InReset;
end;

function TCardLayer.MoveOk: Boolean;
begin
  Result := FPort.MoveReady;
end;

function TCardLayer.BoardMoving: Boolean;
var
  d: TUnoStatus;
begin
  FPort.GetStatus(d);
  Result := d.State = K_MOVE;
end;

procedure TCardLayer.ShowForm(owner: HWND);
begin
  ShowCardWindow(owner, @WinGetInfo);
end;

procedure TCardLayer.UpdateForm;
begin
  UpdateCardWindow;
end;

procedure TCardLayer.FreeForm;
begin
  FreeCardWindow;
end;

function TCardLayer.WinGetInfo: TCardInfo;
begin
  Result.Linked := FLinked;
  Result.Present := FPort.Seen;
  Result.PortName := FPort.PortName;
  Result.Baud := UNO_BAUD;
  Result.HasFb := FPort.Seen and (FStNo <> 0);
  Result.Lamp := StatusLampCode(FSt.State);
end;

function TCardLayer.JogAccel(axis: Integer): Double;
var
  a, s: Double;
begin
  a := FHost.Accel(axis);
  s := FHost.StepUnit(axis);
  if IsNan(a) or IsInfinite(a) or IsNan(s) or IsInfinite(s) then Exit(0);
  if (a > 0) and (s > 0) then Result := a * s else Result := 0;
end;

{ When the motion is slow enough, a short USB delay cannot cause a buffer underrun. }
function TCardLayer.UnderrunSafe: Boolean;
var
  known, i: Integer;
  a: Double;
begin
  known := 0;
  for i := 0 to UNO_AXES - 1 do
  begin
    a := JogAccel(i);
    if a <= 0 then Continue;
    if a * FMoveTick * FMoveTick > UNO_FAST_SLICE / 2.0 then Exit(False);
    Inc(known);
  end;
  Result := known > 0;
end;

function TCardLayer.ControlBits: Byte;
begin
  Result := 0;
  if FHost.LimitStop then Result := Result or O_LIMSTOP;
  if UNDERRUN_CHECK and FGcodeActive and UnderrunSafe then Result := Result or O_UNDERRUN;
end;

procedure TCardLayer.RefreshOut;
var
  covered, m: Byte;
begin
  m := FHost.OutputMask(covered);
  m := m and covered and O_OUTPUTS;
  if SPINDLE_CCW_ENABLES and ((m and O_SPINDLE_DIR) <> 0) then m := m or O_SPINDLE_EN;
  if InReset then m := 0
  else m := m or ControlBits;
  FOutput := m;
  FSpindle := FHost.SpindlePWM;
end;

procedure TCardLayer.SendOutputs;
begin
  FPort.SetOutputs(FSpindle, FOutput);
  FSentOutput := FOutput;
  FSentSpindle := FSpindle;
end;

procedure TCardLayer.SyncOutputs;
begin
  if (not FLinked) or (not FPort.Connected) then Exit;
  if (FOutput <> FSentOutput) or (FSpindle <> FSentSpindle) then SendOutputs;
end;

{ Tells Mach3 the highest step rate (pulse rate); Mach3 limits the axis
  velocities to it. }
procedure TCardLayer.ConfigureKernel(stepHz: Integer);
var
  khz: Integer;
begin
  FMaxStepHz := stepHz;
  khz := stepHz div 1000;
  if khz < 1 then khz := 1;
  FHost.ConfigureDriver(khz, EX_BUFFER_HI, UNO_SLICE_SEC);
end;

{ While no card is connected Mach3 is given the protocol ceiling (100 kHz), so
  Motor Tuning can be set up to it. Once the card is connected and reports its
  own limit with K_INFO, Mach3 is given that limit and the axis velocities are
  checked. The step limit used for the slices (CardStepMax) always stays at
  the last known card value. The Mach3 driver setting is written only when
  the value changes. }
procedure TCardLayer.CheckCardInfo;
var
  info: TUnoInfo;
  no: DWord;
  changed: Boolean;
begin
  no := FPort.GetInfo(info);
  if no = FInfoNo then
  begin
    if not FPort.Seen then
    begin
      FCardInfoKnown := False;
      if FMaxStepHz <> UNO_CEILING_HZ then ConfigureKernel(UNO_CEILING_HZ);
    end;
    Exit;
  end;
  FInfoNo := no;
  ApplyCardInfo(info);
  if not info.FromCard then Exit;
  FCardInfo := info;
  FCardInfoKnown := True;
  changed := info.MaxStepHz <> FMaxStepHz;
  if changed then ConfigureKernel(info.MaxStepHz);
  if changed then
    FHost.ShowMessage(FormatText(MsgCardSpeed,
      [UnicodeString(IntToStr(info.MaxStepHz div 1000) + '.' +
                     IntToStr((info.MaxStepHz mod 1000) div 100)),
       UnicodeString(IntToStr(info.StepLimit))]));
  VelocitiesOk(True);
end;

function NumText(v: Double; digits: Integer): UnicodeString;
begin
  Result := UnicodeString(FloatToStrF(v, ffFixed, 15, digits));
end;

{ Compares the Mach3 axis velocities (Motor Tuning) with the card limit.
  Returns True if they are all within it. If an axis is too fast it returns
  False and, when warn is True (and the window is not open yet), opens the
  warning with each axis velocity and the highest velocity allowed. The
  plugin does not change the Mach3 settings. }
function TCardLayer.VelocitiesOk(warn: Boolean): Boolean;
const
  AXIS_LETTER: array[0..2] of UnicodeString = ('X', 'Y', 'Z');
var
  i: Integer;
  spu, vel, maxVel: Double;
  lines: UnicodeString;
begin
  Result := True;
  if (not FCardInfoKnown) or (MainPlanner = nil) then Exit;
  lines := '';
  for i := 0 to UNO_AXES - 1 do
  begin
    spu := Abs(PlnStepsPerAxis(i)^);
    vel := PlnVelocity(i)^;
    if IsNan(spu) or IsInfinite(spu) or IsNan(vel) or IsInfinite(vel) then Continue;
    if (spu <= 0) or (vel <= 0) then Continue;
    if vel * spu <= FCardInfo.MaxStepHz then Continue;
    Result := False;
    maxVel := Floor(FCardInfo.MaxStepHz / spu * 60.0 * 100.0) / 100.0;
    lines := lines + FormatText(SpeedLine, [AXIS_LETTER[i], NumText(vel * 60.0, 2),
      NumText(vel * spu / 1000.0, 1), NumText(maxVel, 2)]) + #13#10;
  end;
  if Result or (not warn) or SpeedWarningOpen then Exit;
  ShowSpeedWarning(FormatText(SpeedIntro, [NumText(FCardInfo.MaxStepHz / 1000.0, 1),
    UnicodeString(IntToStr(FCardInfo.StepLimit))]) + #13#10#13#10 + lines + #13#10 + SpeedEnd);
end;

procedure TCardLayer.Startup;
var
  info: TUnoInfo;
begin
  FInfoNo := FPort.GetInfo(info);
  ApplyCardInfo(info);
  ConfigureKernel(UNO_CEILING_HZ);
  FStNo := 0;
  FPort.Service;
  FPortNo := FPort.NewPortCount;
  StartSearch(MsgSearchingCard);
end;

procedure TCardLayer.Shutdown;
begin
  if FPort.Connected then
  begin
    if FLinked then
    begin
      FPort.LinkReset(H_POWERON);
      FLinked := False;
      FWantLink := False;
    end;
    FPort.Flush(300);
    FPort.Close;
  end;
end;

{ Links the card (opens the connection) when Mach3 has left reset and a card is present. }
procedure TCardLayer.TryLink;
begin
  if FLinked or (not FWantLink) then Exit;
  if not FPort.Seen then Exit;
  if InReset then Exit;
  { If the Mach3 velocities exceed the card limit the card is not linked: the
    warning opens and Mach3 is put back into reset. Checked again on every Reset. }
  if not VelocitiesOk(True) then
  begin
    FHost.ShowMessage(MsgVelocitiesTooHigh);
    EnterReset(RESET_REASON_CARD);
    Exit;
  end;
  FGcodeActive := False;
  FEndWaiting := False;
  RefreshOut;
  SendOutputs;
  FPort.LinkStart;
  FLinked := True;
end;

{ Mach3 is out of reset but the card is not linked: waits CARD_WAIT_MS for the
  card, then puts Mach3 back into reset. }
procedure TCardLayer.CheckConnection;
begin
  if FLinked or InReset or (not FHost.HasResetInfo) then
  begin
    FWaiting := False;
    Exit;
  end;
  FWantLink := True;
  if not FWaiting then
  begin
    FWaiting := True;
    FWaitStart := NowTick;
    Exit;
  end;
  if Elapsed(NowTick, FWaitStart) < CARD_WAIT_MS then Exit;
  FWaiting := False;
  FHost.ShowMessage(MsgCardNotFoundReset);
  EnterReset(RESET_REASON_WATCHDOG);
end;

procedure TCardLayer.StartSearch(const msgText: UnicodeString);
begin
  FConnState := csSearching;
  FConnStart := NowTick;
  FHost.TimedMessage(msgText, MSG_CONNECTION, 0);
end;

{ Shows the connection messages (searching, connected, not found, lost) on the status line. }
procedure TCardLayer.ConnectionMessages;
var
  newPort: Boolean;
begin
  newPort := FPort.NewPortCount <> FPortNo;
  FPortNo := FPort.NewPortCount;
  if FPort.Seen then
  begin
    if FConnState <> csConnected then
    begin
      FConnState := csConnected;
      FHost.TimedMessage(MsgCardConnected, MSG_CONNECTION, CONNECT_MSG_MS);
    end;
    Exit;
  end;
  if newPort then
  begin
    StartSearch(MsgNewConnection);
    Exit;
  end;
  case FConnState of
    csConnected:
      begin
        FConnState := csLost;
        FHost.TimedMessage(MsgConnectionLost, MSG_CONNECTION, 0);
      end;
    csSearching:
      if Elapsed(NowTick, FConnStart) >= CARD_SEARCH_MS then
      begin
        FConnState := csNotFound;
        FHost.TimedMessage(MsgCardNotFound, MSG_CONNECTION, 0);
      end;
  end;
end;

{ Mach3 Reset button: links the card or drops the link. }
procedure TCardLayer.ToggleConnection;
var
  wantConnect: Boolean;
begin
  if FHost.HasResetInfo then
    wantConnect := not InReset
  else
    wantConnect := not (FLinked or FWantLink);
  if wantConnect then
  begin
    FWantLink := True;
    FPort.Service;
    TryLink;
  end
  else
    DropLink(H_POWERON);
end;

procedure TCardLayer.DropLink(cause: Byte);
var
  i: Integer;
begin
  if FPort.Connected then FPort.LinkReset(cause);
  FLinked := False;
  FWantLink := False;
  JogReset;
  FGcodeActive := False;
  FEndWaiting := False;
  FSentOutput := 0;
  FSentSpindle := 0;
  for i := 0 to AXIS_COUNT - 1 do FRequested[i] := FSteps[i];
end;

procedure TCardLayer.EnterReset(reason: Integer);
begin
  if FHost.HasResetInfo then
  begin
    FHost.DoReset(reason);
    DropLink(H_ESTOP);
  end
  else
  begin
    DropLink(H_ESTOP);
    PressResetButton;
  end;
end;

procedure TCardLayer.PressResetButton;
begin
  if InReset then Exit;
  FHost.PressResetButton;
end;

{ Handles errors reported by the card or by the link: shows the reason
  and puts Mach3 into reset. }
procedure TCardLayer.CheckCardAlarm;
var
  h, source: Byte;
  hasError: Boolean;
  reason: Integer;
begin
  hasError := FPort.TakeError(h, source);
  if (not hasError) and FLinked and (not FPort.Connected) then
  begin
    hasError := True;
    h := H_TIMEOUT;
    source := HS_PC;
  end;
  if (not hasError) or (not FLinked) then Exit;
  if InReset then
  begin
    DropLink(h);
    Exit;
  end;
  FHost.ShowMessage(CardResetMessage(h, source = HS_CARD));
  if not ALARM_RESET_ENABLED then
  begin
    DropLink(h);
    FWantLink := True;
    Exit;
  end;
  case h of
    H_POWERON, H_TIMEOUT: reason := RESET_REASON_WATCHDOG;
  else
    reason := RESET_REASON_CARD;
  end;
  EnterReset(reason);
end;

{ After a stop, corrects the position by the steps the card did not execute. }
procedure TCardLayer.ApplyStop;
var
  diff: TAxisInt;
  uncertain: Boolean;
  i: Integer;
begin
  if not FPort.TakeStop(diff, uncertain) then Exit;
  for i := 0 to UNO_AXES - 1 do
  begin
    FSteps[i] := FSteps[i] - diff[i];
    FRequested[i] := FRequested[i] - diff[i];
  end;
  if uncertain then FHost.ShowMessage(MsgPositionUncertain);
end;

function TCardLayer.CanAccept: Boolean;
begin
  Result := FLinked and MoveOk and (FPort.Depth < FMaxAhead) and
    (FPerUpdate < MAX_SLICES_PER_UPDATE);
end;

{ Converts one slice of the planned move into whole steps per axis and sends it.
  Fractions are kept, so no step is lost over time. }
function TCardLayer.ApplySlice(const d: array of Double): Boolean;
var
  i, stepCount: Integer;
  delta: Double;
begin
  Result := False;
  if not FLinked then Exit;
  for i := 0 to AXIS_COUNT - 1 do
  begin
    if i <= High(d) then delta := d[i] else delta := 0;
    if IsNan(delta) or IsInfinite(delta) then delta := 0;
    FSliceDelta[i] := delta;
    stepCount := RoundPosition(FRequested[i] + delta, FSteps[i]) - FSteps[i];
    if i < UNO_AXES then
    begin
      if stepCount > CardStepMax then stepCount := CardStepMax
      else if stepCount < -CardStepMax then stepCount := -CardStepMax;
    end;
    FSliceSteps[i] := stepCount;
  end;
  if not FPort.SendSlice(FSliceSteps[0], FSliceSteps[1], FSliceSteps[2]) then
  begin
    for i := 0 to AXIS_COUNT - 1 do FRequested[i] := FSteps[i];
    Exit;
  end;
  Inc(FPerUpdate);
  FLastSliceStatusNo := FPort.GetStatus(FLastSliceStatus);
  for i := 0 to AXIS_COUNT - 1 do
  begin
    FRequested[i] := FRequested[i] + FSliceDelta[i];
    FSteps[i] := FSteps[i] + FSliceSteps[i];
  end;
  Result := True;
end;

procedure TCardLayer.GcodeSlice(const stepCount: array of Double);
begin
  if InReset or (not FLinked) then Exit;
  FGcodeActive := True;
  FEndWaiting := False;
  ApplySlice(stepCount);
end;

{ Is there a whole-step difference between the requested position and the steps
  actually sent? }
function TCardLayer.HasResidualSteps: Boolean;
var
  i: Integer;
begin
  Result := False;
  for i := 0 to UNO_AXES - 1 do
    if RoundPosition(FRequested[i], FSteps[i]) <> FSteps[i] then Exit(True);
end;

{ Waits until the card has finished the last moves, then tells Mach3 that the program is done. }
procedure TCardLayer.FinalizeMoves;
begin
  if FLinked and FGcodeActive then
  begin
    { If the last slices hit the step limit (CardStepMax) and some residual
      steps could not be sent, send them before finishing the move.
      Otherwise the machine stops a step or two short of the target. }
    if HasResidualSteps then
    begin
      if CanAccept then ApplySlice(ZERO_RATES);
      FEndWaiting := False;
      Exit;
    end;
    if FPort.Connected then
    begin
      if FPort.Depth > 0 then
      begin
        FEndWaiting := False;
        Exit;
      end;
      if not FEndWaiting then
      begin
        FEndWaiting := True;
        FEndSince := NowTick;
      end;
      if BoardMoving and (Elapsed(NowTick, FEndSince) < END_WAIT_MS) then Exit;
    end;
    FGcodeActive := False;
    FEndWaiting := False;
  end;
  FHost.GcodeFinished;
end;

{ Drops an interrupted G-code move: closes the motion state and aligns the
  fractional requested position with the steps actually executed. }
procedure TCardLayer.Purge;
var
  i: Integer;
begin
  FGcodeActive := False;
  FEndWaiting := False;
  for i := 0 to AXIS_COUNT - 1 do FRequested[i] := FSteps[i];
end;

procedure TCardLayer.JogReset;
var
  i: Integer;
begin
  FJogRunning := False;
  FJogStopping := False;
  FillChar(FJogVel, SizeOf(FJogVel), 0);
  { Jog finished: align the fractional requested position with the executed steps.
    Mach3 plans the next move from the whole-step position (DRO); a leftover
    fraction could shift it by one step. }
  for i := 0 to AXIS_COUNT - 1 do FRequested[i] := FSteps[i];
end;

{ Has the jog ended, is the card buffer empty and has the card stopped moving?
  Mach3 must be told "jog finished" only then; otherwise it reads the position
  before the deceleration steps are done and the next move is shifted. }
function TCardLayer.MotionStopped: Boolean;
var
  d: TUnoStatus;
  no: DWord;
begin
  if JogBusy then Exit(False);
  if (not FLinked) or (not FPort.Connected) then Exit(True);
  if FPort.Depth > 0 then Exit(False);
  { At least 2 fresh status packets (one every 50 ms) must have arrived AFTER
    the last slice was sent, and the card must report "buffer empty, no motion".
    An old status packet or the buffer estimate alone is not trusted. }
  no := FPort.GetStatus(d);
  if DWord(no - FLastSliceStatusNo) < 2 then Exit(False);
  Result := (d.State <> K_MOVE) and (d.Fifo = 0);
end;

{ Moves the jog velocity of every axis one slice towards the requested rate,
  limited by the axis acceleration, and stores the steps for that slice. }
procedure TCardLayer.NextJogVelocity(const rates: array of Double);
var
  maxRate, target, acc, step: Double;
  i: Integer;
begin
  maxRate := (CardStepMax - 1) / FMoveTick;
  for i := 0 to AXIS_COUNT - 1 do
  begin
    if i <= High(rates) then target := rates[i] else target := 0;
    if target > maxRate then target := maxRate;
    if target < -maxRate then target := -maxRate;
    acc := JogAccel(i);
    if acc > 0 then
    begin
      step := acc * FMoveTick;
      if FJogVel[i] < target - step then FJogVel[i] := FJogVel[i] + step
      else if FJogVel[i] > target + step then FJogVel[i] := FJogVel[i] - step
      else FJogVel[i] := target;
    end
    else
      FJogVel[i] := target;
    FJogD[i] := FJogVel[i] * FMoveTick;
  end;
end;

{ Produces jog slices with acceleration; keeps the card buffer short so that
  the machine stops quickly when the key is released. }
procedure TCardLayer.JogRun(const rates: array of Double);
var
  fifo, targetSlices: Integer;
begin
  if InReset or (not FLinked) then Exit;
  if not FJogRunning then
  begin
    FJogRunning := True;
    FillChar(FJogVel, SizeOf(FJogVel), 0);
    FJogT0 := NowTick;
    FJogSent := 0;
  end;
  if not MoveOk then Exit;
  { If the card buffer ran dry during the jog (for example Windows blocks the
    Mach3 window while its title bar is clicked or dragged), the motors have
    already stopped. The ramp is restarted from zero; resuming at full speed
    would stall the motors. }
  if (FJogSent > 0) and (FPort.Depth = 0) then
  begin
    FillChar(FJogVel, SizeOf(FJogVel), 0);
    FJogT0 := NowTick;
    FJogSent := 0;
  end;
  fifo := FMaxAhead;
  if FFastJog and (JOG_BUFFER_PACKETS < fifo) then fifo := JOG_BUFFER_PACKETS;
  targetSlices := Trunc(Elapsed(NowTick, FJogT0) / (FMoveTick * 1000.0)) + fifo;
  while (FJogSent < targetSlices) and (FPort.Depth < fifo) and
        (FPerUpdate < MAX_SLICES_PER_UPDATE) do
  begin
    NextJogVelocity(rates);
    if not ApplySlice(FJogD) then Break;
    Inc(FJogSent);
  end;
end;

{ Queues the whole deceleration ramp to the card at once (up to about 0.9 s).
  Used when Windows is about to block the Mach3 window: the card then slows
  down and stops on its own, even if no further slice can be produced.
  If the ramp is longer than that, JogService continues it afterwards. }
procedure TCardLayer.JogStopRamp;
var
  moving: Boolean;
  i: Integer;
begin
  if not FJogRunning then Exit;
  FJogStopping := True;
  if InReset or (not FLinked) or (not MoveOk) then Exit;
  while FPort.Depth < JOG_STOP_PACKETS do
  begin
    NextJogVelocity(ZERO_RATES);
    moving := False;
    for i := 0 to AXIS_COUNT - 1 do
      if FJogVel[i] <> 0 then moving := True;
    if not moving then Break;
    if not ApplySlice(FJogD) then Break;
    Inc(FJogSent);
  end;
end;

procedure TCardLayer.JogAdvance(const rates: array of Double; dt: Double);
begin
  FJogStopping := False;
  JogRun(rates);
end;

procedure TCardLayer.JogService;
begin
  ServiceJogStop;
end;

function TCardLayer.JogBusy: Boolean;
begin
  Result := FJogRunning or FJogStopping;
end;

procedure TCardLayer.JogStop;
begin
  if FJogRunning then FJogStopping := True;
end;

procedure TCardLayer.ServiceJogStop;
var
  allZero: Boolean;
  i: Integer;
begin
  if InReset then
  begin
    JogReset;
    Exit;
  end;
  if not FJogStopping then Exit;
  if FLinked then JogRun(ZERO_RATES);
  allZero := True;
  for i := 0 to AXIS_COUNT - 1 do
    if FJogVel[i] <> 0 then allZero := False;
  if allZero or (not FLinked) or (not FPort.Connected) then JogReset;
end;

{ Main service, called on every update cycle: reads the card status, inputs and alarms,
  updates the link and copies the step counters to the Mach3 DROs. }
procedure TCardLayer.UpdatePosition;
var
  d: TUnoStatus;
  no: DWord;
  i, droPos: Integer;
begin
  ServiceJogStop;
  FPort.Service;
  CheckCardInfo;
  no := FPort.GetStatus(d);
  if no <> FStNo then
  begin
    FStNo := no;
    FSt := d;
    if FHost.TransferInputs(FSt.Inputs) and (not InReset) then
      EnterReset(RESET_REASON_LIMIT);
  end;
  CheckCardAlarm;
  RefreshOut;
  TryLink;
  CheckConnection;
  SyncOutputs;
  ConnectionMessages;
  if FHost.HasPosition then
  begin
    for i := 0 to AXIS_COUNT - 1 do
    begin
      droPos := FHost.ReadPosition(i);
      if droPos <> FLastWritten[i] then
      begin
        FSteps[i] := FSteps[i] + (droPos - FLastWritten[i]);
        FRequested[i] := FRequested[i] + (droPos - FLastWritten[i]);
      end;
    end;
  end;
  ApplyStop;
  for i := 0 to AXIS_COUNT - 1 do
  begin
    if FHost.HasPosition then FHost.WritePosition(i, FSteps[i]);
    FLastWritten[i] := FSteps[i];
  end;
  FPerUpdate := 0;
end;

end.
