{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uSerialLink.pas - USB serial link to the card
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  Finds the card on any COM port, opens the port with overlapped I/O and
  runs separate reader and writer threads so that Mach3 never waits for USB.
  Checks every packet (length, checksum, sequence) and the 200 ms link
  timeout.
}
unit uSerialLink;

{$mode objfpc}{$H+}
{$codepage utf8}
{$R-}{$Q-}

interface

uses
  Windows, Classes, SysUtils, Registry, uWinApi, uMessages, uCardProtocol;

const
  SER_QUEUE_SIZE = 1024;
  SER_HISTORY = 1024;
  SER_RX_TIMEOUT_MS = 1500;
  SER_OPEN_WAIT_MS = 3000;
  SER_OPEN_WAIT_CARD_MS = 8000;
  SER_REOPEN_MS = 500;
  SER_PREF_RETRIES = 6;
  SER_OPEN_WAIT_FRESH_MS = 2500;
  SER_FRESH_WAIT_MS = 1000;
  SER_CFG_TIMEOUT_MS = 6000;
  SER_FRESH_RETRIES = 3;
  SER_WRITE_TIMEOUT_MS = 300;
  SER_WRITE_RETRIES = 3;
  SER_READ_TIMEOUT_MS = 25;
  SER_STATUS_MS = 50;
  SER_LINK_TIMEOUT_MS = 200;
  SER_LINK_GRACE_MS = 250;
  SER_BYTE_GAP_MS = 30;
  HS_PC = 0;
  HS_CARD = 1;
  READ_BUF_SIZE = 64;

type
  TSerialLink = class;

  { Thread that runs either the read loop or the write loop }
  TLinkThread = class(TThread)
  private
    FOwner: TSerialLink;
    FIsReader: Boolean;
  protected
    procedure Execute; override;
  public
    constructor Create(AOwner: TSerialLink; AIsReader: Boolean);
  end;

  TSerialLink = class
  private
    FLock: TRTLCriticalSection;
    FHandle: THandle;
    FReadOv: TOverlapped_;
    FWriteOv: TOverlapped_;
    FReadBuf: array[0..READ_BUF_SIZE - 1] of Byte;
    FWriteBuf: TPacket;
    FReadPending: Boolean;
    FWriter, FReader: TLinkThread;
    FThreadStop: Boolean;
    FWake: THandle;
    FStopping: Boolean;
    FConnected: Boolean;
    FSeen: Boolean;
    FWriting: Boolean;
    FPortName: string;
    FPreferred: string;
    FCandIdx: Integer;
    FWasSeen: Boolean;
    FOpenWait: QWord;
    FPrefMissing: Boolean;
    FPrefError: Integer;
    FFreshTries: Integer;
    FFreshAt: QWord;
    FPortList: TStringList;
    FNewPortNo: DWord;
    FCfgPending: Boolean;
    FCfgStart: QWord;
    FCfgTries: Integer;
    FLastError: UnicodeString;
    FQueue: array[0..SER_QUEUE_SIZE - 1] of TPacket;
    FHead, FCount: Integer;
    FGen: DWord;
    FWin: TPacket;
    FRxPacket: TPacket;
    FTxPacket: TPacket;
    FWinN: Integer;
    FBadBytes: DWord;
    FStat: TUnoStatus;
    FStatNo: DWord;
    FInfo: TUnoInfo;
    FInfoNo: DWord;
    FRxTick, FOpenTick, FLastTry: QWord;
    FUseQpc: Boolean;
    FSlice: Int64;
    FPerMs: Int64;
    FModelDepth: Integer;
    FModelT: Int64;
    FTimerOn: Boolean;
    FTxK, FTxH, FTxS, FTxO, FTxN: Byte;
    FConnectRequest: Boolean;
    FCardInReset: Boolean;
    FResetAck: Boolean;
    FStatusNow: Boolean;
    FLastStatusMs: Int64;
    FLinkMs: Int64;
    FRxMs: Int64;
    FLastByteMs: Int64;
    FRxN: Byte;
    FRxSync: Boolean;
    FErrNo, FErrTaken: DWord;
    FErrH, FErrSource: Byte;
    FSession: Boolean;
    FSessionCard: Boolean;
    FSessionH: Byte;
    FWritten: Int64;
    FLastSliceMs: Int64;
    FWrittenTotal: array[0..UNO_AXES - 1] of Int64;
    FAcceptedTotal: array[0..UNO_AXES - 1] of Int64;
    FHistory: array[0..SER_HISTORY - 1, 0..UNO_AXES - 1] of SmallInt;
    FDiff: array[0..UNO_AXES - 1] of Int64;
    FHasDiff: Boolean;
    FDiffUncertain: Boolean;

    procedure Lock; inline;
    procedure Unlock; inline;
    function QpcNow: Int64;
    function NowMs: Int64;
    function BuildCandidates: TStringList;
    procedure WatchPortList(l: TStringList);
    function PreferredListed: Boolean;
    procedure Fail(const msg: UnicodeString);
    function OpenPort(const name: string): Boolean;
    function TryConfigure: Boolean;
    procedure ClosePort;
    procedure DropQueue;
    procedure ModelDrain;
    procedure BeginSession;
    procedure EndSession(const d: TUnoStatus; report: Boolean);
    procedure SliceWritten(const p: TPacket);
    procedure LinkError(h, source: Byte);
    procedure Feed(b: Byte; t: Int64);
    procedure Accept(const p: TPacket; t: Int64);
    procedure AcceptInfo(const p: TPacket; t: Int64);
    procedure Supervise;
    function TakeStatus(var p: TPacket; out len: Integer): Boolean;
    function Pop(var p: TPacket; out gen: DWord): Boolean;
    function WritePacket(const p: TPacket; len: Integer; out err: DWord): Boolean;
    procedure WriteWithRetry(var p: TPacket; len: Integer; gen: DWord; slice: Boolean);
    function GetSeen: Boolean;
    function GetPortNameU: UnicodeString;
  public
    constructor Create;
    destructor Destroy; override;

    procedure Close;
    procedure Service;
    procedure LinkStart;
    procedure LinkReset(h: Byte);
    procedure SetOutputs(spindle, output: Byte);
    function TakeError(out h, source: Byte): Boolean;
    function TakeStop(out diff: TAxisInt; out uncertain: Boolean): Boolean;
    function MoveReady: Boolean;
    function SendSlice(x, y, z: Integer): Boolean;
    procedure Flush(timeoutMs: DWord);
    function Depth: Integer;
    function GetStatus(out d: TUnoStatus): DWord;
    function GetInfo(out info: TUnoInfo): DWord;
    procedure ReadPump;
    procedure CancelPendingRead;
    procedure Pump;

    property Connected: Boolean read FConnected;
    property Seen: Boolean read GetSeen;
    property PortName: UnicodeString read GetPortNameU;
    property NewPortCount: DWord read FNewPortNo;
    property LastError: UnicodeString read FLastError;
  end;

implementation

var
  gThrottleDone: Boolean = False;

{ ================= TLinkThread ================= }

constructor TLinkThread.Create(AOwner: TSerialLink; AIsReader: Boolean);
begin
  FOwner := AOwner;
  FIsReader := AIsReader;
  inherited Create(True);
  FreeOnTerminate := False;
end;

procedure TLinkThread.Execute;
begin
  while not FOwner.FThreadStop do
  begin
    try
      if FIsReader then
        FOwner.ReadPump
      else
        FOwner.Pump;
    except
    end;
  end;
  if FIsReader then
  begin
    try
      FOwner.CancelPendingRead;
    except
    end;
  end;
end;

{ ================= Helpers ================= }

{ Asks Windows not to throttle this process (power throttling delays packets). }
procedure ReleasePowerThrottling;
var
  st: TPowerThrottling_;
begin
  if gThrottleDone then Exit;
  gThrottleDone := True;
  try
    st.Version := 1;
    st.ControlMask := 1 or 4;
    st.StateMask := 0;
    SetProcessInformation_(GetCurrentProcess, PROCESS_POWER_THROTTLING_, @st, SizeOf(st));
  except
  end;
end;

{ USB serial drivers (Arduino USBSER, CH340, CP210x, PL2303, FTDI VCP, ...) }
function IsUsbSerial(const path: string): Boolean;
begin
  Result := (Pos('USBSER', path) > 0) or (Pos('VCP', path) > 0) or (Pos('CH34', path) > 0) or
    (Pos('WCH', path) > 0) or (Pos('SILAB', path) > 0) or (Pos('CP210', path) > 0) or
    (Pos('PROLIFIC', path) > 0) or (Pos('PL2303', path) > 0) or (Pos('USB', path) > 0);
end;

{ Lists the COM ports from the registry (HARDWARE\DEVICEMAP\SERIALCOMM).
  USB serial ports come first. }
procedure ListComPorts(l: TStringList);
var
  r: TRegistry;
  names, usb, other: TStringList;
  i: Integer;
  key, data, path, s: string;
begin
  l.Clear;
  r := TRegistry.Create(KEY_READ);
  names := TStringList.Create;
  usb := TStringList.Create;
  other := TStringList.Create;
  usb.CaseSensitive := True;
  other.CaseSensitive := True;
  try
    try
      r.RootKey := HKEY_LOCAL_MACHINE;
      if not r.OpenKeyReadOnly('HARDWARE\DEVICEMAP\SERIALCOMM') then Exit;
      r.GetValueNames(names);
      for i := 0 to names.Count - 1 do
      begin
        if i >= 256 then Break;
        key := names[i];
        if r.GetDataType(key) <> rdString then Continue;
        data := r.ReadString(key);
        if data = '' then Continue;
        path := UpperCase(key);
        s := UpperCase(Trim(data));
        if Copy(s, 1, 3) <> 'COM' then Continue;
        if Pos('BTH', path) > 0 then Continue;
        if (usb.IndexOf(s) >= 0) or (other.IndexOf(s) >= 0) then Continue;
        if IsUsbSerial(path) then usb.Add(s) else other.Add(s);
      end;
      l.AddStrings(usb);
      l.AddStrings(other);
    except
    end;
  finally
    other.Free;
    usb.Free;
    names.Free;
    r.Free;
  end;
end;

function NewList: TStringList;
begin
  Result := TStringList.Create;
  Result.CaseSensitive := True;
end;

{ ================= TSerialLink ================= }

constructor TSerialLink.Create;
var
  f: Int64;
begin
  inherited Create;
  InitCriticalSection(FLock);
  FHandle := INVALID_HANDLE_VALUE;
  FWake := w_CreateEventW(nil, False, False, nil);
  FillChar(FReadOv, SizeOf(FReadOv), 0);
  FillChar(FWriteOv, SizeOf(FWriteOv), 0);
  f := 0;
  FUseQpc := w_QueryPerformanceFrequency(f);
  FUseQpc := FUseQpc and (f >= 1000);
  if FUseQpc then
  begin
    FSlice := f div 200;
    FPerMs := f div 1000;
  end
  else
  begin
    FSlice := 5;
    FPerMs := 1;
  end;
  FModelT := QpcNow;
  FTxK := K_RESET;
  FTxH := H_POWERON;
  FInfo := DefaultInfo;
end;

destructor TSerialLink.Destroy;
begin
  ClosePort;
  if FWake <> 0 then
  begin
    CloseHandle(FWake);
    FWake := 0;
  end;
  FreeAndNil(FPortList);
  DoneCriticalSection(FLock);
  inherited Destroy;
end;

procedure TSerialLink.Lock; inline;
begin
  EnterCriticalSection(FLock);
end;

procedure TSerialLink.Unlock; inline;
begin
  LeaveCriticalSection(FLock);
end;

function TSerialLink.GetSeen: Boolean;
begin
  Lock;
  try
    Result := FSeen and FConnected;
  finally
    Unlock;
  end;
end;

function TSerialLink.GetPortNameU: UnicodeString;
begin
  Result := UnicodeString(FPortName);
end;

function TSerialLink.QpcNow: Int64;
var
  v: Int64;
begin
  if FUseQpc then
  begin
    v := 0;
    w_QueryPerformanceCounter(v);
    Result := v;
  end
  else
    Result := Int64(TickMs64);
end;

function TSerialLink.NowMs: Int64;
begin
  Result := QpcNow div FPerMs;
end;

{ List of the ports to try: the last used port first, then the others. }
function TSerialLink.BuildCandidates: TStringList;
var
  l: TStringList;
  i: Integer;
begin
  Result := NewList;
  l := NewList;
  try
    ListComPorts(l);
    WatchPortList(l);
    if FPrefMissing and (FPreferred <> '') and (l.IndexOf(FPreferred) >= 0) then
    begin
      FFreshTries := SER_FRESH_RETRIES;
      FFreshAt := TickMs64;
    end;
    FPrefMissing := (FPreferred <> '') and (l.IndexOf(FPreferred) < 0);
    if (not FPrefMissing) and (FPreferred <> '') then Result.Add(FPreferred);
    for i := 0 to l.Count - 1 do
      if Result.IndexOf(l[i]) < 0 then Result.Add(l[i]);
  finally
    l.Free;
  end;
end;

{ Detects newly plugged USB serial ports. }
procedure TSerialLink.WatchPortList(l: TStringList);
var
  i: Integer;
begin
  if FPortList = nil then
  begin
    FPortList := NewList;
    FPortList.AddStrings(l);
    Exit;
  end;
  for i := 0 to l.Count - 1 do
    if FPortList.IndexOf(l[i]) < 0 then
      Inc(FNewPortNo);
  FPortList.Clear;
  FPortList.AddStrings(l);
end;

function TSerialLink.PreferredListed: Boolean;
var
  l: TStringList;
begin
  Result := False;
  if FPreferred = '' then Exit;
  l := NewList;
  try
    ListComPorts(l);
    Result := l.IndexOf(FPreferred) >= 0;
  finally
    l.Free;
  end;
end;

procedure TSerialLink.Fail(const msg: UnicodeString);
begin
  Lock;
  try
    LinkError(H_TIMEOUT, HS_PC);
  finally
    Unlock;
  end;
  FLastError := msg;
  FConnected := False;
end;

{ Opens the COM port for overlapped (asynchronous) I/O. }
function TSerialLink.OpenPort(const name: string): Boolean;
var
  devicePath: AnsiString;
begin
  ClosePort;
  FPortName := name;
  devicePath := AnsiString('\\.\' + name);
  FHandle := w_CreateFileA(PAnsiChar(devicePath), GENERIC_READ or GENERIC_WRITE, 0, nil,
    OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
  if FHandle = INVALID_HANDLE_VALUE then
  begin
    FLastError := SerOpenFailed + UnicodeString(name);
    Exit(False);
  end;
  w_EscapeCommFunction(FHandle, CLRDTR_);
  w_EscapeCommFunction(FHandle, CLRRTS_);
  FCfgPending := True;
  FCfgStart := TickMs64;
  FCfgTries := 0;
  TryConfigure;
  Result := True;
end;

{ Sets 115200 baud, 8N1, no flow control and the timeouts.
  Retried, because some USB drivers need time after the port has appeared. }
function TSerialLink.TryConfigure: Boolean;
var
  dcb: TDCB_;
  tmo: TCommTimeouts_;
begin
  Result := False;
  if (not FCfgPending) or (FHandle = INVALID_HANDLE_VALUE) then Exit;
  Inc(FCfgTries);
  w_SetupComm(FHandle, 4096, 4096);
  FillChar(dcb, SizeOf(dcb), 0);
  dcb.DCBlength := SizeOf(TDCB_);
  if not w_GetCommState(FHandle, @dcb) then
  begin
    FLastError := SerGetCommState + UnicodeString(FPortName);
    Exit;
  end;
  dcb.BaudRate := UNO_BAUD;
  dcb.ByteSize := 8;
  dcb.Parity := 0;
  dcb.StopBits := 0;
  dcb.Flags := 1;
  if not w_SetCommState(FHandle, @dcb) then
  begin
    FLastError := SerSetCommState + UnicodeString(FPortName);
    Exit;
  end;
  FCfgPending := False;
  w_EscapeCommFunction(FHandle, CLRDTR_);
  w_EscapeCommFunction(FHandle, CLRRTS_);
  tmo.ReadIntervalTimeout := $FFFFFFFF;
  tmo.ReadTotalTimeoutMultiplier := $FFFFFFFF;
  tmo.ReadTotalTimeoutConstant := 20;
  tmo.WriteTotalTimeoutMultiplier := 0;
  tmo.WriteTotalTimeoutConstant := 0;
  w_SetCommTimeouts(FHandle, @tmo);
  w_PurgeComm(FHandle, PURGE_ALL_);
  FillChar(FReadOv, SizeOf(FReadOv), 0);
  FillChar(FWriteOv, SizeOf(FWriteOv), 0);
  FReadOv.hEvent := w_CreateEventW(nil, True, False, nil);
  FWriteOv.hEvent := w_CreateEventW(nil, True, False, nil);
  FReadPending := False;
  FLastError := '';
  Lock;
  try
    FWinN := 0;
    FBadBytes := 0;
    FSeen := False;
    FRxTick := TickMs64;
    FRxMs := NowMs;
    FHead := 0;
    FCount := 0;
    FModelDepth := 0;
    FWriting := False;
    Inc(FGen);
    FTxK := K_RESET;
    FTxH := H_POWERON;
    FConnectRequest := False;
    FCardInReset := False;
    FResetAck := False;
    FSession := False;
    FStatusNow := False;
    FLastStatusMs := 0;
    FRxSync := False;
  finally
    Unlock;
  end;
  FOpenTick := TickMs64;
  FStopping := False;
  FConnected := True;
  w_timeBeginPeriod(1);
  FTimerOn := True;
  ReleasePowerThrottling;
  FThreadStop := False;
  FReader := TLinkThread.Create(Self, True);
  FReader.Priority := tpHigher;
  FWriter := TLinkThread.Create(Self, False);
  FWriter.Priority := tpHigher;
  FReader.Start;
  FWriter.Start;
  Result := True;
end;

procedure TSerialLink.ClosePort;
begin
  FStopping := True;
  if (FWriter <> nil) or (FReader <> nil) then
  begin
    FThreadStop := True;
    SetEvent(FWake);
    if FWriter <> nil then
    begin
      FWriter.WaitFor;
      FreeAndNil(FWriter);
    end;
    if FReader <> nil then
    begin
      FReader.WaitFor;
      FreeAndNil(FReader);
    end;
  end;
  if FTimerOn then
  begin
    w_timeEndPeriod(1);
    FTimerOn := False;
  end;
  FConnected := False;
  FCfgPending := False;
  if FHandle <> INVALID_HANDLE_VALUE then
  begin
    w_CancelIo(FHandle);
    CloseHandle(FHandle);
    FHandle := INVALID_HANDLE_VALUE;
  end;
  if FReadOv.hEvent <> 0 then CloseHandle(FReadOv.hEvent);
  if FWriteOv.hEvent <> 0 then CloseHandle(FWriteOv.hEvent);
  FReadOv.hEvent := 0;
  FWriteOv.hEvent := 0;
  FReadPending := False;
  Lock;
  try
    FSeen := False;
    EndSession(EmptyStatus, False);
    DropQueue;
    FTxK := K_RESET;
    FConnectRequest := False;
  finally
    Unlock;
  end;
end;

procedure TSerialLink.Close;
begin
  ClosePort;
end;

{ Called on every update cycle: opens / searches for the port, checks the timeouts
  and closes the port when the card stops answering. }
procedure TSerialLink.Service;
var
  t, rx: QWord;
  seenNow: Boolean;
  l: TStringList;
  cand: string;
begin
  t := TickMs64;
  if FCfgPending then
  begin
    if TryConfigure then Exit;
    if t - FCfgStart < SER_CFG_TIMEOUT_MS then Exit;
    ClosePort;
    if FPortName = FPreferred then
    begin
      FCandIdx := 0;
      FFreshTries := SER_FRESH_RETRIES;
      FFreshAt := t;
    end;
    FLastTry := t;
    Exit;
  end;
  if FConnected then
  begin
    Lock;
    try
      seenNow := FSeen;
      rx := FRxTick;
    finally
      Unlock;
    end;
    if seenNow then
    begin
      FWasSeen := True;
      FFreshTries := 0;
      if FPreferred <> FPortName then
      begin
        FPreferred := FPortName;
        FCandIdx := 0;
      end;
      if (t > rx) and (t - rx > SER_RX_TIMEOUT_MS) then Fail(SerNoData);
    end
    else if t - FOpenTick > FOpenWait then
      Fail(SerNoReply + UnicodeString(FPortName))
    else if FPrefMissing and PreferredListed then
    begin
      FConnected := False;
      FLastTry := 0;
      FCandIdx := 0;
      FPrefError := 0;
      FFreshTries := SER_FRESH_RETRIES;
      FFreshAt := t;
    end;
  end;
  if FConnected then Exit;
  if (FHandle <> INVALID_HANDLE_VALUE) or (FReader <> nil) or (FWriter <> nil) then
  begin
    ClosePort;
    FLastTry := 0;
  end;
  if (FLastTry <> 0) and (t - FLastTry < SER_REOPEN_MS) then Exit;
  FLastTry := t;
  l := BuildCandidates;
  try
    if l.Count = 0 then
    begin
      FLastError := SerNoPort;
      Exit;
    end;
    if FWasSeen then
    begin
      FWasSeen := False;
      FCandIdx := 0;
    end;
    cand := l[FCandIdx mod l.Count];
  finally
    l.Free;
  end;
  if (cand = FPreferred) and (FFreshTries = SER_FRESH_RETRIES) and (FFreshAt <> 0) and
     (t - FFreshAt < SER_FRESH_WAIT_MS) then Exit;
  if (cand = FPreferred) and (FFreshTries > 0) then
  begin
    FOpenWait := SER_OPEN_WAIT_FRESH_MS;
    Dec(FFreshTries);
  end
  else if cand = FPreferred then
    FOpenWait := SER_OPEN_WAIT_CARD_MS
  else
    FOpenWait := SER_OPEN_WAIT_MS;
  if OpenPort(cand) then
  begin
    FPrefError := 0;
    Inc(FCandIdx);
  end
  else if (cand = FPreferred) and (FPrefError < SER_PREF_RETRIES) then
  begin
    Inc(FPrefError);
    FFreshTries := SER_FRESH_RETRIES;
    FFreshAt := t;
  end
  else
  begin
    FPrefError := 0;
    Inc(FCandIdx);
  end;
end;

procedure TSerialLink.DropQueue;
begin
  FHead := 0;
  FCount := 0;
  FModelDepth := 0;
  Inc(FGen);
end;

{ Estimates how many slices are left in the card buffer (5 ms per slice). }
procedure TSerialLink.ModelDrain;
var
  now, sl: Int64;
begin
  now := QpcNow;
  if FModelDepth <= 0 then
  begin
    FModelDepth := 0;
    FModelT := now;
    Exit;
  end;
  sl := (now - FModelT) div FSlice;
  if sl <= 0 then Exit;
  if sl >= FModelDepth then
  begin
    FModelDepth := 0;
    FModelT := now;
  end
  else
  begin
    Dec(FModelDepth, Integer(sl));
    FModelT := FModelT + sl * FSlice;
  end;
end;

{ Starts a motion session: clears the history used to compute the stop position. }
procedure TSerialLink.BeginSession;
var
  i: Integer;
begin
  FSession := True;
  FSessionCard := False;
  FSessionH := $FF;
  FWritten := 0;
  FLastSliceMs := 0;
  for i := 0 to UNO_AXES - 1 do
  begin
    FWrittenTotal[i] := 0;
    FAcceptedTotal[i] := 0;
  end;
end;

{ Ends the session when the card stops: from the stop slice reported by the card,
  computes how many steps were really executed on each axis. }
procedure TSerialLink.EndSession(const d: TUnoStatus; report: Boolean);
var
  s, f, l, k: Int64;
  valid: Boolean;
  executed: array[0..UNO_AXES - 1] of Int64;
  i: Integer;
begin
  if not FSession then Exit;
  FSession := False;
  if not FSessionCard then Exit;
  s := FWritten;
  f := 0;
  valid := report and (d.SubStep < SubStepScale) and
    ((d.Error <> H_POWERON) or (FSessionH = H_POWERON));
  if valid then
  begin
    l := s - ((s - d.SliceCount) and $FFFF);
    f := l;
    if d.SubStep > 0 then Dec(f);
    valid := (f >= 0) and (s - f <= SER_HISTORY);
  end;
  for i := 0 to UNO_AXES - 1 do executed[i] := FWrittenTotal[i];
  if valid then
  begin
    k := f;
    while k < s do
    begin
      for i := 0 to UNO_AXES - 1 do
        executed[i] := executed[i] - FHistory[Integer(k mod SER_HISTORY), i];
      Inc(k);
    end;
    if d.SubStep > 0 then
      for i := 0 to UNO_AXES - 1 do
        executed[i] := executed[i] + PartialStep(FHistory[Integer(f mod SER_HISTORY), i], d.SubStep);
  end
  else if (FStat.State = K_MOVE) or (FStat.Fifo > 0) or (FCount > 0) or
          (NowMs - FLastSliceMs < SER_LINK_TIMEOUT_MS) then
    FDiffUncertain := True;
  for i := 0 to UNO_AXES - 1 do
    FDiff[i] := FDiff[i] + FAcceptedTotal[i] - executed[i];
  FHasDiff := True;
end;

procedure TSerialLink.SliceWritten(const p: TPacket);
var
  idx, i, v: Integer;
begin
  if not FSession then Exit;
  idx := Integer(FWritten mod SER_HISTORY);
  for i := 0 to UNO_AXES - 1 do
  begin
    v := PacketSteps(p, i);
    FHistory[idx, i] := SmallInt(v);
    FWrittenTotal[i] := FWrittenTotal[i] + v;
  end;
  Inc(FWritten);
  FLastSliceMs := NowMs;
end;

procedure TSerialLink.LinkError(h, source: Byte);
begin
  if FTxK = K_RESET then Exit;
  FTxK := K_RESET;
  FConnectRequest := False;
  if h > H_MAX then h := H_POWERON;
  FTxH := h;
  DropQueue;
  FErrH := h;
  FErrSource := source;
  Inc(FErrNo);
  FStatusNow := True;
  SetEvent(FWake);
end;

{ Takes one byte and assembles a packet. On a checksum mismatch it shifts by one
  byte and tries again, so the link resynchronises after noise. }
procedure TSerialLink.Feed(b: Byte; t: Int64);
var
  remaining, i: Integer;
  tmp: array of Byte;
begin
  if (FWinN > 0) and (t - FLastByteMs > SER_BYTE_GAP_MS) then
  begin
    FWinN := 0;
    Inc(FBadBytes);
    LinkError(H_BADLEN, HS_PC);
  end;
  FLastByteMs := t;
  if (FWinN = 0) and (PacketLength(b, False) = 0) then
  begin
    Inc(FBadBytes);
    LinkError(H_BADCMD, HS_PC);
    Exit;
  end;
  FWin[FWinN] := b;
  Inc(FWinN);
  if FWinN < UNO_STATUS_LEN then Exit;
  FRxPacket := FWin;
  if ChecksumOk(FRxPacket, UNO_STATUS_LEN) then
  begin
    FWinN := 0;
    Accept(FRxPacket, t);
    Exit;
  end;
  Inc(FBadBytes);
  LinkError(H_CHECKSUM, HS_PC);
  remaining := FWinN - 1;
  SetLength(tmp, remaining);
  for i := 0 to remaining - 1 do tmp[i] := FWin[i + 1];
  FWinN := 0;
  for i := 0 to remaining - 1 do Feed(tmp[i], t);
end;

{ Handles a valid status packet: sequence check, card reset and errors. }
procedure TSerialLink.Accept(const p: TPacket; t: Int64);
var
  d: TUnoStatus;
begin
  if p[0] = K_INFO then
  begin
    AcceptInfo(p, t);
    Exit;
  end;
  if not DecodePacket(p, d) then Exit;
  if FRxSync and (FTxK <> K_RESET) and (d.State <> K_RESET) and (d.Counter <> FRxN) then
    LinkError(H_SEQ, HS_PC);
  FRxN := Byte(d.Counter + 1);
  FRxSync := True;
  FSeen := True;
  FRxTick := TickMs64;
  FRxMs := t;
  if d.State = K_RESET then
  begin
    if not FCardInReset then
    begin
      FCardInReset := True;
      FResetAck := False;
      EndSession(d, True);
      LinkError(d.Error, HS_CARD);
    end
    else if (FTxK <> K_RESET) and (t - FLinkMs > SER_LINK_GRACE_MS) then
      LinkError(d.Error, HS_CARD);
  end
  else
  begin
    FCardInReset := False;
    FResetAck := False;
    if FSession then FSessionCard := True;
  end;
  FStat := d;
  Inc(FStatNo);
  if d.State = K_RESET then
    FModelDepth := FCount
  else
    FModelDepth := d.Fifo + FCount;
  FModelT := QpcNow;
end;

{ K_INFO from the card: its speed limit. The card sends it only in RESET
  (power-up, link timeout, first K_RESET of a connection), so the sequence
  counter is followed but not checked, like a RESET status packet. }
procedure TSerialLink.AcceptInfo(const p: TPacket; t: Int64);
var
  info: TUnoInfo;
begin
  if not DecodeInfo(p, info) then
  begin
    Inc(FBadBytes);
    Exit;
  end;
  FRxN := Byte(p[1] + 1);
  FRxSync := True;
  FSeen := True;
  FRxTick := TickMs64;
  FRxMs := t;
  FInfo := info;
  Inc(FInfoNo);
end;

{ Link watchdog: incomplete packets and the 200 ms receive timeout. }
procedure TSerialLink.Supervise;
var
  t: Int64;
begin
  t := NowMs;
  Lock;
  try
    if (FWinN > 0) and (t - FLastByteMs > SER_BYTE_GAP_MS) then
    begin
      FWinN := 0;
      Inc(FBadBytes);
      LinkError(H_BADLEN, HS_PC);
    end;
    if (FTxK <> K_RESET) and (t - FRxMs > SER_LINK_TIMEOUT_MS) then
      LinkError(H_TIMEOUT, HS_PC);
  finally
    Unlock;
  end;
end;

procedure TSerialLink.LinkStart;
begin
  Lock;
  try
    FConnectRequest := True;
    if FTxK = K_RESET then FStatusNow := True;
  finally
    Unlock;
  end;
  SetEvent(FWake);
end;

procedure TSerialLink.LinkReset(h: Byte);
begin
  Lock;
  try
    FConnectRequest := False;
    DropQueue;
    if FTxK <> K_RESET then
    begin
      FTxK := K_RESET;
      if h > H_MAX then h := H_POWERON;
      FTxH := h;
      FStatusNow := True;
    end;
  finally
    Unlock;
  end;
  SetEvent(FWake);
end;

procedure TSerialLink.SetOutputs(spindle, output: Byte);
var
  changed: Boolean;
begin
  output := output and O_MASK;
  Lock;
  try
    changed := (spindle <> FTxS) or (output <> FTxO);
    if changed then
    begin
      FTxS := spindle;
      FTxO := output;
      if FTxK <> K_RESET then FStatusNow := True;
    end;
  finally
    Unlock;
  end;
  if changed then SetEvent(FWake);
end;

function TSerialLink.TakeError(out h, source: Byte): Boolean;
begin
  Lock;
  try
    Result := FErrNo <> FErrTaken;
    h := FErrH;
    source := FErrSource;
    FErrTaken := FErrNo;
  finally
    Unlock;
  end;
end;

function TSerialLink.TakeStop(out diff: TAxisInt; out uncertain: Boolean): Boolean;
var
  i: Integer;
begin
  Lock;
  try
    Result := FHasDiff;
    uncertain := FDiffUncertain;
    for i := 0 to UNO_AXES - 1 do
    begin
      diff[i] := LongInt(FDiff[i]);
      FDiff[i] := 0;
    end;
    FHasDiff := False;
    FDiffUncertain := False;
  finally
    Unlock;
  end;
end;

function TSerialLink.MoveReady: Boolean;
begin
  Lock;
  try
    Result := FConnected and (FTxK = K_ACTIVE) and FSessionCard and
      ((FStat.State = K_ACTIVE) or (FStat.State = K_MOVE));
  finally
    Unlock;
  end;
end;

{ Queues one move packet (X, Y, Z steps for the next 5 ms). }
function TSerialLink.SendSlice(x, y, z: Integer): Boolean;
var
  idx, i: Integer;
begin
  Result := False;
  if not FConnected then Exit;
  Lock;
  try
    if (FTxK <> K_ACTIVE) or (not FSessionCard) then Exit;
    if FCount >= SER_QUEUE_SIZE then Exit;
    idx := (FHead + FCount) mod SER_QUEUE_SIZE;
    MovePacket(FQueue[idx], x, y, z);
    for i := 0 to UNO_AXES - 1 do
      FAcceptedTotal[i] := FAcceptedTotal[i] + PacketSteps(FQueue[idx], i);
    Inc(FCount);
    ModelDrain;
    Inc(FModelDepth);
  finally
    Unlock;
  end;
  SetEvent(FWake);
  Result := True;
end;

procedure TSerialLink.Flush(timeoutMs: DWord);
var
  t: QWord;
  busy: Boolean;
begin
  t := TickMs64;
  repeat
    Lock;
    try
      busy := (FCount > 0) or FWriting or (FStatusNow and FSeen);
    finally
      Unlock;
    end;
    if (not busy) or (not FConnected) then Exit;
    Sleep(2);
  until TickMs64 - t >= timeoutMs;
end;

function TSerialLink.Depth: Integer;
begin
  Lock;
  try
    ModelDrain;
    Result := FModelDepth;
  finally
    Unlock;
  end;
end;

function TSerialLink.GetStatus(out d: TUnoStatus): DWord;
begin
  Lock;
  try
    d := FStat;
    Result := FStatNo;
  finally
    Unlock;
  end;
end;

{ Card limits in use (defaults until K_INFO arrives). The result changes every
  time new limits are stored, so the caller can see that they must be applied. }
function TSerialLink.GetInfo(out info: TUnoInfo): DWord;
begin
  Lock;
  try
    info := FInfo;
    Result := FInfoNo;
  finally
    Unlock;
  end;
end;

{ Reads all bytes received so far (overlapped ReadFile). }
procedure TSerialLink.ReadPump;
var
  done: Boolean;
  n, err, waitRes: DWord;
  t: Int64;
  i: Integer;
begin
  if (not FConnected) or (FHandle = INVALID_HANDLE_VALUE) then
  begin
    Sleep(10);
    Exit;
  end;
  done := False;
  n := 0;
  if not FReadPending then
  begin
    ResetEvent(FReadOv.hEvent);
    if w_ReadFile(FHandle, @FReadBuf[0], READ_BUF_SIZE, nil, @FReadOv) then
    begin
      if w_GetOverlappedResult(FHandle, @FReadOv, n, False) then
        done := True
      else
      begin
        Fail(SerReadError + UnicodeString(IntToStr(GetLastError)));
        Exit;
      end;
    end
    else
    begin
      err := GetLastError;
      if err = ERROR_IO_PENDING_ then
        FReadPending := True
      else
      begin
        Fail(SerReadError + UnicodeString(IntToStr(err)));
        Exit;
      end;
    end;
  end;
  if FReadPending then
  begin
    waitRes := WaitForSingleObject(FReadOv.hEvent, SER_READ_TIMEOUT_MS);
    if waitRes = WAIT_OBJECT_0_ then
    begin
      FReadPending := False;
      n := 0;
      if w_GetOverlappedResult(FHandle, @FReadOv, n, False) then
        done := True
      else
      begin
        Fail(SerReadError + UnicodeString(IntToStr(GetLastError)));
        Exit;
      end;
    end
    else if waitRes <> WAIT_TIMEOUT_ then
    begin
      Fail(SerReadWaitError + UnicodeString(IntToStr(GetLastError)));
      Exit;
    end;
  end;
  if done and (n > 0) then
  begin
    t := NowMs;
    Lock;
    try
      for i := 0 to Integer(n) - 1 do Feed(FReadBuf[i], t);
    finally
      Unlock;
    end;
  end;
end;

procedure TSerialLink.CancelPendingRead;
var
  n: DWord;
begin
  if FReadPending and (FHandle <> INVALID_HANDLE_VALUE) then
  begin
    w_CancelIo(FHandle);
    n := 0;
    w_GetOverlappedResult(FHandle, @FReadOv, n, True);
  end;
  FReadPending := False;
end;

function TSerialLink.TakeStatus(var p: TPacket; out len: Integer): Boolean;
var
  t: Int64;
begin
  Result := False;
  len := UNO_STATUS_LEN;
  t := NowMs;
  Lock;
  try
    if not FSeen then Exit;
    if (FTxK = K_RESET) and FConnectRequest and FResetAck then
    begin
      FTxK := K_ACTIVE;
      FTxH := H_POWERON;
      FResetAck := False;
      FLinkMs := t;
      FRxMs := t;
      FStatusNow := True;
      BeginSession;
    end;
    if (not FStatusNow) and (t - FLastStatusMs < SER_STATUS_MS) then Exit;
    FStatusNow := False;
    FLastStatusMs := t;
    StatePacket(p, FTxK, FTxH, FTxS, FTxO);
    if FTxK = K_RESET then
    begin
      if FSession and (FSessionH = $FF) then FSessionH := FTxH;
      if FCardInReset and (not FResetAck) then
      begin
        FResetAck := True;
        if FConnectRequest then FStatusNow := True;
      end;
    end;
    FWriting := True;
    Result := True;
  finally
    Unlock;
  end;
end;

function TSerialLink.Pop(var p: TPacket; out gen: DWord): Boolean;
begin
  Result := False;
  Lock;
  try
    gen := FGen;
    if FCount <= 0 then Exit;
    p := FQueue[FHead];
    FHead := (FHead + 1) mod SER_QUEUE_SIZE;
    Dec(FCount);
    FWriting := True;
    Result := True;
  finally
    Unlock;
  end;
end;

function TSerialLink.WritePacket(const p: TPacket; len: Integer; out err: DWord): Boolean;
var
  n: DWord;
begin
  Result := False;
  err := 0;
  Move(p[0], FWriteBuf[0], len);
  ResetEvent(FWriteOv.hEvent);
  if w_WriteFile(FHandle, @FWriteBuf[0], DWord(len), nil, @FWriteOv) then Exit(True);
  err := GetLastError;
  if err = ERROR_IO_PENDING_ then
  begin
    n := 0;
    if WaitForSingleObject(FWriteOv.hEvent, SER_WRITE_TIMEOUT_MS) = WAIT_OBJECT_0_ then
    begin
      if w_GetOverlappedResult(FHandle, @FWriteOv, n, False) then
      begin
        err := 0;
        Exit(True);
      end;
      err := GetLastError;
    end
    else
    begin
      w_CancelIo(FHandle);
      if w_GetOverlappedResult(FHandle, @FWriteOv, n, True) then
      begin
        err := 0;
        Exit(True);
      end;
      err := WAIT_TIMEOUT_;
    end;
  end;
end;

{ Writes the packet; retries on a short timeout before reporting an error. }
procedure TSerialLink.WriteWithRetry(var p: TPacket; len: Integer; gen: DWord; slice: Boolean);
var
  attempt: Integer;
  err: DWord;
begin
  Lock;
  try
    if slice then
    begin
      if gen <> FGen then Exit;
      SliceWritten(p);
    end;
    p[1] := FTxN;
    FTxN := Byte(FTxN + 1);
    AddChecksum(p, len);
  finally
    Unlock;
  end;
  for attempt := 1 to SER_WRITE_RETRIES do
  begin
    if FStopping or (not FConnected) then Exit;
    if WritePacket(p, len, err) then Exit;
    if err <> WAIT_TIMEOUT_ then
    begin
      Fail(SerWriteError + UnicodeString(IntToStr(err)));
      Exit;
    end;
  end;
  Fail(SerWriteTimeout);
end;

procedure TSerialLink.Pump;
var
  written: Boolean;
  len: Integer;
  gen: DWord;
begin
  if (not FConnected) or (FHandle = INVALID_HANDLE_VALUE) then
  begin
    Sleep(10);
    Exit;
  end;
  Supervise;
  written := False;
  try
    if TakeStatus(FTxPacket, len) then
    begin
      written := True;
      WriteWithRetry(FTxPacket, len, 0, False);
    end
    else if Pop(FTxPacket, gen) then
    begin
      written := True;
      WriteWithRetry(FTxPacket, UNO_MOVE_LEN, gen, True);
    end;
  finally
    if written then
    begin
      Lock;
      try
        FWriting := False;
      finally
        Unlock;
      end;
    end;
  end;
  if not written then WaitForSingleObject(FWake, 2);
end;

end.
