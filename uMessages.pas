{
 ============================================================================
  SafCNC - Mach3 motion plugin for SafCNC controller cards (Arduino Uno / Nano)
 ----------------------------------------------------------------------------
  File      : uMessages.pas - texts shown to the user
  Developer : Fikret Palas
  E-mail    : safcnc@tutamail.com
  Web       : https://github.com/fikretpalas
  Firmware  : https://github.com/fikretpalas/SafCNC_Arduino
  License   : MIT
  Version   : 1.0.0
  Cards     : Arduino Uno / Nano (about 30 kHz); STM32 up to 100 kHz (not tested)
 ============================================================================
  All user-visible texts are kept in this unit so that they are easy to
  translate.
}
unit uMessages;

{$mode objfpc}{$H+}
{$codepage utf8}

interface

const
  PluginName: UnicodeString = 'SafCNC';
  MainMenuName: UnicodeString = 'SafCNC';
  MenuName: UnicodeString = 'SafCNC';
  CardFormTitle: UnicodeString = 'SafCNC';

  MsgSearchingCard: UnicodeString = 'Searching for the SafCNC card...';
  MsgNewConnection: UnicodeString = 'New USB device found - checking whether it is a SafCNC card...';
  MsgCardConnected: UnicodeString = 'SafCNC card connected.';
  MsgCardNotFound: UnicodeString = 'SafCNC card not found - check the USB cable and the port';
  MsgConnectionLost: UnicodeString = 'Connection to the SafCNC card was lost';
  MsgOppositeKeys: UnicodeString = 'Opposite jog keys pressed - motion stopped. Release all jog keys.';
  MsgJogWindowStop: UnicodeString = 'Jog stopped - release all keys';
  MsgJogWindowMove: UnicodeString = 'Move cancelled - release all keys';
  MsgCardNotConnected: UnicodeString = 'SafCNC card is not connected - motion is disabled';
  MsgCardNotFoundReset: UnicodeString = 'SafCNC card is not connected - Mach3 was put into reset';
  MsgNoJogInReset: UnicodeString = 'Jogging is not possible while in E-Stop';
  MsgProbeNotEnabled: UnicodeString = 'Probe requested but the probe is not enabled';
  MsgNoProbeInReset: UnicodeString = 'Probing is not possible while in E-Stop';
  MsgProbeWait: UnicodeString = 'Waiting for the current operation to finish...';
  MsgOverrideExited: UnicodeString = 'Limit override mode ended';
  MsgCardSentReset: UnicodeString = 'The card requested a reset';
  MsgResetSentToCard: UnicodeString = 'Reset sent to the card';
  MsgPositionUncertain: UnicodeString = 'The card could not report where it stopped - axis positions may be wrong. Please home the machine.';
  MsgUnknownError: UnicodeString = 'unknown error code %d';
  MsgCardSpeed: UnicodeString = 'SafCNC card speed limit: %s kHz per axis (%d steps per 5 ms)';
  MsgVelocitiesTooHigh: UnicodeString = 'Axis velocities exceed the card limit - card not linked. Lower them in Motor Tuning.';
  MsgMoveDuringJog: UnicodeString = 'Move command received before the jog had stopped - command cancelled. Try again when the machine has stopped.';

  MsgError: array[0..13] of UnicodeString = (
    'power-on / normal reset',
    'E-Stop',
    'packet counter error',
    'checksum error',
    'unknown command',
    'incomplete packet',
    'communication timeout (200 ms)',
    'card buffer overflow',
    'step limit exceeded',
    'E-Stop input on the card is active',
    'command not allowed in this state',
    'limit switch triggered',
    'probe contact detected',
    'buffer underrun - motion data arrived too late');

  ErrStructure: UnicodeString = 'unexpected Mach3 data layout (the DLL must be built as 32-bit / x86).'#13#10 +
    'The plugin has been disabled.';
  ErrStatusWindow: UnicodeString = 'Status window error: ';

  FormCardStatus: UnicodeString = 'SafCNC card status';
  FormConnected: UnicodeString = 'Connected';
  FormUsbReady: UnicodeString = 'Card ready - Mach3 is in reset';
  FormCardNotFound: UnicodeString = 'Card not found';
  FormPortFormat: UnicodeString = 'Port: %s, %d baud';

  SpeedTitle: UnicodeString = 'SafCNC - axis velocities';
  SpeedPanel: UnicodeString = 'Mach3 axis velocities exceed the card limit';
  SpeedIntro: UnicodeString = 'The card can step at most %s kHz per axis (%d steps per 5 ms).';
  SpeedLine: UnicodeString = '%s axis:  %s units/min (%s kHz)  -  at most %s units/min';
  SpeedEnd: UnicodeString = 'The card is not linked with these velocities. Lower and save them in Config > Motor Tuning, then press Reset.';
  SpeedButtonOk: UnicodeString = 'OK';

  { Serial link error texts (internal use / debugging only) }
  SerOpenFailed: UnicodeString = 'could not open port: ';
  SerGetCommState: UnicodeString = 'GetCommState failed: ';
  SerSetCommState: UnicodeString = 'SetCommState failed: ';
  SerNoData: UnicodeString = 'no data from the card (timeout)';
  SerNoReply: UnicodeString = 'no card answering on this port: ';
  SerNoPort: UnicodeString = 'no serial port';
  SerReadError: UnicodeString = 'read error ';
  SerReadWaitError: UnicodeString = 'read wait error ';
  SerWriteError: UnicodeString = 'write error ';
  SerWriteTimeout: UnicodeString = 'write timeout (port does not accept data)';

{ Replaces the %s / %d placeholders in fmt with the items of args, in order. }
function FormatText(const fmt: UnicodeString; const args: array of UnicodeString): UnicodeString;

{ Message for a card reset code (H). }
function CardResetMessage(h: Byte; fromCard: Boolean): UnicodeString;

implementation

uses
  SysUtils;

function FormatText(const fmt: UnicodeString; const args: array of UnicodeString): UnicodeString;
var
  i, n: Integer;
begin
  Result := '';
  n := 0;
  i := 1;
  while i <= Length(fmt) do
  begin
    if (fmt[i] = '%') and (i < Length(fmt)) and
       ((fmt[i + 1] = 's') or (fmt[i + 1] = 'd')) then
    begin
      if n <= High(args) then
        Result := Result + args[n];
      Inc(n);
      Inc(i, 2);
    end
    else
    begin
      Result := Result + fmt[i];
      Inc(i);
    end;
  end;
end;

function CardResetMessage(h: Byte; fromCard: Boolean): UnicodeString;
var
  t, r: UnicodeString;
begin
  if h <= High(MsgError) then
    t := MsgError[h]
  else
    t := FormatText(MsgUnknownError, [UnicodeString(IntToStr(h))]);
  if fromCard then
    r := MsgCardSentReset
  else
    r := MsgResetSentToCard;
  Result := r + ' (code=' + UnicodeString(IntToStr(h)) + ': ' + t + ')';
end;

end.
