%CHECKRADIOS Enumerate reachable USRP radios and confirm both configured IP addresses respond.
%
%   Run this first, before either of the tone-test scripts in this
%   folder: it only queries the radios over the network (via UHD) and
%   never streams an RF signal, so it's safe to run regardless of what's
%   connected to the RF ports. A radio that doesn't show up here won't
%   work in the scripts that actually transmit/receive.
%
%   Requires the Communications Toolbox Support Package for USRP Radio
%   (which provides findsdru) to be installed.

clear; clc;

addpath(fileparts(mfilename('fullpath')));
config = sdrTestConfig();

% findsdru's exact return fields can vary by support-package release --
% VERIFY the field names below (Platform/Status/IPAddress/SerialNum)
% against your installed version if this errors.
fprintf('Radios found on this network:\n');
radioList = findsdru();
for k = 1:numel(radioList)
    fprintf('  [%d] Platform=%s  Status=%s  IPAddress=%s  SerialNum=%s\n', ...
        k, radioList(k).Platform, radioList(k).Status, radioList(k).IPAddress, radioList(k).SerialNum);
end

fprintf('\nChecking configured TX radio (%s) ...\n', config.tx.ipAddress);
txStatus = findsdru(config.tx.ipAddress);
reportRadioStatus('TX', config.tx, txStatus);

fprintf('Checking configured RX radio (%s) ...\n', config.rx.ipAddress);
rxStatus = findsdru(config.rx.ipAddress);
reportRadioStatus('RX', config.rx, rxStatus);

if strcmp(txStatus.Status, 'Success') && strcmp(rxStatus.Status, 'Success')
    fprintf('\nBoth radios reachable at the same time -- OK to proceed to the tone test.\n');
else
    fprintf('\nAt least one radio did NOT respond -- resolve this before the tone test.\n');
end

function reportRadioStatus(role, expected, status)
%REPORTRADIOSTATUS Print one radio's findsdru result and flag a platform-string mismatch.
if strcmp(status.Status, 'Success')
    fprintf('  %s radio OK: Platform=%s\n', role, status.Platform);
    if ~strcmp(status.Platform, expected.platform)
        warning('sdr_test:PlatformMismatch', ...
            '%s radio reports Platform=%s, configured Platform=%s -- update sdrTestConfig.m.', ...
            role, status.Platform, expected.platform);
    end
else
    fprintf('  %s radio NOT reachable (Status=%s).\n', role, status.Status);
end
end
