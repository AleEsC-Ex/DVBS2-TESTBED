function client = dvbs2TCPConnectRetry(host, port, description)
%DVBS2TCPCONNECTRETRY Connect as a tcpclient, retrying until the server is listening.
%
%   client = dvbs2TCPConnectRetry(host, port, description) repeatedly
%   attempts tcpclient(host, port) until it succeeds. Each testbed
%   script acts as a TCP server for some links and a client for others;
%   retrying here means none of them has to be started in a strict
%   order relative to the others -- whichever needs a server that isn't
%   up yet just waits for it.
%
%   description is a short string used only in the printed status
%   message while waiting (e.g. "the simulated-channel server").
%
%   Retries every 0.5 s and prints a status line every 10 attempts
%   (~5 s) so a genuinely unreachable server doesn't fail silently.

    attempt = 0;
    while true
        attempt = attempt + 1;
        try
            client = tcpclient(host, port);
            return;
        catch
            if mod(attempt, 10) == 1
                fprintf('  ... still waiting for %s at %s:%d (attempt %d)\n', ...
                    description, host, port, attempt);
            end
            pause(0.5);
        end
    end
end
