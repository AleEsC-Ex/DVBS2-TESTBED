function server = dvbs2TCPServerRetry(host, port, description)
%DVBS2TCPSERVERRETRY Open a tcpserver, retrying if the port is still in use.
%
%   server = dvbs2TCPServerRetry(host, port, description) repeatedly
%   attempts tcpserver(host, port) until it succeeds.
%
%   WHY THIS EXISTS: when a script is stopped abruptly (Ctrl+C, closing
%   the MATLAB window, a crash) while a tcpserver object is bound to a
%   port, clearing/re-running the script releases MATLAB's own handle
%   to it, but the operating system itself still holds a just-closed
%   TCP port in a brief TIME_WAIT state before it's reusable -- trying
%   to bind a new server to the same port immediately after can fail
%   with "address already in use" even though nothing in MATLAB is
%   still holding it. Retrying here rides out that OS-level delay
%   instead of making you wait it out by hand or restart MATLAB.
%
%   DESCRIPTION is a short string used only in the printed status
%   message while waiting (e.g. "the feedback server").

    attempt = 0;
    while true
        attempt = attempt + 1;
        try
            server = tcpserver(host, port);
            return;
        catch ME
            if mod(attempt, 25) == 1
                fprintf('  ... %s (port %d) not available yet (%s); retrying (attempt %d)\n', ...
                    description, port, ME.message, attempt);
            end
            pause(0.2);
        end
    end
end
