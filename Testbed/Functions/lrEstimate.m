function fnorm = lrEstimate(y, M)
%LRESTIMATE Multi-lag Luise & Reggiannini normalized frequency estimator.
%
%   fnorm = lrEstimate(y, M) estimates the normalized frequency offset
%   (in cycles/sample) of a sequence y of complex samples that share a
%   common, unknown phase-modulation-free residual (e.g. a known
%   reference sequence's residual after removing the reference, or a
%   constant-modulus signal raised to a power that strips the data
%   modulation). It implements the classical Luise & Reggiannini (1995)
%   estimator, generalized to average over multiple correlation lags
%   1..M instead of just lag 1.
%
%   Using more lags (larger M) trades off unambiguous frequency capture
%   range for lower estimation variance: the unambiguous range shrinks
%   to +/-1/(M+1) cycles/sample (numerically verified: the summed-lag
%   phasor aliases just past this point), so M should be chosen based
%   on how large the expected residual frequency offset is at the point
%   this estimator is applied (e.g. small once a coarse correction stage
%   has already run first).
%
% Inputs:
%   y - Column (or row) vector of complex samples with the residual
%       frequency offset to be estimated.
%   M - Maximum correlation lag to average over (1 <= M <= length(y)-1).
%       M=1 reduces to the original single-lag Luise & Reggiannini
%       estimator.
%
% Output:
%   fnorm - Estimated normalized frequency offset, in cycles/sample.
%           Multiply by the sample rate to convert to Hz.

    N = length(y);
    Rsum = complex(0);
    for m = 1:M
        % Autocorrelation at lag m, averaged over the (N-m) valid
        % sample pairs, then summed across all lags 1..M.
        Rsum = Rsum + (1/(N-m)) * sum(y(m+1:end) .* conj(y(1:end-m)));
    end
    % The phase of the lag-averaged correlation grows linearly with
    % frequency offset; normalizing by pi*(M+1) recovers the frequency
    % in cycles/sample (see Luise & Reggiannini, IEEE Trans. Commun.,
    % 1995, for the derivation).
    fnorm = angle(Rsum) / (pi*(M+1));
end
