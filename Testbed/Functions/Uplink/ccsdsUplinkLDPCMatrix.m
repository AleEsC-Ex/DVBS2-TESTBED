function [H, decCfg] = ccsdsUplinkLDPCMatrix(codewordLength)
%CCSDSUPLINKLDPCMATRIX Parity-check matrix for the CCSDS TC LDPC codes.
%
%   [H, decCfg] = ccsdsUplinkLDPCMatrix(codewordLength) returns the sparse
%   parity-check matrix H for the CCSDS 231.0-B (128,64) or (512,256)
%   Telecommand LDPC code, and a matching ldpcDecoderConfig. Both are cached
%   -- building the decoder configuration is the expensive part and it never
%   changes.
%
%   STRUCTURE. Both codes are quasi-cyclic with the same 4x8 block layout,
%   differing only in the circulant size M (16 and 64) and the shift
%   exponents. Each block is either zero, a single cyclic shift Phi^k, or
%   the sum of the identity and one shift. Row weight is a uniform 8; column
%   weight is 3 or 5.
%
%   WHY IT IS BUILT HERE RATHER THAN TAKEN FROM THE TOOLBOX. MATLAB does
%   have these matrices, but only inside an undocumented internal package
%   that ccsdsTCIdealReceiver reaches into. Depending on that path would put
%   an unsupported, release-specific file on the critical path of the link.
%   Building H from the standard's own description costs a few lines and is
%   verifiable -- which it is: UplinkRxTest.m checks H against codewords
%   produced by ccsdsTCWaveform, and H*c' = 0 for every one of them is proof
%   that the matrix is right.
%
%   WHY WE WANT H AT ALL, rather than handing the burst to
%   ccsdsTCIdealReceiver. Owning the decode is what makes the last two
%   stages real:
%
%     - LLRs can be scaled from a noise variance the receiver actually
%       measured, off the start sequence. The previous chain could not, and
%       its LDPC results were measurably WORSE than BCH because the soft
%       decoder was being starved of correctly-scaled input.
%     - the parity check becomes a validity gate. A false detection almost
%       never yields a parity-consistent codeword, so it costs nothing and
%       rejects nearly every one -- this link has no CRC of its own.

    persistent cachedH cachedCfg cachedN

    if nargin < 1 || isempty(codewordLength)
        codewordLength = 128;
    end

    if ~isempty(cachedN) && cachedN == codewordLength
        H = cachedH;
        decCfg = cachedCfg;
        return;
    end

    % Circulant shift exponents, 4 block-rows by 8 block-columns. NaN marks
    % a zero block; a two-element entry {0,k} means I + Phi^k. Read straight
    % out of the standard's definition of H.
    switch codewordLength
        case 128
            M = 16;
            shifts = { [0 7], 2,      14,     6,      [],  0,   13,  0
                       6,      [0 15], 0,      1,      0,   [],  0,   7
                       4,      1,      [0 15], 14,     11,  0,   [],  3
                       0,      1,      9,      [0 13], 14,  1,   0,   []  };
        case 512
            M = 64;
            shifts = { [0 63], 30,     50,     25,     [],  43,  62,  0
                       56,     [0 61], 50,     23,     0,   [],  37,  26
                       16,     0,      [0 55], 27,     56,  0,   [],  43
                       35,     56,     62,     [0 11], 58,  3,   0,   []  };
        otherwise
            error('ccsdsUplinkLDPCMatrix:BadLength', ...
                'CCSDS TC defines LDPC codeword lengths 128 and 512 only, not %d.', ...
                codewordLength);
    end

    [nBR, nBC] = size(shifts);
    rows = []; cols = [];
    for br = 1:nBR
        for bc = 1:nBC
            for k = shifts{br, bc}
                % Phi^k: a cyclic shift, one 1 per row at column r+k mod M.
                r = (0:M-1).';
                c = mod(r + k, M);
                rows = [rows; (br-1)*M + r + 1];   %#ok<AGROW>
                cols = [cols; (bc-1)*M + c + 1];   %#ok<AGROW>
            end
        end
    end
    H = sparse(rows, cols, true, nBR*M, nBC*M);

    % Structural self-check. These two numbers come from the code's
    % definition, and a mistyped exponent above would break one of them
    % rather than silently costing coding gain.
    if any(sum(H, 2) ~= 8)
        error('ccsdsUplinkLDPCMatrix:BadRowWeight', ...
            'Row weights are %s; the CCSDS TC LDPC codes are uniformly 8.', ...
            mat2str(unique(full(sum(H, 2))).'));
    end
    if ~all(ismember(unique(full(sum(H, 1))), [3 5]))
        error('ccsdsUplinkLDPCMatrix:BadColWeight', ...
            'Column weights are %s; expected 3 and 5.', ...
            mat2str(unique(full(sum(H, 1))).'));
    end

    decCfg = ldpcDecoderConfig(H);

    cachedH = H;
    cachedCfg = decCfg;
    cachedN = codewordLength;
end
