function [retv] = clampv(v, minv, maxv)
    assert(length(minv) == length(v));
    assert(length(maxv) == length(v));
    
    retv = v;
    for i = 1:length(v)
        retv(i) = min(max(v(i), minv(i)), maxv(i));
    end
end
