function [output] = compr(A,B,mode)
    if mode == 1
        if A > B
            output = A;
        else
            output = B;
        end
    else
        if A < B
            output = A;
        else
            output = B;
        end
    end
end