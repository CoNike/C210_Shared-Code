    function w_skew=skew0(w)
        %�Ա�ķ��Գƾ���Ϊ������skew����Ϊskew0
        if length(w)~=3
            error('������������')
        end
        w_skew=[0   -w(3)   w(2);
            w(3) 0      -w(1);
            -w(2)   w(1)    0];
    end%���Գƾ���