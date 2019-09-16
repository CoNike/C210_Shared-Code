    function w_skew=skew0(w)
        %自编的反对称矩阵，为区别与skew命名为skew0
        if length(w)~=3
            error('输入数据有误')
        end
        w_skew=[0   -w(3)   w(2);
            w(3) 0      -w(1);
            -w(2)   w(1)    0];
    end%反对称矩阵