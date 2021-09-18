function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    d_order = 4;
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        % 很显然该段代码只支持minimum snap，对于别的阶数不适用
        Q_k = zeros(n_order+1);
        
        for i=d_order:n_order
            for l=d_order:n_order
                if i <= l
                    Q_k(i+1, l+1) = factorial(i)/factorial(i-4)*...
                        factorial(l)/factorial(l-4)*...
                        ts(k)^(3-2*4)/(i+l-7);
                else
                        Q_k(i+1, l+1) = Q_k(l+1, i+1);
                end
            end
        end

        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end