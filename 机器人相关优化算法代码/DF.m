clc;
clear all;
syms x;
f =  @(x) sin(x).* cos(x) .* exp(sin(x)) .*sin(5*x) .* sqrt(1/2 * x.*x);
x_low = -10;
x_upper = 10;
x_number = 529;
x = zeros(1,x_number);
v = zeros(1,x_number);
u = zeros(1,x_number);
scale_factor = 0.01; % 0.01缩放
cross_rate = 0.1;% 0.1 交叉
result_last = zeros(1,x_number);
result_now = zeros(1,x_number); 
tolerence = 1e-2;
epoch = 1;
result = zeros(1,100);
%初始化评价
for index=1:1:x_number
    x(index) = x_low + rand(1) * (x_upper - x_low);
end
%
while(epoch < 100)
        v = ceil(x_number * rand(1,x_number)) + scale_factor * (ceil(x_number * rand(1,x_number)) - ceil(x_number * rand(1,x_number)));
        temp = reshape(rand(sqrt(x_number)),1,x_number) - cross_rate * ones(1,x_number);
        for index=1:1:x_number
            if(temp(index) < 0)
                 u(index) = v(index);
            else
                 u(index) = x(index);
            end
        end
        for index=1:1:x_number
            if(f(x(index)) >= f(u(index)))
                x(index) = u(index);
            else
                x(index) = x(index);
            end 
        end
        epoch = epoch + 1;
        result(epoch) = min(f(x));
     
end
subplot(121);
ezplot(f,[-10,10]);
subplot(122);
plot(result);
disp(min(f(x)));
