function [roll_acc, pitch_acc]   = fcn(acc_body)
%#eml
n = norm(acc_body,2);
if n>eps
    acc_body = acc_body/n; 
else
    acc_body = [0 0 1]';
end

roll_acc = atan2(acc_body(2),acc_body(3));
pitch_acc = atan2( -acc_body(1), acc_body(2)*sin(roll_acc)+acc_body(3)*cos(roll_acc));
