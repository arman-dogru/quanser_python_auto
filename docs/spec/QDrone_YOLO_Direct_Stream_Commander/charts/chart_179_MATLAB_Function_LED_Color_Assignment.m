function value = fcn(arm, low_battery)

if low_battery
    value = 1;
elseif arm
    value = 2;
else 
    value = 3;
end
