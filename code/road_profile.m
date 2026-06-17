function y = road_profile(profile,V,t,whichWheel,p)
% Road excitation profiles: a half-cosine "bump" (speed bump) and a
% trapezoidal "cushion" speed table.
if t < 0
    y = 0;
    return;
end

switch lower(profile)
    case 'bump'
        h = 0.1;
        L = 0.3;
        x = V * t; % wave coordinate relative to wheel hitting bump at t=0
        if x >= 0 && x <= L
            y = (h/2)*(1 - cos(2*pi*x/L));
        else
            y = 0;
        end
    case 'cushion'
        h = 0.26;
        a = 0.26; b = 1.787; c = 2.382;
        x = V * t;
        if x < 0
            y = 0;
        elseif x <= a
            y = (h/a)*x;
        elseif x <= a+b
            y = h;
        elseif x <= a+b+c
            y = h*(1 - (x - (a+b))/c);
        else
            y = 0;
        end
    otherwise
        y = 0;
end
end
