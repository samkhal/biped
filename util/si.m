% Sam Khalandovsky
% Easy conversion to SI units for common imperial units

%% input
% val: value to convert
% unit: standard abbreviated name of unit to convert from

function si_value = si(val,unit)
% conv stores conversion factors to SI
persistent conv; 
if isempty(conv) 
    conv = containers.Map;
    conv('in') = 0.0254;
    conv('ft') = 0.3048;
    conv('lbf') = 4.4482;
    conv('lbf/in') = conv('lbf')/conv('in');
    conv('lbf*in') = conv('lbf')*conv('in');
end

try
    si_value = conv(unit)*val;
catch 
    error(['unit "' unit '" not found']);
end