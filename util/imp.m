% Sam Khalandovsky
% Easy conversion from SI units to imperial units (in, lbf)

%% input
% val: value to convert
% unit: standard abbreviated name of unit to convert from

function si_value = imp(val,unit)
% conv stores conversion factors to SI
persistent conv; 
if isempty(conv) 
    conv = containers.Map;
    conv('m') = 1/si(1,'in');
    conv('N') = 1/si(1,'lbf');
    conv('N/m') = conv('N')/conv('m');
    conv('N*m') = conv('N')*conv('m');
end

try
    si_value = conv(unit)*val;
catch 
    error(['unit "' unit '" not found']);
end