% Useful for clearing java path: javaclasspath({})

javaaddpath('/home/sam/drake-distro/build/install/share/java/lcm.jar')
javaaddpath('build/obj/types.jar')

lc = lcm.lcm.LCM.getSingleton();

disp('Sending blink command')

msg = lcmtypes.blink_command;
msg.command = lcmtypes.blink_command.ON;

lc.publish('BLINK_COMMAND', msg);

% Wait for response
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('BLINK_COUNT', aggregator);
disp waiting

while true
    
    millis_to_wait = 500;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        decoded = lcmtypes.blink_count(msg.data);
        disp(['Blink count: ' num2str(decoded.count)]);
    end
end