function opmodeCallback(~, msg_mode)

    global crane;
    global opmode;
    global end_eff_pos;
    global placed_objs;
    
    new_mode = msg_mode.Data;
    if ~(strcmp(new_mode, 'pick') || (strcmp(new_mode, 'place')))
        fprintf('Unsupported operating mode: %s\n', new_mode);
        return;
    end
    
    if (strcmp(opmode, 'place') && strcmp(new_mode, 'pick'))
        placed_objs = [placed_objs; 
            [end_eff_pos(1) end_eff_pos(2) end_eff_pos(3)]];
    end
    
    % Add 1kg payload when lifting the object
    if (strcmp(opmode, 'pick') && strcmp(new_mode, 'place'))
        crane.payload(1, [0 0 0]);
    else
        crane.payload(0, [0 0 0]);
    end

    % Update the operating mode
    opmode = new_mode;
end
