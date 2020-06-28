function opmodeCallback(~, msg_mode)

    global crane;
    global opmode;
    global end_eff_pos;
    global placed_objs;
    global q_des;
    global q0;
    
    new_mode = msg_mode.Data;
    if ~(strcmp(new_mode, 'pick') || strcmp(new_mode, 'place') || strcmp(new_mode, 'rest'))
        fprintf('Unsupported operating mode: %s\n', new_mode);
        return;
    end
    
    % When moving from 'PLACE' to another state, detach the obj from EE
    if (strcmp(opmode, 'place') && ~strcmp(new_mode, 'place'))
        placed_objs = [placed_objs;
            [end_eff_pos(1) end_eff_pos(2) end_eff_pos(3)]];
    end
    
    % Add 1kg payload when lifting the object
    if strcmp(new_mode, 'place')
        crane.payload(1, [0 0 0]);
    else
        crane.payload(0, [0 0 0]);
    end
    
    % Return to resting position if requested
    if strcmp(new_mode, 'rest')
        q_des = q0;
    end

    % Update the operating mode
    opmode = new_mode;
end
