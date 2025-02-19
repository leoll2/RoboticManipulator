function [] = show_possible_movements(crane, q0)
    q = q0;
    L1 = crane.links(1);
    L2 = crane.links(2);
    L3 = crane.links(3);
    L4 = crane.links(4);
    L5 = crane.links(5);
    L6 = crane.links(6);
    % Rotate
    for r = 0:20
        q(1) = r*(pi/10);
        crane.plot(q, 'floorlevel', 0)
    end
    % Boom
    for j = q0(2):pi/20:L2.qlim(2)
        q(2) = j;
        crane.plot(q, 'floorlevel', 0)
    end
    for j = L2.qlim(2):-pi/20:L2.qlim(1)
        q(2) = j;
        crane.plot(q, 'floorlevel', 0)
    end
    for j = L2.qlim(1):pi/20:q0(2)
        q(2) = j;
        crane.plot(q, 'floorlevel', 0)
    end
    % Jib
    for j = q0(3):pi/20:L3.qlim(2)
        q(3) = j;
        crane.plot(q, 'floorlevel', 0)
    end
    for j = L3.qlim(2):-pi/20:L3.qlim(1)
        q(3) = j;
        crane.plot(q, 'floorlevel', 0)
    end
    for j = L3.qlim(1):pi/20:q0(3)
        q(3) = j;
        crane.plot(q, 'floorlevel', 0)
    end
end