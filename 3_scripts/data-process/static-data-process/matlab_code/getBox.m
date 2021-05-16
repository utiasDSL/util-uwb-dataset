function box = getBox(center, lwd)
    % compute the [x_min, x_max, y_min, y_max, z_min, z_max] from center
    % and lwd
    x = center(1);         y = center(2);         z=center(3);
    x_half = 0.5 * lwd(1); y_half = 0.5 * lwd(2); z_half = 0.5 * lwd(3);
    box = [x-x_half, x+x_half, y-y_half, y+y_half, z-z_half, z+z_half];
    
end