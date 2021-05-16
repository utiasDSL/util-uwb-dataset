function los = detect_los(edge, box)
    % detect if the line segment (edge) intersect with the box (obstacle)
    % return true  --> line-of-sight
    %        false --> none-line-of-sight
    clip = clipEdge3d(edge, box);   %clipEdge3d is a function in geom3d

    if sum(isnan(clip)) ==6
        los = true;
    else
        los = false;
    end
    
end