function pdf_value = negative_lognorm_pdf(x)
    % fix the lognorm mu and std
    % x should be negative
    if x >= 0
        pdf_value = 0;
    else
        log_norm = Lognormal(-3.002,1.3778);
        x_neg = -x;
        pdf_value = log_norm.PDF(x_neg); 
    end
end