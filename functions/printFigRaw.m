function printFigRaw(hax,name)
    return
    hfig = figure;
    hax_new = copyobj(hax, hfig);
    %leg = findobj(hfig,'type','legend');
    set(hax_new, 'Position', get(0, 'DefaultAxesPosition'));
    %legend(leg,'Location', 'Best')

    print(hfig, '-dpng', name)
    print(hfig, '-dsvg', name)
    print(hfig, '-dpdf', name)
    savefig(hfig,name+".fig")
    close(hfig)
    
end