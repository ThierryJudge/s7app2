figure()
title('Exemple graphique animé') 
for i =1:20
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z') 
    axis([0 20 0 20 0 20])
    hold on
    scatter3(i, i, i, 'O')
    hold off
    pause(1/8)
end