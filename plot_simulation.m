function plot_simulation(Nagents,Active_Agents,navMemory,x,kk,cloud,t,numberOfCollision)
labels = cellstr( num2str([1:Nagents]') );
labels = strcat('Agent',labels);

    f(1) = plot(x(1,:),x(2,:),'*');
    hold on
    for mm = 1 : Nagents
        if navMemory{mm}.personalCollision == 1
%             plot(x(1,mm),x(2,mm),'ro','MarkerSize',25);
        end
%         plot(navMemory{mm}.inFrontX, navMemory{mm}.inFrontY,'g-')
%         plot(navMemory{mm}.behindX, navMemory{mm}.behindY,'r-')
%          plot(navMemory{mm}.FrontRightX, navMemory{mm}.FrontRightY,'r-')
        if kk > 11
            plot(navMemory{mm}.xs_1(mm,end-10:end),navMemory{mm}.xs_2(mm,end-10:end),'c-');
        end
        if mm == 1
            label1 = navMemory{mm}.navState;
        elseif mm > 1
            label1 = [label1 navMemory{mm}.navState];
        end
        text(x(1,mm)-40, x(2,mm)-40, num2str(label1(mm)));
    end
%     title({['Number of Collisions = 0']})%,num2str(numberOfCollision)]})
%     title({['Number of Collisions = ' ,num2str(numberOfCollision)]})
    title('UAVs Tracking A Pollutant Cloud')
    %set(gcf,'units','normalized','outerposition',[0 0 1 1])
    text(x(1,:)+15, x(2,:)+15, labels);
    plot(0,0,'bo','MarkerSize',40)
    text(90,90, 'Base')
    cloudplot(cloud,t);
    hold off
end