function traj=mytraj(beginning,ending,t)

    traj=zeros(length(t),3);
    traj(:,1)=quintic_polynomial(beginning(1),ending(1),t);
    traj(:,2)=quintic_polynomial(beginning(2),ending(2),t);
    traj(:,3)=quintic_polynomial(beginning(3),ending(3),t);

end