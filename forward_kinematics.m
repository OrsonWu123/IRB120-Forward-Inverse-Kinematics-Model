function T=forward_kinematics(robot,q)
    i=robot.n;
    T=eye(4);

    if(robot.mdh==0)

        for j=1:1:i
            
            T=T*[cos(robot.offset(j)+q(j))   -cos(robot.alpha(j))*sin(robot.offset(j)+q(j))   sin(robot.alpha(j))*sin(robot.offset(j)+q(j))                                                           robot.a(j)*cos(robot.offset(j)+q(j))
                 sin(robot.offset(j)+q(j))   cos(robot.alpha(j))*cos(robot.offset(j)+q(j))    sin(robot.offset(j))*sin(q(j))*sin(robot.alpha(j))-cos(robot.offset(j))*cos(q(j))*sin(robot.alpha(j))   robot.a(j)*sin(robot.offset(j)+q(j))
                 0                           sin(robot.alpha(j))                              cos(robot.alpha(j))                                                                                     robot.d(j)
                 0                           0                                                0                                                                                                       1
            ];
        end

    elseif(robot.mdh==1)%unfinished
        T=T*[cos(robot.offset(1))  -sin(robot.offset(1))  0  0
                 sin(robot.offset(1))  cos(robot.offset(1))   0  0
                 0                       0                    1  0
                 0                       0                    0  1
            ];
        T=T*[cos(q(1))    -sin(q(1))    0    0
             sin(q(1))    cos(q(1))     0    0
             0            0             1    robot.d(1)
             0            0             0    1
            ];
        
        for j=2:1:i
%{
            T=T*[cos(robot.offset(j)+q(j))                         -sin(robot.offset(j)+q(j))                        0                        robot.a(j-1)
                 cos(robot.alpha(j-1))*sin(robot.offset(j)+q(j))   cos(robot.alpha(j-1))*cos(robot.offset(j)+q(j))   -sin(robot.alpha(j-1))   -sin(robot.alpha(j-1))*robot.d(j)
                 sin(robot.alpha(j-1))*sin(robot.offset(j)+q(j))   sin(robot.alpha(j-1))*cos(robot.offset(j)+q(j))   cos(robot.alpha(j-1))    cos(robot.alpha(j-1))*robot.d(j)
                 0                                                   0                                                   0                        1
            ];
        %}
            T=T*[1    0                        0                         robot.a(j-1)
                 0    cos(robot.alpha(j-1))    -sin(robot.alpha(j-1))    0
                 0    sin(robot.alpha(j-1))    cos(robot.alpha(j-1))     0
                 0    0                        0                         1
            ];
            T=T*[cos(robot.offset(j))  -sin(robot.offset(j))  0  0
                 sin(robot.offset(j))  cos(robot.offset(j))   0  0
                 0                       0                    1  0
                 0                       0                    0  1
            ];
            T=T*[cos(q(j))  -sin(q(j))  0  0
                 sin(q(j))  cos(q(j))   0  0
                 0          0           1  0
                 0          0           0  1
            ];

            
            T=T*[1  0  0  0
                 0  1  0  0
                 0  0  1  robot.d(j)
                 0  0  0  1
            ];

        end
    end
end